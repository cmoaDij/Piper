"""Action Chunking Transformer policy adapted to the UMI data contract.

The model follows the LeRobot ACT design: a conditional Transformer decoder
predicts an action chunk, while a small VAE encoder supplies a latent during
training.  UMI observations are kept in their native dictionary format and the
policy exposes the same ``predict_action``/``action_pred`` interface as the
existing diffusion policies.
"""

from typing import Dict, Optional, Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from diffusion_policy.model.common.normalizer import LinearNormalizer
from diffusion_policy.policy.base_image_policy import BaseImagePolicy


class ACTUmiTransformer(nn.Module):
    """Transformer and VAE components used by :class:`ACTUmiPolicy`."""

    def __init__(
        self,
        obs_tokens: int,
        obs_dim: int,
        state_dim: int,
        action_dim: int,
        chunk_size: int,
        dim_model: int = 512,
        n_heads: int = 8,
        dim_feedforward: int = 2048,
        n_encoder_layers: int = 4,
        n_decoder_layers: int = 1,
        n_vae_encoder_layers: int = 4,
        latent_dim: int = 32,
        dropout: float = 0.1,
        use_vae: bool = True,
    ):
        super().__init__()
        if dim_model % n_heads != 0:
            raise ValueError("dim_model must be divisible by n_heads")

        self.obs_tokens = obs_tokens
        self.obs_dim = obs_dim
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.chunk_size = chunk_size
        self.dim_model = dim_model
        self.latent_dim = latent_dim
        self.use_vae = use_vae

        self.obs_proj = (
            nn.Identity() if obs_dim == dim_model else nn.Linear(obs_dim, dim_model)
        )
        self.state_proj = nn.Linear(state_dim, dim_model)
        self.latent_proj = nn.Linear(latent_dim, dim_model)
        self.action_proj = nn.Linear(action_dim, dim_model)

        encoder_layer = nn.TransformerEncoderLayer(
            d_model=dim_model,
            nhead=n_heads,
            dim_feedforward=dim_feedforward,
            dropout=dropout,
            batch_first=True,
            norm_first=False,
            activation="relu",
        )
        self.encoder = nn.TransformerEncoder(encoder_layer, n_encoder_layers)

        decoder_layer = nn.TransformerDecoderLayer(
            d_model=dim_model,
            nhead=n_heads,
            dim_feedforward=dim_feedforward,
            dropout=dropout,
            batch_first=True,
            norm_first=False,
            activation="relu",
        )
        self.decoder = nn.TransformerDecoder(decoder_layer, n_decoder_layers)

        self.action_queries = nn.Parameter(torch.randn(1, chunk_size, dim_model))
        self.action_head = nn.Linear(dim_model, action_dim)

        # The VAE encoder consumes [CLS, current_state, action_chunk].
        if use_vae:
            vae_layer = nn.TransformerEncoderLayer(
                d_model=dim_model,
                nhead=n_heads,
                dim_feedforward=dim_feedforward,
                dropout=dropout,
                batch_first=True,
                norm_first=False,
                activation="relu",
            )
            self.vae_encoder = nn.TransformerEncoder(
                vae_layer, n_vae_encoder_layers
            )
            self.vae_cls = nn.Parameter(torch.randn(1, 1, dim_model))
            self.vae_head = nn.Linear(dim_model, latent_dim * 2)
            self.vae_pos = nn.Parameter(
                torch.randn(1, chunk_size + 2, dim_model)
            )
        else:
            self.vae_encoder = None
            self.vae_cls = None
            self.vae_head = None
            self.vae_pos = None

        # [latent, state, observation tokens] is the encoder memory.
        self.condition_pos = nn.Parameter(
            torch.randn(1, obs_tokens + 2, dim_model)
        )
        self._reset_parameters()

    def _reset_parameters(self):
        nn.init.normal_(self.action_queries, std=0.02)
        nn.init.normal_(self.condition_pos, std=0.02)
        if self.vae_pos is not None:
            nn.init.normal_(self.vae_pos, std=0.02)
        if self.vae_cls is not None:
            nn.init.normal_(self.vae_cls, std=0.02)

    def _encode_latent(
        self, state: torch.Tensor, actions: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        batch_size = state.shape[0]
        cls = self.vae_cls.expand(batch_size, -1, -1)
        state_token = self.state_proj(state).unsqueeze(1)
        action_tokens = self.action_proj(actions)
        vae_input = torch.cat([cls, state_token, action_tokens], dim=1)
        vae_input = vae_input + self.vae_pos[:, : vae_input.shape[1]]
        vae_output = self.vae_encoder(vae_input)[:, 0]
        mu, logvar = self.vae_head(vae_output).chunk(2, dim=-1)
        return mu, logvar.clamp(min=-10.0, max=10.0)

    def forward(
        self,
        obs_tokens: torch.Tensor,
        state: torch.Tensor,
        actions: Optional[torch.Tensor] = None,
    ) -> Tuple[torch.Tensor, Tuple[Optional[torch.Tensor], Optional[torch.Tensor]]]:
        batch_size = obs_tokens.shape[0]
        obs_tokens = self.obs_proj(obs_tokens)

        mu = logvar = None
        if self.use_vae and self.training and actions is not None:
            mu, logvar = self._encode_latent(state, actions)
            latent = mu + torch.exp(0.5 * logvar) * torch.randn_like(mu)
        else:
            latent = torch.zeros(
                batch_size,
                self.latent_dim,
                dtype=obs_tokens.dtype,
                device=obs_tokens.device,
            )

        condition = torch.cat(
            [
                self.latent_proj(latent).unsqueeze(1),
                self.state_proj(state).unsqueeze(1),
                obs_tokens,
            ],
            dim=1,
        )
        condition = condition + self.condition_pos[:, : condition.shape[1]]
        memory = self.encoder(condition)

        queries = self.action_queries.expand(batch_size, -1, -1)
        decoded = self.decoder(queries, memory)
        action_hat = self.action_head(decoded)
        return action_hat, (mu, logvar)


class ACTUmiPolicy(BaseImagePolicy):
    """LeRobot-style ACT policy with UMI observations and actions."""

    def __init__(
        self,
        shape_meta: dict,
        obs_encoder: nn.Module,
        chunk_size: Optional[int] = None,
        dim_model: int = 512,
        n_heads: int = 8,
        dim_feedforward: int = 2048,
        n_encoder_layers: int = 4,
        n_decoder_layers: int = 1,
        n_vae_encoder_layers: int = 4,
        latent_dim: int = 32,
        dropout: float = 0.1,
        use_vae: bool = True,
        kl_weight: float = 10.0,
        **kwargs,
    ):
        super().__init__()
        action_shape = tuple(shape_meta["action"]["shape"])
        if len(action_shape) != 1:
            raise ValueError(f"ACT expects a flat action shape, got {action_shape}")

        self.shape_meta = shape_meta
        self.obs_encoder = obs_encoder
        self.normalizer = LinearNormalizer()
        self.action_dim = int(action_shape[0])
        self.chunk_size = int(chunk_size or shape_meta["action"]["horizon"])
        self.kl_weight = float(kl_weight)
        self.state_keys = list(getattr(obs_encoder, "low_dim_keys", []))
        self.state_dim = int(
            sum(np.prod(shape_meta["obs"][key]["shape"]) for key in self.state_keys)
        )
        if self.state_dim <= 0:
            raise ValueError("UMI ACT requires at least one low-dimensional observation")

        with torch.no_grad():
            obs_shape = obs_encoder.output_shape()
        if len(obs_shape) != 3:
            raise ValueError(f"Expected observation tokens [B,N,D], got {obs_shape}")
        obs_tokens, obs_dim = int(obs_shape[-2]), int(obs_shape[-1])
        self.model = ACTUmiTransformer(
            obs_tokens=obs_tokens,
            obs_dim=obs_dim,
            state_dim=self.state_dim,
            action_dim=self.action_dim,
            chunk_size=self.chunk_size,
            dim_model=dim_model,
            n_heads=n_heads,
            dim_feedforward=dim_feedforward,
            n_encoder_layers=n_encoder_layers,
            n_decoder_layers=n_decoder_layers,
            n_vae_encoder_layers=n_vae_encoder_layers,
            latent_dim=latent_dim,
            dropout=dropout,
            use_vae=use_vae,
        )
        self.kwargs = kwargs

    def _extract_state(self, obs_dict: Dict[str, torch.Tensor]) -> torch.Tensor:
        state_values = []
        for key in self.state_keys:
            value = obs_dict[key]
            state_values.append(value[:, -1].reshape(value.shape[0], -1))
        return torch.cat(state_values, dim=-1)

    def set_normalizer(self, normalizer: LinearNormalizer):
        self.normalizer.load_state_dict(normalizer.state_dict())

    def get_optimizer(
        self,
        lr: float,
        weight_decay: float,
        obs_encoder_lr: float,
        obs_encoder_weight_decay: float,
        betas: Tuple[float, float],
    ) -> torch.optim.Optimizer:
        # Keep the same fine-tuning convention as the existing UMI policies:
        # vision backbones use a smaller learning rate, while the ACT head,
        # projection layers, and non-backbone encoder parameters use ``lr``.
        backbone_params = []
        other_obs_params = []
        for name, param in self.obs_encoder.named_parameters():
            if not param.requires_grad:
                continue
            if name.startswith("key_model_map"):
                backbone_params.append(param)
            else:
                other_obs_params.append(param)

        encoder_ids = {id(param) for param in self.obs_encoder.parameters()}
        act_params = [
            param
            for param in self.parameters()
            if param.requires_grad and id(param) not in encoder_ids
        ]
        optim_groups = [
            {
                "params": act_params,
                "lr": lr,
                "weight_decay": weight_decay,
            },
            {
                "params": backbone_params,
                "lr": obs_encoder_lr,
                "weight_decay": obs_encoder_weight_decay,
            },
            {
                "params": other_obs_params,
                "lr": lr,
                "weight_decay": obs_encoder_weight_decay,
            },
        ]
        return torch.optim.AdamW(optim_groups, betas=betas)

    @torch.no_grad()
    def predict_action(self, obs_dict: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        nobs = self.normalizer.normalize(obs_dict)
        state = self._extract_state(nobs)
        obs_tokens = self.obs_encoder(nobs)
        action_hat, _ = self.model(obs_tokens, state, actions=None)
        action_pred = self.normalizer["action"].unnormalize(action_hat)
        return {"action": action_pred, "action_pred": action_pred}

    def compute_loss(self, batch: Dict[str, torch.Tensor]) -> torch.Tensor:
        nobs = self.normalizer.normalize(batch["obs"])
        nactions = self.normalizer["action"].normalize(batch["action"])
        state = self._extract_state(nobs)
        obs_tokens = self.obs_encoder(nobs)
        action_hat, (mu, logvar) = self.model(obs_tokens, state, actions=nactions)

        l1_loss = F.l1_loss(action_hat, nactions)
        if mu is None or logvar is None:
            return l1_loss
        kld = (-0.5 * (1 + logvar - mu.pow(2) - logvar.exp())).sum(-1).mean()
        return l1_loss + self.kl_weight * kld

    def forward(self, batch: Dict[str, torch.Tensor]) -> torch.Tensor:
        return self.compute_loss(batch)
