import torch
import torch.nn as nn

from diffusion_policy.policy.act_umi_policy import ACTUmiPolicy


class TinyObsEncoder(nn.Module):
    low_dim_keys = ["robot0_eef_pos", "robot0_gripper_width"]

    def __init__(self):
        super().__init__()
        self.token = nn.Parameter(torch.zeros(1))

    def output_shape(self):
        return (1, 5, 32)

    def forward(self, obs):
        batch_size = next(iter(obs.values())).shape[0]
        return torch.zeros(batch_size, 5, 32, device=self.token.device) + self.token


def _shape_meta():
    return {
        "obs": {
            "camera0_rgb": {"shape": [3, 8, 8], "horizon": 2, "type": "rgb"},
            "robot0_eef_pos": {"shape": [3], "horizon": 2, "type": "low_dim"},
            "robot0_gripper_width": {"shape": [1], "horizon": 2, "type": "low_dim"},
        },
        "action": {"shape": [10], "horizon": 4},
    }


def test_act_umi_shapes_and_loss():
    policy = ACTUmiPolicy(
        shape_meta=_shape_meta(),
        obs_encoder=TinyObsEncoder(),
        chunk_size=4,
        dim_model=32,
        n_heads=4,
        dim_feedforward=64,
        n_encoder_layers=1,
        n_decoder_layers=1,
        n_vae_encoder_layers=1,
        latent_dim=8,
    )

    # Use identity normalizers so the policy can be exercised without loading a dataset.
    from diffusion_policy.model.common.normalizer import SingleFieldLinearNormalizer

    for key in ["camera0_rgb", "robot0_eef_pos", "robot0_gripper_width", "action"]:
        policy.normalizer[key] = SingleFieldLinearNormalizer.create_identity()

    obs = {
        "camera0_rgb": torch.rand(2, 2, 3, 8, 8),
        "robot0_eef_pos": torch.rand(2, 2, 3),
        "robot0_gripper_width": torch.rand(2, 2, 1),
    }
    action = torch.rand(2, 4, 10)

    policy.train()
    loss = policy({"obs": obs, "action": action})
    assert loss.ndim == 0
    loss.backward()

    policy.eval()
    result = policy.predict_action(obs)
    assert result["action_pred"].shape == (2, 4, 10)
