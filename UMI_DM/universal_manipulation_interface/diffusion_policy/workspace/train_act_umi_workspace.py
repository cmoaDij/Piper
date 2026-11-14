"""Training workspace for the UMI ACT policy.

The existing transformer workspace already provides the UMI dataset,
normalizer, EMA, checkpointing, and rollout plumbing.  ACT follows the same
policy contract, so this named workspace keeps the training entry point clear
without duplicating that infrastructure.
"""

from diffusion_policy.workspace.train_diffusion_transformer_timm_workspace import (
    TrainDiffusionTransformerTimmWorkspace,
)


class TrainActUmiWorkspace(TrainDiffusionTransformerTimmWorkspace):
    pass
