"""Load episode first frames for optional visual start-state alignment."""
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass
class TrainingReference:
    dataset: str
    images_rgb: np.ndarray
    episode_starts: list
    camera_key: str = 'camera0_rgb'

    def metadata(self, episode=0, alpha=.5):
        if not 0 <= episode < len(self.images_rgb):
            raise ValueError(f'--match-episode must be between 0 and {len(self.images_rgb) - 1}')
        if not np.isfinite(alpha) or not 0 <= alpha <= 1:
            raise ValueError('--match-alpha must be between 0 and 1')
        return {'enabled': True, 'dataset': self.dataset, 'camera_key': self.camera_key,
                'episode_count': len(self.images_rgb), 'episode_starts': self.episode_starts,
                'initial_episode': int(episode), 'initial_alpha': float(alpha),
                'image_size': list(self.images_rgb.shape[2:0:-1]),
                'note': 'Reference and live frames share the model input crop. '
                        'Blending is display-only and happens in the browser.'}

    def save(self, output, episode=0, alpha=.5):
        import json
        output = Path(output)
        output.mkdir(parents=True, exist_ok=True)
        metadata = self.metadata(episode, alpha)
        (output / 'training_reference.json').write_text(json.dumps(metadata, indent=2) + '\n')
        path = output / 'training_reference_initial.png'
        if not cv2.imwrite(str(path), cv2.cvtColor(self.images_rgb[episode], cv2.COLOR_RGB2BGR)):
            raise OSError(f'Cannot write {path}')


def load_training_reference(dataset, camera_key='camera0_rgb'):
    """Read only each episode's first stored image; never load an entire video."""
    import zarr
    from diffusion_policy.codecs.imagecodecs_numcodecs import register_codecs
    register_codecs(verbose=False)
    path = Path(dataset).resolve()
    if not path.exists():
        raise ValueError(f'Training dataset does not exist: {path}')
    store = zarr.DirectoryStore(str(path)) if path.is_dir() else zarr.ZipStore(str(path), mode='r')
    try:
        group = zarr.open_group(store=store, mode='r')
        ends = np.asarray(group['meta/episode_ends'][:], dtype=np.int64)
        frames = group[f'data/{camera_key}']
        if (ends.ndim != 1 or not len(ends) or ends[0] <= 0
                or np.any(np.diff(ends) <= 0) or ends[-1] > len(frames)):
            raise ValueError('Training dataset has invalid episode boundaries')
        if len(frames.shape) != 4 or frames.shape[-1] != 3 or frames.dtype != np.uint8:
            raise ValueError('Training reference images must be uint8 RGB in (T,H,W,3) order')
        starts = np.r_[0, ends[:-1]]
        images = np.stack([frames[int(start)] for start in starts])
    finally:
        store.close()
    images.setflags(write=False)
    return TrainingReference(str(path), images, starts.tolist(), camera_key)
