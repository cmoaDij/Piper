"""Reference images must be episode starts from the stored training images."""
import numpy as np
from pathlib import Path
import tempfile
import zarr

from umi.real_world.piper_training_reference import load_training_reference


def check_episode_starts(tmp_path, zipped):
    path = tmp_path / ('dataset.zarr.zip' if zipped else 'dataset.zarr')
    store = zarr.ZipStore(str(path), mode='w') if zipped else zarr.DirectoryStore(str(path))
    images = np.zeros((5, 8, 10, 3), np.uint8)
    images[0] = [255, 0, 0]
    images[2] = [0, 0, 255]
    try:
        group = zarr.group(store)
        group.create_dataset('meta/episode_ends', data=np.array([2, 5]))
        group.create_dataset('data/camera0_rgb', data=images, chunks=(1, 8, 10, 3))
    finally:
        store.close()
    reference = load_training_reference(path)
    assert reference.episode_starts == [0, 2]
    np.testing.assert_array_equal(reference.images_rgb, images[[0, 2]])
    assert reference.metadata()['image_size'] == [10, 8]
    assert not reference.images_rgb.flags.writeable
    for kwargs, message in [({'episode': 2}, 'match-episode'), ({'alpha': 1.5}, 'match-alpha')]:
        try:
            reference.metadata(**kwargs)
        except ValueError as exc:
            assert message in str(exc)
        else:
            raise AssertionError('Invalid reference selection must be rejected')


def test_training_episode_starts():
    with tempfile.TemporaryDirectory() as folder:
        for zipped in (False, True):
            check_episode_starts(Path(folder), zipped)


if __name__ == '__main__':
    test_training_episode_starts()
    print('Training reference: directory/ZIP episode starts, RGB and selection validation passed')
