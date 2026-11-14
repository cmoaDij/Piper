"""Exercise the local preview with synthetic frames; no camera is opened."""
from urllib.error import HTTPError
from urllib.request import urlopen
import json

import cv2
import numpy as np
import pytest

from umi.real_world.piper_camera_preview import CameraPreview
from umi.real_world.piper_training_reference import TrainingReference


def test_preview_serves_camera_and_model_frames():
    preview = CameraPreview(0)
    preview.start()
    url = f'http://127.0.0.1:{preview.port}'
    try:
        assert b'Model input' in urlopen(url).read()
        assert json.loads(urlopen(url + '/reference.json').read()) == {'enabled': False}
        with pytest.raises(HTTPError) as exc:
            urlopen(url + '/raw.jpg')
        assert exc.value.code == 503
        raw = np.full((48, 64, 3), (0, 0, 255), dtype=np.uint8)
        model = np.full((22, 22, 3), (255, 0, 0), dtype=np.uint8)
        preview.update(raw, model)
        raw_jpeg = np.frombuffer(urlopen(url + '/raw.jpg?t=1').read(), dtype=np.uint8)
        model_jpeg = np.frombuffer(urlopen(url + '/model.jpg').read(), dtype=np.uint8)
        assert cv2.imdecode(raw_jpeg, cv2.IMREAD_COLOR).shape == (48, 64, 3)
        assert cv2.imdecode(model_jpeg, cv2.IMREAD_COLOR).shape == (22, 22, 3)
        assert cv2.imdecode(raw_jpeg, cv2.IMREAD_COLOR)[0, 0, 2] > 240
        assert cv2.imdecode(model_jpeg, cv2.IMREAD_COLOR)[0, 0, 2] > 240
    finally:
        preview.close()


def test_reference_selection_does_not_change_live_model_input():
    first = np.full((22, 22, 3), (255, 0, 0), np.uint8)
    second = np.full((22, 22, 3), (0, 0, 255), np.uint8)
    reference = TrainingReference('/test/dataset.zarr.zip', np.stack([first, second]), [0, 10])
    preview = CameraPreview(0, training_reference=reference, match_episode=1, match_alpha=.35)
    preview.start()
    url = f'http://127.0.0.1:{preview.port}'
    live = np.full((22, 22, 3), (0, 255, 0), np.uint8)
    before = live.copy()
    preview.update(np.zeros((48, 64, 3), np.uint8), live)
    try:
        metadata = json.loads(urlopen(url + '/reference.json').read())
        assert metadata['episode_count'] == 2 and metadata['initial_episode'] == 1
        assert metadata['initial_alpha'] == .35
        for index, channel in ((0, 2), (1, 0)):
            jpeg = np.frombuffer(urlopen(url + f'/reference.jpg?episode={index}').read(), np.uint8)
            assert cv2.imdecode(jpeg, cv2.IMREAD_COLOR)[0, 0, channel] > 240
        with pytest.raises(HTTPError) as exc:
            urlopen(url + '/reference.jpg?episode=2')
        assert exc.value.code == 400
        np.testing.assert_array_equal(live, before)
        model = cv2.imdecode(np.frombuffer(urlopen(url + '/model.jpg').read(), np.uint8), cv2.IMREAD_COLOR)
        assert model[0, 0, 1] > 240
    finally:
        preview.close()
