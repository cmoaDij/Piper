import copy
import json
import threading
from types import SimpleNamespace as NS

import cv2
import numpy as np
import pytest

from data_collect.application.collector import validate_calibration
from data_collect.common.config import image_streams
from data_collect.sensors.camera_selection import connected_gopro, load_gopro_latency, resolve_cameras
from data_collect.sensors.gopro import gopro_worker
from data_collect.sensors.capture import ImageBuffer
from data_collect.recording.timeline import RecordingTimeline


DEVICE = "/dev/v4l/by-id/usb-Elgato_HD60_X_TEST-video-index0"


def test_auto_prefers_complete_realsense_profile(config):
    original = copy.deepcopy(config["cameras"])
    assert resolve_cameras(config, realsense_serials={c["serial"] for c in original.values()},
                           gopro_devices=[DEVICE]) == "realsense"
    for name, options in original.items():
        assert config["cameras"][name] == {**options, "backend": "realsense"}
    assert len(image_streams(config)) == 4


def test_auto_selects_one_gopro_without_duplicating_views(config):
    assert resolve_cameras(config, realsense_serials=set(), gopro_devices=[DEVICE]) == "gopro"
    assert list(config["cameras"]) == ["wrist_rgb"]
    assert config["cameras"]["wrist_rgb"]["serial"] == DEVICE
    assert config["calibration_id"] == "calibration_gopro_v1"
    assert len(image_streams(config)) == 3


@pytest.mark.parametrize("devices", [[], [DEVICE, DEVICE + "-other"]])
def test_auto_does_not_guess_when_gopro_is_missing_or_ambiguous(config, devices):
    with pytest.raises(ValueError, match="明确各相机位置"):
        resolve_cameras(config, realsense_serials=set(), gopro_devices=devices)


def test_discovery_excludes_laptop_and_metadata_nodes(tmp_path):
    for name in ("usb-GoPro_TEST-video-index0", "usb-Elgato_HD60_X_TEST-video-index0",
                 "usb-MACROSILICON_UGREEN_35287_86477247-video-index0",
                 "usb-MACROSILICON_UGREEN_35287_86477247-video-index1",
                 "usb-GoPro_TEST-video-index1", "usb-HP_True_Vision-video-index0"):
        (tmp_path / name).touch()
    (tmp_path / "usb-GoPro_GONE-video-index0").symlink_to(tmp_path / "missing")
    assert [p.split("/")[-1] for p in connected_gopro(tmp_path)] == [
        "usb-Elgato_HD60_X_TEST-video-index0", "usb-GoPro_TEST-video-index0",
        "usb-MACROSILICON_UGREEN_35287_86477247-video-index0"]


def test_duplicate_camera_aliases_are_rejected(config, tmp_path):
    device = tmp_path / "capture"
    device.touch()
    alias = tmp_path / "alias"
    alias.symlink_to(device)
    profile = config["camera_selection"]["gopro"]["cameras"]
    profile["wrist_rgb"]["device"] = str(device)
    profile["global_rgb"] = {**profile["wrist_rgb"], "device": str(alias)}
    with pytest.raises(ValueError, match="同一个采集设备"):
        resolve_cameras(config, realsense_serials=set())


def latency_options(tmp_path, **changes):
    measured = {"camera": DEVICE, "resolution": [1280, 960], "capture_fps": 60,
                "sample_count": 20, "nominal_latency_ms": 146.283, **changes}
    path = tmp_path / "latency.json"
    path.write_text(json.dumps(measured))
    return dict(device=DEVICE, width=1280, height=960, fps=60, receive_latency_file=str(path))


def test_loads_umi_measured_latency_without_reusing_other_devices(tmp_path):
    options = latency_options(tmp_path)
    load_gopro_latency(options)
    assert options["receive_latency_ns"] == 146_283_000
    assert len(options["receive_latency_sha256"]) == 64


@pytest.mark.parametrize("changes", [{"camera": "/dev/video99"}, {"resolution": [640, 480]},
                                     {"capture_fps": 30}, {"nominal_latency_ms": -1},
                                     {"nominal_latency_ms": float("nan")}, {"sample_count": 0}])
def test_latency_must_match_capture_path_and_mode(tmp_path, changes):
    with pytest.raises(ValueError):
        load_gopro_latency(latency_options(tmp_path, **changes))


def test_gopro_cannot_reuse_realsense_calibration(config, tmp_path):
    from data_collect.common.config import ROOT

    template = json.loads((ROOT / "config/calibration.example.json").read_text())
    template.update(verified=True, calibration_id="calibration_gopro_v1")
    path = tmp_path / "calibration.json"
    path.write_text(json.dumps(template))
    resolve_cameras(config, realsense_serials=set(), gopro_devices=[DEVICE])
    config["calibration_file"] = str(path)
    with pytest.raises(ValueError, match="设备清单与标定不符"):
        validate_calibration(config)


@pytest.mark.parametrize("reported_fps", [60, 59.94])
@pytest.mark.parametrize("deferred_resolution", [False, True])
def test_umi_gopro_worker_timestamps_rgb_and_releases_device(monkeypatch, reported_fps, deferred_resolution):
    stop = threading.Event()
    output, errors = [], []
    settings = {}
    bgr = np.zeros((2, 4, 3), dtype=np.uint8)
    bgr[..., 0] = 255

    class Capture:
        released = False

        def isOpened(self):
            return True

        def set(self, prop, value):
            settings[prop] = value
            return True

        def get(self, prop):
            # UGREEN can negotiate a different size until both requested
            # dimensions have been supplied. Check the completed mode.
            if (deferred_resolution and prop == cv2.CAP_PROP_FRAME_WIDTH
                    and cv2.CAP_PROP_FRAME_HEIGHT not in settings):
                return 1920
            return reported_fps if prop == cv2.CAP_PROP_FPS else settings[prop]

        def read(self):
            return True, bgr

        def release(self):
            self.released = True

    cap = Capture()
    monkeypatch.setattr(cv2, "VideoCapture", lambda device, backend: cap)
    monkeypatch.setattr("data_collect.sensors.gopro.time.monotonic_ns", lambda: 1_000_000_000)

    def emit(stream, record):
        output.append((stream, record))
        stop.set()

    gopro_worker("wrist_rgb", dict(device=DEVICE, width=4, height=2, fps=60, receive_latency_ns=150_000_000),
                  NS(emit=emit, fail=errors.append), stop)
    assert not errors and cap.released
    stream, record = output[0]
    assert stream == "cameras/wrist_rgb"
    assert record["available_host_ts_ns"] == 1_000_000_000
    assert record["capture_host_ts_ns"] == 850_000_000
    assert record["data"][0, 0].tolist() == [0, 0, 255]
    assert settings[cv2.CAP_PROP_BUFFERSIZE] == 1


@pytest.mark.parametrize("mismatch", [cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS])
def test_gopro_rejects_incompatible_final_video_mode(monkeypatch, mismatch):
    errors, settings = [], {}

    class Capture:
        released = False

        def isOpened(self):
            return True

        def set(self, prop, value):
            settings[prop] = value
            return True

        def get(self, prop):
            return settings[prop] + 2 if prop == mismatch else settings[prop]

        def read(self):
            pytest.fail("An incompatible video mode must not reach frame capture")

        def release(self):
            self.released = True

    cap = Capture()
    monkeypatch.setattr(cv2, "VideoCapture", lambda device, backend: cap)
    gopro_worker("wrist_rgb", dict(device=DEVICE, width=1280, height=960, fps=60),
                  NS(fail=errors.append), threading.Event())
    assert len(errors) == 1 and "未接受" in errors[0]
    assert cap.released


def test_resume_rejects_delayed_pictures_from_pause_even_when_seeded():
    timeline = RecordingTimeline(100)
    timeline.set_recording(False, 200)
    timeline.set_recording(True, 300)
    record = dict(available_host_ts_ns=350, capture_host_ts_ns=250)
    assert timeline.prepare("cameras/wrist_rgb", record, seed=True) is None
    record["capture_host_ts_ns"] = 300
    assert timeline.prepare("cameras/wrist_rgb", record)["recording_segment"] == 1


def test_gopro_resume_waits_for_new_scene_instead_of_reusing_pause(config, tmp_path):
    resolve_cameras(config, realsense_serials=set(), gopro_devices=[DEVICE])
    config["data_root"] = str(tmp_path)
    manager = ImageBuffer(config, None)
    manager.history["cameras/wrist_rgb"].append(dict(available_host_ts_ns=350, capture_host_ts_ns=250))
    # Seed only GoPro here; other SDKs are outside this test's boundary.
    manager.history = {"cameras/wrist_rgb": manager.history["cameras/wrist_rgb"]}
    records = []
    manager.seed_images(NS(submit=lambda *args, **kw: records.append(args)), 300)
    assert not records and manager.failure is None
