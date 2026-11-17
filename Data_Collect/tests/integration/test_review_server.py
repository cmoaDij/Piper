import json
import re
import threading
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

import pytest

from data_collect.cli import review
from data_collect.web.review import make_review_server
from data_collect.dataset.audit import audit_batch
from tests.fixtures.episodes import write_episode


def test_review_empty_page_names_actual_source_and_rescans_new_recording(tmp_path, config):
    raw = tmp_path / "recordings"
    audit = tmp_path / "audit"
    rules = {**config["audit"], "min_duration_s": .1}
    server = make_review_server(audit, rules, port=0, raw_dir=raw)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    opener = build_opener(ProxyHandler({}))
    url = f"http://127.0.0.1:{server.server_port}"

    def get(path):
        with opener.open(url + path, timeout=5) as response:
            return json.load(response)

    def post(path, token, body):
        request = Request(url + path, data=json.dumps(body).encode(),
                          headers={"Content-Type": "application/json", "X-Review-Token": token})
        with opener.open(request, timeout=5) as response:
            return json.load(response)

    try:
        with opener.open(url, timeout=5) as response:
            html = response.read().decode()
        assert str(raw) in html and "__RAW_DIR_JSON__" not in html
        assert "机械臂能遥操不代表数据已经写入" in html
        token = re.search(r"const token='([^']+)'", html)[1]
        assert get("/api/episodes") == []
        episode = write_episode(raw, config)
        with pytest.raises(HTTPError) as error:
            post("/api/rescan", "invalid-token", {})
        assert error.value.code == 403
        assert post("/api/rescan", token, {}) == {"episodes": 1, "approved_episodes": 0}
        rows = get("/api/episodes")
        assert rows[0]["episode"] == episode.name and rows[0]["frames"] == 20
        assert rows[0]["status"] == "PASS" and rows[0]["review"] == "pending"
        assert post("/api/decision", token, {"episode": episode.name, "decision": "approve",
                                            "operator": "test"})["approved_episodes"] == 1
        assert post("/api/rescan", token, {})["approved_episodes"] == 1
        assert get("/api/episodes")[0]["review"] == "approve"
        # An unfinished recording must be listed as such, never hidden or opened
        # as a complete HDF5 file while another process might still be writing.
        (raw / "episode_0002.h5.partial").touch()
        assert post("/api/rescan", token, {})["episodes"] == 2
        partial = get("/api/episodes")[1]
        assert partial["status"] == "REJECT" and partial["reasons"] == ["incomplete_partial_file"]
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)


@pytest.mark.parametrize("missing_gopro_segment", [False, True])
def test_rejected_paused_preview_can_reach_the_last_recorded_frame(tmp_path, config, missing_gopro_segment):
    import cv2
    import numpy as np

    raw, audit = tmp_path / "raw", tmp_path / "audit"
    if missing_gopro_segment:
        config["cameras"].pop("global_rgb")
        config["cameras"]["wrist_rgb"].update(backend="gopro", receive_latency_ns=300_000_000,
                                               receive_latency_sha256="synthetic_latency")
    episode = write_episode(raw, config, paused=True)
    # Complete file rejected only for its 0.75 s active duration; wall time is 3 s.
    rules = {**config["audit"], "min_duration_s": 1}
    reports, _ = audit_batch(raw, audit, rules)
    assert reports[0]["status"] == "REJECT"
    assert reports[0]["wall_duration_s"] == 3
    if missing_gopro_segment:
        assert "无对应 GoPro 图像" in str(reports[0]["reasons"])
        assert reports[0]["image_streams"] == ["cameras/wrist_rgb", "tactile/left_raw", "tactile/right_raw"]
    server = make_review_server(audit, rules, port=0, raw_dir=raw)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    opener = build_opener(ProxyHandler({}))
    url = f"http://127.0.0.1:{server.server_port}"
    try:
        with opener.open(url + "/api/episodes", timeout=5) as response:
            row = json.load(response)[0]
        assert row["frames"] == 60
        with opener.open(url + f"/api/frame?episode={episode.name}&k=59&stream=cameras/wrist_rgb", timeout=5) as response:
            image = cv2.imdecode(np.frombuffer(response.read(), np.uint8), cv2.IMREAD_COLOR)
        assert abs(image.mean() - 19) <= 2  # Final real MP4 frame, after the pause.
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)


def test_review_command_reports_empty_source_without_claiming_recording_succeeded(tmp_path, config, monkeypatch, capsys):
    config["data_root"] = str(tmp_path)
    served = []
    monkeypatch.setattr(review, "load_config", lambda _: config)
    monkeypatch.setattr(review, "serve_review", lambda *args, **kwargs: served.append((args, kwargs)))
    monkeypatch.setattr("sys.argv", ["review"])
    review.main()
    output = capsys.readouterr().out
    assert str(tmp_path / "raw") in output and "未发现录制文件" in output
    assert served[0][1]["raw_dir"] == tmp_path / "raw"
    assert json.loads((tmp_path / "audit/audit_report.json").read_text()) == []
