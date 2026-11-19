import json
import threading
from urllib.request import build_opener, ProxyHandler, Request

from data_collect.common.config import local_path
from data_collect.web.dashboard import DemoBridge, LaunchSupervisor, make_server, reuse_dashboard


def test_demo_serves_static_files_models_and_state_without_hardware(config):
    bridge = DemoBridge(config)
    supervisor = LaunchSupervisor(bridge, local_path("config/collection.yaml"), demo=True)
    server = make_server(config, local_path("config/collection.yaml"), bridge, supervisor, 0, demo=True, can_port="can7")
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    opener = build_opener(ProxyHandler({}))
    url = f"http://127.0.0.1:{server.server_port}"
    try:
        for path in ("/", "/static/app.js", "/static/robot.js", "/static/vendor/three.module.js", "/model/0"):
            with opener.open(url + path, timeout=3) as response:
                assert response.status == 200 and response.read()
        with opener.open(url + "/api/boot", timeout=3) as response:
            boot = json.load(response)
        assert boot["demo"] and len(boot["model"]["links"]) > 6
        assert "safety" not in boot and boot["teleop"]["fast_mode"] is True
        assert boot["can_port"] == "can7"
        # Reusing a GUI starts its original supervisor, without a second service.
        assert reuse_dashboard(server.server_port, local_path("config/collection.yaml"), True, True, True)
        started_at = supervisor.started_at
        assert reuse_dashboard(server.server_port, local_path("config/collection.yaml"), True, True, True)
        assert supervisor.started_at == started_at
        with opener.open(url + "/api/state", timeout=3) as response:
            state = json.load(response)
        assert state["process"]["running"] and supervisor.process is None
        supervisor.stop()
        with opener.open(url, timeout=3) as response:
            import re
            html = response.read().decode()
            assert 'id="coordinate-mode"' in html
            token = re.search(r'<meta name="control-token" content="([^"]+)">', html).group(1)
        request = Request(url + "/api/start", data=json.dumps({"can_port": "can7", "coordinate_mode": "world"}).encode(),
                          headers={"Content-Type": "application/json", "X-Control-Token": token}, method="POST")
        with opener.open(request, timeout=3) as response:
            assert response.status == 200
        assert supervisor.snapshot()["coordinate_mode"] == "world"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)
        supervisor.close()
        bridge.close()
