import json
import threading
from urllib.request import build_opener, ProxyHandler

from data_collect.common.config import local_path
from data_collect.web.dashboard import DemoBridge, LaunchSupervisor, make_server


def test_demo_serves_static_files_models_and_state_without_hardware(config):
    bridge = DemoBridge(config)
    supervisor = LaunchSupervisor(bridge, local_path("config/collection.yaml"), demo=True)
    server = make_server(config, local_path("config/collection.yaml"), bridge, supervisor, 0, demo=True)
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
        supervisor.start("can0")
        with opener.open(url + "/api/state", timeout=3) as response:
            state = json.load(response)
        assert state["process"]["running"] and supervisor.process is None
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)
        supervisor.close()
        bridge.close()
