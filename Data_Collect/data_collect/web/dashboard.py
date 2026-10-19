"""Local GUI server, ROS observation and ownership of one launch process group."""

import argparse
import copy
import errno
import json
import math
import mimetypes
import os
import re
import secrets
import signal
import subprocess
import sys
import threading
import time
import webbrowser
import xml.etree.ElementTree as ET
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse
from urllib.error import URLError
from urllib.request import ProxyHandler, build_opener

from data_collect.common.config import ROOT, IMAGE_STREAMS, load_config, local_path

STATIC = Path(__file__).with_name("static")
NODES = {
    "agx_arm_ctrl_single_node": "机械臂与采集驱动",
    "arm_ik_pose_node": "IK 求解",
    "pub_pose_node": "Quest 位姿",
    "pub_delta_pose_node": "增量遥操",
}


def existing_dashboard(port):
    """Identify our local GUI before creating a second ROS participant.

    The schema check also recognizes GUI servers started before the explicit
    application identifier was added.
    """
    if port == 0:
        return None
    try:
        opener = build_opener(ProxyHandler({}))
        with opener.open(f"http://127.0.0.1:{port}/api/boot", timeout=1) as response:
            body = response.read(65537)
        if len(body) > 65536:
            return None
        boot = json.loads(body)
        if not isinstance(boot, dict):
            return None
        identified = boot.get("application") == "piper-data-collect-dashboard"
        legacy = isinstance(boot.get("nodes"), dict) and boot["nodes"] == NODES and isinstance(boot.get("model"), dict)
        if (identified or legacy) and isinstance(boot.get("demo"), bool) and isinstance(boot.get("config"), str):
            return boot
    except (URLError, OSError, ValueError):
        pass
    return None


def reuse_dashboard(port, config_path, demo, no_browser):
    boot = existing_dashboard(port)
    if boot is None:
        return False
    url = f"http://127.0.0.1:{port}/"
    if boot["demo"] != demo or Path(boot["config"]).resolve() != config_path:
        raise ValueError(f"{url} 已运行另一种模式或配置的工作台；请使用 --port 指定其他端口。")
    print(f"工作台已在运行：{url}\n直接打开已有界面，服务由原进程继续运行。", flush=True)
    if not no_browser:
        webbrowser.open(url)
    return True


def robot_description(cfg):
    """Expose only the selected URDF's existing collision meshes, never arbitrary files."""
    tree = ET.parse(local_path(cfg["urdf"])).getroot()
    assets, links, joints = {}, [], []
    package = ROOT / "ros_ws/src/agx_arm_description"

    def origin(element):
        o = element.find("origin")
        return {name: [float(v) for v in o.get(name, "0 0 0").split()] if o is not None else [0, 0, 0] for name in ("xyz", "rpy")}

    for link in tree.findall("link"):
        geometries = []
        for collision in link.findall("collision"):
            mesh = collision.find("geometry/mesh")
            if mesh is None:
                continue
            name = mesh.get("filename", "")
            prefix = "package://agx_arm_description/"
            if not name.startswith(prefix):
                raise ValueError(f"GUI 不支持模型资源：{name}")
            path = (package / name[len(prefix):]).resolve()
            if not path.is_relative_to(package) or not path.is_file() or path.suffix != ".stl":
                raise ValueError(f"模型网格无效：{path}")
            key = str(len(assets))
            assets[key] = path
            geometries.append({**origin(collision), "url": "/model/" + key, "scale": [float(v) for v in mesh.get("scale", "1 1 1").split()]})
        links.append({"name": link.get("name"), "meshes": geometries})
    for joint in tree.findall("joint"):
        axis, limit, mimic = joint.find("axis"), joint.find("limit"), joint.find("mimic")
        row = {"name": joint.get("name"), "type": joint.get("type"), "parent": joint.find("parent").get("link"), "child": joint.find("child").get("link"), **origin(joint), "axis": [float(v) for v in axis.get("xyz").split()] if axis is not None else [1, 0, 0]}
        if limit is not None:
            row["limits"] = [float(limit.get("lower", 0)), float(limit.get("upper", 0))]
        if mimic is not None:
            row["mimic"] = {"joint": mimic.get("joint"), "multiplier": float(mimic.get("multiplier", 1)), "offset": float(mimic.get("offset", 0))}
        joints.append(row)
    return {"links": links, "joints": joints, "base_link": cfg["base_link"], "tip_link": cfg["tip_link"], "tcp_offset": cfg["tcp_offset"]}, assets


class RosBridge:
    """Subscribe to small status packets; no CAN/SDK/image access from the GUI."""
    def __init__(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import String, Bool
        from geometry_msgs.msg import PoseStamped

        self.lock = threading.RLock()
        self.status, self.received, self.quest_received = {}, 0.0, 0.0
        self.nodes, self.error = [], None
        self.context = rclpy.context.Context()
        rclpy.init(args=[], context=self.context)
        self.node = Node("piper_collection_dashboard", context=self.context)
        self.stop_pub = self.node.create_publisher(Bool, "/teleop/enabled", 10)
        self.node.create_subscription(String, "/data_collect/status", self._status, 1)
        self.node.create_subscription(PoseStamped, "/right_handle_pose", self._quest, 1)
        self.node.create_timer(0.5, self._graph)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._spin, name="dashboard-ros", daemon=True)
        self.thread.start()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception as exc:
            with self.lock:
                self.error = str(exc)

    def _status(self, msg):
        try:
            row = json.loads(msg.data)
            if not isinstance(row, dict):
                return
            with self.lock:
                self.status, self.received = row, time.monotonic()
        except (ValueError, TypeError):
            pass

    def _quest(self, msg):
        with self.lock:
            self.quest_received = time.monotonic()

    def _graph(self):
        with self.lock:
            self.nodes = self.node.get_node_names()

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            return {"collector": copy.deepcopy(self.status), "collector_age_ms": (now - self.received) * 1000 if self.received else None, "quest_age_ms": (now - self.quest_received) * 1000 if self.quest_received else None, "nodes": list(self.nodes), "bridge_error": self.error}

    def hold(self):
        from std_msgs.msg import Bool
        self.stop_pub.publish(Bool(data=False))

    def close(self):
        self.executor.shutdown(timeout_sec=2)
        self.thread.join(timeout=2)
        self.node.destroy_node()
        self.context.shutdown()


class DemoBridge:
    """Explicit offline preview; never imports ROS, connects hardware or writes episodes."""
    def __init__(self, cfg):
        from data_collect.control.kinematics import ForwardKinematics
        self.fk = ForwardKinematics(local_path(cfg["urdf"]), tcp_offset=cfg["tcp_offset"])
        self.lock = threading.RLock()
        self.started, self.paused = None, False
        self.frozen = [0.0] * 6
        self.frozen_width = .04

    def start(self):
        with self.lock:
            self.started, self.paused = time.monotonic(), False

    def hold(self):
        with self.lock:
            robot = self.snapshot()["collector"]["robot"]
            self.frozen, self.frozen_width = robot["joint_pos"], robot["gripper_pos"][0]
            self.paused = True

    def stop(self):
        with self.lock:
            robot = self.snapshot()["collector"]["robot"]
            self.frozen, self.frozen_width = robot["joint_pos"], robot["gripper_pos"][0]
            self.started = None

    def snapshot(self):
        with self.lock:
            active = self.started is not None
            elapsed = time.monotonic() - self.started if active else 0
            t = max(0, elapsed - 3)
            q = [0.09 * math.sin(t * .35), 0.12 * (1 - math.cos(t * .3)), -0.18 * (1 - math.cos(t * .25)), .1 * math.sin(t * .4), .1 * math.sin(t * .3), .12 * math.sin(t * .2)] if active and not self.paused else self.frozen
            target = [v + .004 * math.sin(t) for v in q] if active and not self.paused and elapsed > 3 else list(q)
            target[1], target[2] = max(0, target[1]), min(0, target[2])
            recording = active and not self.paused and elapsed > 5
            width = .04 + .02 * math.sin(t * .2) if active and not self.paused else self.frozen_width
            status = {"recording": "演示轨迹（不保存）" if recording else None, "recording_duration_s": max(0, elapsed - 5) if recording else 0, "armed": active and not self.paused and 3 < elapsed <= 5, "homing": active and not self.paused and elapsed <= 3, "aligned": active and not self.paused and elapsed > 3, "teleop_enabled": active and not self.paused and elapsed > 3, "arm_enabled": active, "control_ready": active, "error": None, "robot": {"joint_pos": q, "target_joint_pos": target if active else None, "tcp_pose": self.fk(q).tolist(), "gripper_pos": [width], "age_ms": 4 if active else None, "joint_host_ts_ns": time.monotonic_ns()}, "streams": {name: {"age_ms": 10 + i * 3 if active else None, "fps": (30 if i < 2 else 60) if active else 0} for i, name in enumerate(IMAGE_STREAMS)}, "writer": {"pending_mb": 8.4 if recording else 0, "capacity_mb": 256}, "fk_position_error_m": .00026, "fk_rotation_error_rad": .00003}
            return {"collector": status, "collector_age_ms": 0 if active else None, "quest_age_ms": 8 if active else None, "nodes": list(NODES) if active else [], "bridge_error": None, "demo_paused": self.paused}

    def close(self):
        pass


class LaunchSupervisor:
    def __init__(self, bridge, config_path, demo=False):
        self.bridge, self.config_path, self.demo = bridge, config_path, demo
        self.lock = threading.RLock()
        self.process, self.stopping, self.returncode, self.log_path = None, False, None, None
        self.logs, self.sequence, self.started_at, self.demo_running = deque(maxlen=600), 0, None, False
        self.stop_thread = None
        self.failure = None

    def log(self, message):
        with self.lock:
            self.sequence += 1
            self.logs.append({"id": self.sequence, "time": datetime.now().strftime("%H:%M:%S"), "text": message.rstrip()[:4000]})

    @staticmethod
    def external_process():
        # ROS discovery may lag behind a launch still connecting to CAN/USB.
        for proc in Path("/proc").iterdir():
            if not proc.name.isdigit() or int(proc.name) == os.getpid():
                continue
            try:
                if proc.stat().st_uid != os.getuid():
                    continue
                args = (proc / "cmdline").read_bytes().split(b"\0")
                if b"data_collect.application.collector" in args or (b"./launch/collect.launch.py" in args and (proc / "cwd").resolve() == ROOT) or str(ROOT / "launch/collect.launch.py").encode() in args:
                    return int(proc.name)
            except (OSError, PermissionError):
                continue
        return None

    def start(self, can_port):
        if not re.fullmatch(r"(?:v?can)[0-9]{1,3}", can_port):
            raise ValueError("CAN 接口应为 can0、can1 或 vcan0 等名称")
        with self.lock:
            if self.stopping or (self.process and self.process.poll() is None) or self.demo_running:
                raise ValueError("程序已经启动或正在停止")
            if self.demo:
                self.bridge.start()
                self.demo_running, self.started_at = True, time.time()
                self.returncode = None
                self.log("离线演示已启动；没有连接硬件，不写入采集数据。")
                return
            snap = self.bridge.snapshot()
            if "agx_arm_ctrl_single_node" in snap["nodes"] or (snap["collector_age_ms"] is not None and snap["collector_age_ms"] < 2000) or self.external_process():
                raise ValueError("检测到已有机械臂/采集程序，请先停止原程序；GUI 可直接观察它的状态")
            directory = ROOT / "data/logs/gui"
            directory.mkdir(parents=True, exist_ok=True)
            self.log_path = directory / (datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ".log")
            command = [sys.executable, str(Path(sys.executable).parent / "ros2"), "launch", "./launch/collect.launch.py", f"config:={self.config_path}", f"can_port:={can_port}"]
            env = {**os.environ, "PYTHONUNBUFFERED": "1", "RCUTILS_COLORIZED_OUTPUT": "0"}
            self.process = subprocess.Popen(command, cwd=ROOT, env=env, start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors="replace", bufsize=1)
            self.returncode, self.started_at = None, time.time()
            self.failure = None
            self.log(f"启动遥操程序，PID {self.process.pid}，接口 {can_port}；等待手柄 A 回零对齐。")
            threading.Thread(target=self._read, args=(self.process, self.log_path), name="dashboard-launch-log", daemon=True).start()

    def _read(self, process, log_path):
        try:
            with log_path.open("w", encoding="utf-8") as output:
                for line in process.stdout:
                    output.write(line)
                    output.flush()
                    clean = re.sub(r"\x1b\[[0-9;]*m", "", line)
                    self.log(clean)
                    if "process has died" in clean or (not self.stopping and "process has finished cleanly" in clean):
                        with self.lock:
                            self.failure = "必要子进程退出：" + clean.strip()
                        if not self.stopping:
                            self.log("子进程异常退出，正在停止本次启动的其余进程。")
                            self.stop()
            code = process.wait()
            cleanup = False
            with self.lock:
                if self.process is process:
                    self.returncode = code
                    if code != 0 and not self.stopping:
                        self.failure = self.failure or f"启动进程非正常退出，返回码 {code}"
                    if not self.stopping:
                        try:
                            os.killpg(process.pid, 0)
                            cleanup = True
                        except ProcessLookupError:
                            self.process = None
            self.log(f"ROS 启动器已退出，返回码 {code}。" + ("存在子进程异常，详见日志。" if self.failure else ""))
            if cleanup:
                self.stop()
        except Exception as exc:
            with self.lock:
                self.failure = f"进程日志读取失败：{exc}"
            self.log(f"进程日志读取失败：{exc}")

    def stop(self):
        with self.lock:
            if self.demo:
                self.bridge.stop()
                self.demo_running = False
                self.returncode = 0
                self.log("离线演示已停止。")
                return
            if not self.process or self.stopping:
                return
            self.stopping = True
            self.stop_thread = threading.Thread(target=self._stop_group, args=(self.process,), name="dashboard-launch-stop", daemon=True)
            self.stop_thread.start()

    def _stop_group(self, process):
        try:
            self.bridge.hold()
            for sig, timeout in ((signal.SIGINT, 20), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
                try:
                    # ros2 launch forwards SIGINT to its children. Signalling
                    # the whole group here interrupts their cleanup twice.
                    if sig == signal.SIGINT and process.poll() is None:
                        process.send_signal(sig)
                    else:
                        os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                target = "启动进程（由 ROS 转发给子进程）" if sig == signal.SIGINT else "进程组"
                self.log(f"向本界面启动的{target}发送 {sig.name}，等待退出及数据封盘。")
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    process.poll()  # Reap the group leader before checking the group.
                    try:
                        os.killpg(process.pid, 0)
                    except ProcessLookupError:
                        return
                    time.sleep(.1)
                self.log("进程仍未退出，准备升级终止信号；异常残留数据保留为 partial。")
                with self.lock:
                    self.failure = self.failure or "程序退出超时，已升级终止信号，请核查 partial 数据。"
        finally:
            with self.lock:
                self.stopping = False
                self.returncode = process.poll()
                if self.process is process:
                    self.process = None

    def snapshot(self, after_log=0):
        with self.lock:
            running = self.demo_running if self.demo else bool(self.process and self.process.poll() is None)
            return {"running": running, "stopping": self.stopping, "pid": self.process.pid if running and self.process else None, "returncode": self.returncode, "error": self.failure, "started_at": self.started_at, "log_path": str(self.log_path) if self.log_path else None, "logs": [row for row in self.logs if row["id"] > after_log], "log_cursor": self.sequence}

    def close(self):
        if self.demo_running or self.process:
            self.stop()
        if self.stop_thread:
            self.stop_thread.join(timeout=29)


def make_server(cfg, config_path, bridge, supervisor, port, demo=False):
    model, assets = robot_description(cfg)
    token = secrets.token_urlsafe(32)
    boot = {"application": "piper-data-collect-dashboard", "demo": demo, "model": model, "safety": cfg["motion"], "nodes": NODES, "streams": list(IMAGE_STREAMS), "config": str(config_path)}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, body, kind="application/json; charset=utf-8", status=200):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def valid_host(self):
            return self.headers.get("Host", "") in {f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}"}

        def do_GET(self):
            if not self.valid_host():
                self.send({"error": "仅允许本机访问"}, status=403)
                return
            url = urlparse(self.path)
            try:
                if url.path == "/":
                    self.send((STATIC / "index.html").read_text().replace("__TOKEN__", token).encode(), "text/html; charset=utf-8")
                elif url.path == "/api/boot":
                    self.send(boot)
                elif url.path == "/api/state":
                    after = max(0, int(parse_qs(url.query).get("after_log", ["0"])[0]))
                    self.send({**bridge.snapshot(), "process": supervisor.snapshot(after), "server_time": time.time(), "demo": demo})
                elif url.path.startswith("/model/") and url.path[7:] in assets:
                    self.send(assets[url.path[7:]].read_bytes(), "application/octet-stream")
                elif url.path.startswith("/static/"):
                    path = (STATIC / url.path[8:]).resolve()
                    if not path.is_relative_to(STATIC) or not path.is_file() or path.name == "index.html":
                        self.send({"error": "资源不存在"}, status=404)
                    else:
                        kind = "text/javascript" if path.suffix == ".js" else mimetypes.guess_type(path.name)[0] or "application/octet-stream"
                        self.send(path.read_bytes(), kind)
                else:
                    self.send({"error": "资源不存在"}, status=404)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except (OSError, ValueError, TypeError) as exc:
                self.send({"error": str(exc)}, status=400)

        def do_POST(self):
            origin = self.headers.get("Origin")
            if not self.valid_host() or (origin and origin not in {f"http://127.0.0.1:{self.server.server_port}", f"http://localhost:{self.server.server_port}"}) or not secrets.compare_digest(self.headers.get("X-Control-Token", ""), token):
                self.send({"error": "本机控制凭证无效，请刷新界面"}, status=403)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 <= length <= 4096:
                    raise ValueError("请求过大")
                body = json.loads(self.rfile.read(length) or b"{}")
                if not isinstance(body, dict):
                    raise ValueError("请求格式错误")
                if self.path == "/api/start":
                    supervisor.start(str(body.get("can_port", "can0")))
                elif self.path == "/api/stop":
                    supervisor.stop()
                elif self.path == "/api/hold":
                    bridge.hold()
                    supervisor.log("已请求停止运动/取消回零（与 B 相同）；程序仍保持运行。")
                else:
                    self.send({"error": "接口不存在"}, status=404)
                    return
                self.send({"ok": True})
            except (ValueError, OSError, RuntimeError) as exc:
                self.send({"error": str(exc)}, status=409)

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.daemon_threads = True
    return server


def main():
    parser = argparse.ArgumentParser(description="Piper 遥操与采集可视化工作台")
    parser.add_argument("--config", default="config/collection.yaml")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--demo", action="store_true", help="离线演示，不连接硬件、不保存数据")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    if not 0 <= args.port <= 65535:
        parser.error("端口须在 0–65535 范围内")
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    signal.signal(signal.SIGHUP, terminate)
    if sys.version_info[:2] not in ((3, 10), (3, 11)):
        parser.error(f"请先执行 source {ROOT}/env.sh，使用独立采集环境 Python 3.11")
    cfg, config_path = load_config(args.config), local_path(args.config)
    try:
        if reuse_dashboard(args.port, config_path, args.demo, args.no_browser):
            return
    except ValueError as exc:
        parser.exit(1, f"界面启动失败：{exc}\n")
    bridge = DemoBridge(cfg) if args.demo else RosBridge()
    supervisor = LaunchSupervisor(bridge, config_path, args.demo)
    server = None
    try:
        server = make_server(cfg, config_path, bridge, supervisor, args.port, args.demo)
        url = f"http://127.0.0.1:{server.server_port}/"
        print(f"Piper GUI：{url} {'（离线演示）' if args.demo else '（尚未启动机械臂）'}", flush=True)
        if not args.no_browser:
            webbrowser.open(url)
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE:
            try:
                if reuse_dashboard(args.port, config_path, args.demo, args.no_browser):
                    return
            except ValueError as conflict:
                parser.exit(1, f"界面启动失败：{conflict}\n")
            parser.exit(1, f"端口 {args.port} 已被其他程序占用，请改用其他端口，例如：python3 -m data_collect.cli.gui --port 8768\n")
        parser.exit(1, f"界面启动失败：{exc}\n")
    finally:
        supervisor.close()
        if server:
            server.server_close()
        bridge.close()


if __name__ == "__main__":
    main()
