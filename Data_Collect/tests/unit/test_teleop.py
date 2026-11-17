"""Exercise the production A/B path and command outlet without ROS or devices."""

import importlib.util
import ast
import sys
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from data_collect.application import collector
from data_collect.common.paths import ROOT
from data_collect.control.robot import JOINT_NAMES, RobotObserver, TeleopTarget
from data_collect.common.config import local_path
from data_collect.control.kinematics import ForwardKinematics


class JointState:
    def __init__(self):
        self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id="")
        self.name, self.position = [], []


class PoseStamped:
    def __init__(self):
        self.header = NS(stamp=None)
        self.pose = NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))


@pytest.fixture
def ros_scripts(monkeypatch):
    """Import ROS entry points with only their message/Node boundary replaced."""
    modules = {
        "rclpy": NS(), "rclpy.node": NS(Node=object),
        "rclpy.executors": NS(ExternalShutdownException=RuntimeError),
        "sensor_msgs.msg": NS(JointState=JointState),
        "std_msgs.msg": NS(Bool=lambda **kw: NS(**kw), Header=lambda: NS(stamp=None), String=NS),
        "geometry_msgs.msg": NS(Pose=NS, PoseStamped=PoseStamped, TransformStamped=NS),
        "tf2_ros": NS(TransformBroadcaster=object),
        "rcl_interfaces.msg": NS(ParameterDescriptor=NS),
        "std_srvs.srv": NS(Empty=NS(Request=object)),
        "oculus_reader": NS(OculusReader=object),
        "data_collect.common.executor": NS(TeleopExecutor=object),
        "ament_index_python.packages": NS(get_package_share_directory=lambda name: ""),
        "casadi": NS(), "pinocchio": NS(casadi=NS()),
    }
    for name, value in modules.items():
        monkeypatch.setitem(sys.modules, name, value)
    directory = ROOT / "ros_ws/src/oculus_reader/scripts"
    monkeypatch.syspath_prepend(str(directory))

    def load(name):
        spec = importlib.util.spec_from_file_location("test_" + name, directory / (name + ".py"))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    return load


def controller_publisher(module, rpy):
    """Initialize the production frame matrices without opening ADB or ROS."""
    node = module.OculusPublisher.__new__(module.OculusPublisher)
    source = ROOT / "ros_ws/src/oculus_reader/scripts/pub_pose.py"
    constructor = next(
        method for item in ast.parse(source.read_text()).body
        if isinstance(item, ast.ClassDef) and item.name == "OculusPublisher"
        for method in item.body if isinstance(method, ast.FunctionDef) and method.name == "__init__"
    )
    assignments = [item for item in constructor.body if isinstance(item, ast.Assign)
                   and isinstance(item.targets[0], ast.Attribute)
                   and item.targets[0].attr in ("adj_mat", "r_adj", "ros_to_arm_mat")]
    exec(compile(ast.Module(body=assignments, type_ignores=[]), str(source), "exec"),
         {"self": node, "np": np, "xyzrpy2Mat": module.xyzrpy2Mat,
          "ros_to_arm_xyz": [0., 0., 0.], "ros_to_arm_rpy": rpy})
    return node


@pytest.mark.parametrize("rpy", [[-1.5708, 0., -1.5708], [0., 1.0471, 0.]])
def test_base_mapping_preserves_reference_quest_axes_for_arbitrary_a_alignment(ros_scripts, rpy):
    pose, delta = ros_scripts("pub_pose"), ros_scripts("pub_delta_pose")
    publisher = controller_publisher(pose, rpy)
    # Reference Quest axes: -Z forward, -X left, +Y up -> ROS +X,+Y,+Z.
    quest_to_ros = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
    scale = np.array([1.2, 1.2, 1.2])
    for hand_rpy, tcp_rpy in (([.3, -.4, .9], [-1.98, .012, -1.54]),
                              ([-.6, .8, -1.2], [.2, -.5, 1.])):
        raw_start = delta.xyzrpy_to_mat(.1, -.2, .3, *hand_rpy)
        start = publisher._correct_to_ros(raw_start)
        tcp = delta.xyzrpy_to_mat(.4, -.1, .25, *tcp_rpy)
        for axis in np.eye(3):
            for sign in (-1, 1):
                for kind in ("translation", "rotation"):
                    raw_end = raw_start.copy()
                    expected_position = tcp[:3, 3].copy()
                    expected_rotation = tcp[:3, :3].copy()
                    if kind == "translation":
                        displacement = sign * .01 * axis
                        raw_end[:3, 3] += displacement
                        expected_position += scale * (quest_to_ros @ displacement)
                    else:
                        rotation = Rotation.from_rotvec(sign * .05 * axis).as_matrix()
                        raw_end[:3, :3] = rotation @ raw_start[:3, :3]
                        expected_rotation = quest_to_ros @ rotation @ quest_to_ros.T @ expected_rotation
                    end = publisher._correct_to_ros(raw_end)
                    current = np.r_[end[:3, 3], Rotation.from_matrix(end[:3, :3]).as_euler("xyz")]
                    position, quaternion = delta.calc_pose_incre(start, current, tcp, scale, "base")
                    np.testing.assert_allclose(position, expected_position, atol=1e-12)
                    np.testing.assert_allclose(Rotation.from_quat(quaternion).as_matrix(), expected_rotation, atol=1e-12)


def test_tcp_mode_reproduces_questarmteleop_increment(ros_scripts):
    delta = ros_scripts("pub_delta_pose")
    source = ROOT.parent / "Piper_DM (2)/QuestArmTeleop/src/oculus_reader/scripts/pub_delta_pose.py"
    if not source.exists():
        pytest.skip("独立部署目录未包含 QuestArmTeleop 参考工程")
    definitions = [item for item in ast.parse(source.read_text()).body
                   if isinstance(item, ast.FunctionDef)
                   and item.name in ("xyzrpy_to_mat", "mat2xyzquat", "calc_pose_incre")]
    namespace = {"np": np, "Rotation": Rotation}
    exec(compile(ast.Module(body=definitions, type_ignores=[]), str(source), "exec"), namespace)
    start = delta.xyzrpy_to_mat(.1, -.2, .3, .3, -.4, .9)
    tcp = delta.xyzrpy_to_mat(.4, -.1, .25, -1.98, .012, -1.54)
    for axis in np.eye(6):
        for sign in (-1, 1):
            current = np.array([.1, -.2, .3, .3, -.4, .9]) + sign * .01 * axis
            expected = namespace["calc_pose_incre"](start, current, tcp, np.ones(3))
            actual = delta.calc_pose_incre(start, current, tcp, np.ones(3), "tcp")
            for left, right in zip(actual, expected):
                np.testing.assert_allclose(left, right, atol=1e-12)


@pytest.fixture
def operator(ros_scripts):
    module = ros_scripts("pub_delta_pose")
    node = module.RosOperator.__new__(module.RosOperator)
    node.x, node.y, node.z = .1, .2, .3
    node.roll = node.pitch = node.yaw = 0.
    node.tcp_x, node.tcp_y, node.tcp_z = .4, -.1, .25
    node.tcp_roll = node.tcp_pitch = node.tcp_yaw = 0.
    node.start_pose_matrix = node.zero_matrix = np.eye(4)
    node.flag, node.home_future, node.home_timer = False, None, None
    node.following = False
    node.follow_requires_release = False
    node.start_button, node.stop_button, node.reset_button = "A", "B", "RJ"
    node.follow_button = "RG"
    node.start_edge, node.stop_edge, node.reset_edge = (module.RisingEdge() for _ in range(3))
    node.trigger_axis, node.gripper_max_range = "rightTrig", .07
    node.gripper_joint_name, node.translation_scale = "gripper", np.ones(3)
    node.delta_frame = "tcp"
    node.auto_home_on_stop = False
    node.hand_name = "right"
    buttons = {}
    node.oculus_reader = NS(get_transformations_and_buttons=lambda: ({}, buttons))
    poses, enables, follows, home_calls, timers = [], [], [], [], []
    node.pub_delta_pose = NS(publish=poses.append)
    node.pub_move_j = NS(publish=lambda msg: None)
    node.pub_teleop_enabled = NS(publish=lambda msg: enables.append(msg.data))
    node.pub_following = NS(publish=lambda msg: follows.append(msg.data))
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: NS(sec=1, nanosec=0)))
    node.get_logger = lambda: NS(info=lambda msg: None, warning=lambda msg: None, error=lambda msg: None)
    node.auto_home_delay_s = .5
    node.create_timer = lambda delay, callback: (timers.append(NS(delay=delay, callback=callback)) or timers[-1])
    node.destroy_timer = lambda timer: timers.remove(timer)
    node.move_home_client = NS(service_is_ready=lambda: True,
                               call_async=lambda request: (home_calls.append(request) or
                                                          NS(done=lambda: False, add_done_callback=lambda cb: None)))
    return NS(node=node, buttons=buttons, poses=poses, enables=enables, follows=follows,
              home_calls=home_calls, timers=timers)


def test_a_rebases_at_current_tcp_without_home_and_rebases_again_after_b(operator):
    node, buttons = operator.node, operator.buttons
    poses, enables = operator.poses, operator.enables
    buttons.update(A=True)

    node.control_loop()
    assert enables == [True]
    assert not poses
    buttons["RG"] = True
    node.control_loop()
    assert poses[-1].header.frame_id == "base_link"
    np.testing.assert_allclose([poses[-1].pose.position.x, poses[-1].pose.position.y,
                               poses[-1].pose.position.z], [.4, -.1, .25])
    node.x += .01
    node.control_loop()  # A is still held: keep the original reference and follow.
    assert enables == [True]
    assert node.start_pose_matrix[0, 3] == pytest.approx(.1)
    assert poses[-1].pose.position.x == pytest.approx(.41)
    buttons.clear()
    buttons["B"] = True
    node.control_loop()
    assert enables[-1] is False
    node.x, node.tcp_x = .8, .6
    buttons.clear()
    buttons["A"] = True
    node.control_loop()
    assert enables[-1] is True
    assert not node.following
    buttons["RG"] = True
    node.control_loop()
    assert poses[-1].pose.position.x == pytest.approx(.6)


def test_grip_pauses_and_resumes_at_actual_tcp_discarding_paused_hand_motion(operator):
    node, buttons, poses = operator.node, operator.buttons, operator.poses
    buttons["RG"] = True
    node.control_loop()  # Grip alone cannot arm teleoperation.
    assert not poses
    buttons.clear()
    buttons["A"] = True
    node.control_loop()
    node.x += .2
    node.control_loop()
    assert node.flag and not node.following and not poses

    buttons.clear()
    buttons["RG"] = True
    node.control_loop()
    assert poses[-1].pose.position.x == pytest.approx(.4)
    node.x += .05
    node.control_loop()
    assert poses[-1].pose.position.x == pytest.approx(.45)
    buttons.clear()
    node.control_loop()
    count = len(poses)
    node.x, node.yaw = .9, .8
    node.tcp_x, node.tcp_yaw = .43, -.2  # The previous .45 target was not reached.
    node.control_loop()
    assert len(poses) == count
    assert operator.enables == [True]  # Pause does not split/stop the Episode.
    assert operator.follows == [True, False]

    buttons["RG"] = True
    node.control_loop()
    assert poses[-1].pose.position.x == pytest.approx(.43)
    np.testing.assert_allclose(
        [poses[-1].pose.orientation.x, poses[-1].pose.orientation.y,
         poses[-1].pose.orientation.z, poses[-1].pose.orientation.w],
        Rotation.from_euler("z", -.2).as_quat(), atol=1e-12,
    )
    assert operator.follows == [True, False, True]
    node.x += .01
    node.control_loop()
    assert poses[-1].pose.position.x == pytest.approx(.43 + .01 * np.cos(1.))


@pytest.mark.parametrize("active,tracking", [(True, True), (False, False)])
def test_b_requests_home_even_when_idle_and_own_disable_echo_does_not_cancel(operator, active, tracking):
    node, buttons = operator.node, operator.buttons
    node.auto_home_on_stop = True
    node.flag = node.following = active
    if not tracking:
        node.x = None
    buttons.update(B=True, A=True, RG=True)  # B takes priority over A and grip.
    node.control_loop()
    assert not node.flag and not node.following and not operator.poses
    timer = node.home_timer
    assert timer.delay == .5
    node.external_enabled_callback(NS(data=False))
    assert node.home_timer is timer
    node.control_loop()  # Held B should not schedule a second request.
    assert operator.timers == [timer]
    timer.callback()
    assert node.home_timer is None and len(operator.home_calls) == 1
    assert node._home_in_progress()
    buttons.clear()
    node.control_loop()
    buttons["A"] = True
    node.control_loop()
    assert not node.flag  # Do not resume while the home service is in progress.


def test_a_cancels_unsent_home_and_external_stop_ends_following(operator):
    node, buttons = operator.node, operator.buttons
    node.auto_home_on_stop = True
    buttons["B"] = True
    node.control_loop()
    buttons.clear()
    node.control_loop()
    buttons.update(A=True)
    node.control_loop()
    assert node.home_timer is None and not operator.home_calls and not node.following
    buttons["RG"] = True
    node.control_loop()
    assert node.following
    node.external_enabled_callback(NS(data=False))
    assert not node.flag and not node.following


def test_rj_restarts_clutch_reference_when_grip_is_still_held(operator):
    node, buttons = operator.node, operator.buttons
    buttons.update(A=True)
    node.control_loop()
    buttons["RG"] = True
    node.control_loop()
    buttons.clear()
    buttons.update(RJ=True, RG=True)
    node.x += .1
    node.control_loop()
    assert operator.follows == [True, False, True]
    assert operator.poses[-1].pose.position.x == pytest.approx(node.tcp_x)


def test_a_with_grip_already_held_waits_for_release_before_following(operator):
    node, buttons = operator.node, operator.buttons
    buttons.update(A=True, RG=True)
    node.control_loop()
    assert node.flag and not node.following and not operator.poses
    node.control_loop()
    assert not node.following and not operator.poses
    buttons.clear()
    node.control_loop()
    buttons["RG"] = True
    node.control_loop()
    assert node.following and len(operator.poses) == 1


def test_targets_aggregate_like_reference_without_extra_limits():
    targets = TeleopTarget()
    targets.set_enabled(True)
    targets.update(JOINT_NAMES, [1., 1., -1., .5, .5, 1.])
    assert targets.snapshot() is None
    targets.update(["gripper"], [.07])
    assert targets.snapshot() == [1., 1., -1., .5, .5, 1., .07]
    targets.update(["gripper"], [.02])
    assert targets.snapshot()[-1] == .02
    targets.set_enabled(False)
    assert targets.snapshot() is None


@pytest.mark.parametrize("teleop_only", [False, True])
@pytest.mark.parametrize("recording_ok", [False, True, "recovers"])
def test_recording_start_pause_resume_does_not_gate_commands(config, monkeypatch, ros_scripts, teleop_only, recording_ok):
    moves, events = [], []
    can_record = NS(ready=recording_ok is True)

    class Driver:
        def __init__(self):
            self.agx_arm, self.gripper = NS(), None

        def create_subscription(self, *args):
            pass

        def create_publisher(self, *args):
            return NS(publish=lambda msg: None)

        def create_timer(self, *args):
            pass

        def get_logger(self):
            return NS(info=lambda msg: None, error=lambda msg: None)

        def get_clock(self):
            return NS(now=lambda: NS(nanoseconds=1_000_000_000))

        def _joint_states_callback(self, message):
            moves.append(list(message.position))
            observer.target_joint = np.asarray(message.position[:6], dtype=float)

    def recording_unavailable():
        events.append("recording_attempt")
        if can_record.ready:
            return "same_episode.h5"
        raise RuntimeError("camera or calibration unavailable")

    manager = NS(lock=threading.RLock(), latest={"robot_feedback": {"joint_pos": np.zeros(6)}},
                 failure="sensor failed" if recording_ok is False else None,
                 start=recording_unavailable, stop=lambda: None, pause=lambda: events.append("pause"), current=None)
    observer = NS(lock=threading.RLock(), timings=NS(add=lambda *args: None))
    def runtime(*args, capture_enabled):
        assert capture_enabled is not teleop_only
        return NS(start=lambda: None)

    monkeypatch.setattr(collector, "SensorRuntime", runtime)
    node_class = collector.build_driver_class(Driver, config, manager, observer, teleop_only=teleop_only)
    monkeypatch.setattr(node_class, "publish_status", lambda self: None)
    node = node_class()
    node.on_enabled(NS(data=True))
    assert not moves and node.targets.enabled
    node.on_target(NS(name=["gripper"], position=[.07],
                      header=NS(stamp=NS(sec=1, nanosec=0), frame_id="")))
    node.control_tick()
    assert not events and not moves  # A alone must not enter SDK fast mode or move the gripper.
    node.on_following(NS(data=True))
    node.control_tick()
    assert not moves and not events  # Wait for the new IK reference's first full solution.
    def target(names, positions, sec=1):
        return NS(name=list(names), position=positions,
                  header=NS(stamp=NS(sec=sec, nanosec=0), frame_id=node.reference_key))
    node.on_target(target(JOINT_NAMES, [1., 1., -1., .5, .5, 1.]))
    node.on_target(target(["gripper"], [.07]))
    node.control_tick()
    assert moves[-1] == [1., 1., -1., .5, .5, 1., .07]
    if teleop_only:
        assert node.recording_error is None and not node.pending_episode
    elif can_record.ready:
        assert node.recording_error is None
    else:
        assert "camera" in node.recording_error
    node.on_target(target(JOINT_NAMES, [-1., 2., -2., -.5, -.5, -1.]))
    node.control_tick()
    assert moves[-1] == [-1., 2., -2., -.5, -.5, -1., .07]
    if not teleop_only:
        assert events.count("recording_attempt") == 1  # No mid-grip automatic retry.
    if recording_ok == "recovers":
        can_record.ready = True  # Both tactile devices/baselines finish startup.
    actual = [.1, .2, -.3, .4, .5, .6]
    manager.latest["robot_feedback"] = {"joint_pos": np.array(actual)}
    node.on_target(target(JOINT_NAMES, [3.] * 6))  # Buffered, never sent to SDK.
    node.on_following(NS(data=False))
    node.on_target(target(JOINT_NAMES, [1.] * 6))  # Late IK result while paused.
    node.on_target(target(["gripper"], [.02]))  # Independent index trigger still works.
    node.control_tick()
    frozen = [-1., 2., -2., -.5, -.5, -1., .02]
    assert moves[-1] == frozen  # Preserve the sent target, not sagging feedback or buffered IK.
    assert node.targets.enabled and not observer.following
    node.on_following(NS(data=True))
    node.on_target(target(JOINT_NAMES, [2.] * 6, sec=0))  # Previous clutch interval.
    node.control_tick()
    assert moves[-1] == frozen  # Keep sending hold while waiting for a new solution.
    if recording_ok == "recovers":
        assert node.recording_error is None
    old_epoch = target(JOINT_NAMES, [2.] * 6)
    old_epoch.header.frame_id = "previous_clutch"
    node.on_target(old_epoch)
    node.control_tick()
    assert moves[-1] == frozen
    node.on_target(target(JOINT_NAMES, [-.1] * 6))
    node.control_tick()
    assert moves[-1] == [-.1] * 6 + [.02]
    node.on_enabled(NS(data=False))
    count = len(moves)
    node.control_tick()
    assert len(moves) == count
    assert events == ([] if teleop_only else
                      ["recording_attempt", "pause", "recording_attempt"] if recording_ok is not False else
                      ["recording_attempt", "pause"])


def test_ik_repeated_clutch_does_not_follow_biased_tcp_feedback(ros_scripts, config):
    module = ros_scripts("arm_ik_pose_node")
    node = module.ArmIKPoseNode.__new__(module.ArmIKPoseNode)
    fk = ForwardKinematics(local_path(config["urdf"]), tcp_offset=config["tcp_offset"])
    q_hold = np.array([.2, .8, -1., .2, -.2, .1])
    reference_fk = fk.matrix(q_hold)
    published, requested, rebases = [], [], []
    node.ik = NS(active_joint_names=lambda: list(JOINT_NAMES),
                 rebase=lambda q: rebases.append(q.copy()), end_effector_pose=fk.matrix,
                 solve=lambda target: (requested.append(target.copy()) or q_hold.copy()),
                 accept=lambda q: None, last_solve={})
    node.use_clutch_reference = True
    node.reference_joint = None
    node.reference_stamp_ns = 0
    node.output_joint_names = list(JOINT_NAMES)
    node.simulation_feedback = node.enable_collision_check = False
    node.pub_joint = NS(publish=published.append)
    node.get_logger = lambda: NS(info=lambda msg: None, warning=lambda msg: None)
    # A constant load/tracking offset must not be subtracted from the command
    # again each time the clutch is operated. This is injected, not measured.
    for segment in range(10):
        reference = JointState()
        reference.name, reference.position = list(JOINT_NAMES), q_hold.tolist()
        reference.header.stamp.sec = segment + 1
        reference.header.frame_id = str(segment + 1)
        node.reference_joint_callback(reference)
        msg = PoseStamped()
        msg.header.stamp = NS(sec=segment + 1, nanosec=10)
        input_pose = reference_fk.copy()
        input_pose[2, 3] -= .03
        quaternion = Rotation.from_matrix(input_pose[:3, :3]).as_quat()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = input_pose[:3, 3]
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = quaternion
        node.pose_stamped_callback(msg)
        node.pose_stamped_callback(msg)
        np.testing.assert_array_equal(published[-1].position, q_hold)
        assert published[-1].header.frame_id == reference.header.frame_id
    assert not requested  # Zero hand increment holds exact q without another optimization.
    assert len(rebases) == 10
    # A real input increment keeps its original base-axis direction and size.
    msg.pose.position.z += .01
    rotation = Rotation.from_rotvec([.01, -.02, .03]).as_matrix()
    quaternion = Rotation.from_matrix(rotation @ input_pose[:3, :3]).as_quat()
    (msg.pose.orientation.x, msg.pose.orientation.y,
     msg.pose.orientation.z, msg.pose.orientation.w) = quaternion
    node.pose_stamped_callback(msg)
    np.testing.assert_allclose(requested[-1][:3, 3], reference_fk[:3, 3] + [0., 0., .01])
    np.testing.assert_allclose(requested[-1][:3, :3], rotation @ reference_fk[:3, :3], atol=1e-12)
    # A late Pose from before this reference cannot become the new input origin.
    reference.header.stamp.sec += 1
    node.reference_joint_callback(reference)
    before = len(published)
    node.pose_stamped_callback(msg)
    assert node.reference_input_pose is None and len(published) == before


def test_ik_rebase_resets_old_segment_smoothing_reference(ros_scripts):
    module = ros_scripts("arm_ik_pose_node")
    ik = module.ArmIK.__new__(module.ArmIK)
    ik.reduced_robot = NS(model=NS(nq=6))
    ik._state_synced = True
    ik.history_data = np.ones(6)
    q = np.arange(6) * .1
    ik.rebase(q)
    for value in (ik.init_data, ik.feedback_data, ik.history_data):
        np.testing.assert_array_equal(value, q)


def test_command_records_following_state_alongside_actual_sent_targets():
    records = []
    manager = NS(emit=lambda group, record: records.append((group, record)))
    observer = RobotObserver(manager)
    observer.command("joint", np.arange(6) * .1, 1, 2)
    observer.command("gripper", .03, 3, 4)
    assert records[-1][0] == "robot_command"
    assert not records[-1][1]["following"]
    observer.following = True
    observer.command("joint", np.zeros(6), 5, 6)
    assert records[-1][1]["following"]
    np.testing.assert_array_equal(records[-1][1]["target_joint_pos"], np.zeros(6))


@pytest.mark.parametrize("paused", [False, True])
def test_console_status_shows_active_duration_and_missing_device(ros_scripts, config, monkeypatch, tmp_path, paused):
    from data_collect.recording.timeline import RecordingTimeline
    from collections import deque

    messages = []
    timeline = RecordingTimeline(1_000_000_000)
    if paused:
        timeline.set_recording(False, 1_500_000_000)
    stamps = deque([1_970_000_000, 1_990_000_000])
    feedback = {name + "_available_host_ts_ns": 1_990_000_000 for name in ("joint", "tcp", "gripper")}
    def not_ready():
        raise RuntimeError("右触觉尚未就绪")
    manager = NS(lock=threading.RLock(), latest={"robot_feedback": feedback},
                 current=NS(path=tmp_path / "episode_0003.h5", timeline=timeline), finished=[],
                 require_ready=not_ready, stream_times={"cameras/wrist_rgb": stamps, "tactile/right_raw": deque()})
    saved = tmp_path / "episode_0002.h5"
    saved.touch()
    manager.finished = [NS(path=saved, thread=NS(is_alive=lambda: False), error=None),
                        NS(path=tmp_path / "pending.h5", thread=NS(is_alive=lambda: True), error=None)]
    observer = NS(timings=NS(snapshot=lambda: {}))
    node_class = collector.build_driver_class(object, config, manager, observer)
    node = node_class.__new__(node_class)
    node.runtime, node.executor, node.reported_files = NS(capture=None), NS(), set()
    node.get_logger = lambda: NS(info=messages.append, error=messages.append)
    monkeypatch.setattr(collector.time, "monotonic_ns", lambda: 2_000_000_000)
    node.log_diagnostics()
    line = next(s for s in messages if s.startswith("[状态]"))
    assert ("暂停" if paused else "录制") in line
    assert ("有效 0.5 秒" if paused else "有效 1.0 秒") in line
    assert "手腕 50fps" in line and "右触觉 未就绪" in line and "反馈 10ms" in line
    node.log_diagnostics()
    assert sum("[采集] 已保存" in s for s in messages) == 1
    assert not any("封盘失败" in s for s in messages)


@pytest.mark.parametrize("override", [{"speed_percent": 25}, {"fast_mode": False},
                                       {"tcp_offset": [0., 0., .2, 0., 0., 0.]}])
def test_tracking_rejects_ros_overrides_before_sdk_initialization(ros_scripts, tracking_config, override):
    attempts = []

    class Driver:
        def __init__(self):
            self._load_parameters()
            attempts.append("SDK connection")

        def _load_parameters(self):
            self.arm_type = tracking_config["arm_type"]
            self.tcp_offset = tracking_config["tcp_offset"]
            self.fast_mode = tracking_config["teleop"]["fast_mode"]
            self.speed_percent = tracking_config["teleop"]["speed_percent"]
            for key, value in override.items():
                setattr(self, key, value)

    node_class = collector.build_driver_class(Driver, tracking_config, NS(), NS())
    with pytest.raises(ValueError, match="当前设置不符"):
        node_class()
    assert attempts == []


def test_teleop_only_runtime_polls_robot_without_constructing_capture(config, monkeypatch):
    def unexpected_capture(*args):
        pytest.fail("纯遥操不应创建传感器采集进程")

    monkeypatch.setitem(sys.modules, "data_collect.sensors.capture", NS(CaptureRuntime=unexpected_capture))
    polled = threading.Event()
    errors = []
    observer = NS(poll=lambda *args: (polled.set() or True),
                  timings=NS(add=lambda *args: None))
    runtime = collector.SensorRuntime(config, NS(fail=errors.append), observer, NS(), NS(),
                                      capture_enabled=False)
    try:
        runtime.start()
        assert polled.wait(timeout=1)
        assert runtime.capture is None
        assert [thread.name for thread in runtime.threads] == ["robot-feedback"]
    finally:
        runtime.close()
    assert not errors and not any(thread.is_alive() for thread in runtime.threads)


def test_ik_publishes_solver_result_without_tcp_step_or_residual_gate(ros_scripts):
    module = ros_scripts("arm_ik_pose_node")
    node = module.ArmIKPoseNode.__new__(module.ArmIKPoseNode)
    published, accepted, requested = [], [], []
    solution = np.array([1., 1., -1., .5, .5, 1.])
    node.ik = NS(solve=lambda target: (requested.append(target.copy()) or solution),
                 accept=accepted.append, last_solve={})
    node.output_joint_names = list(JOINT_NAMES)
    node.pub_joint = NS(publish=published.append)
    node.simulation_feedback = node.enable_collision_check = False
    target = np.eye(4)
    target[:3, 3] = [.4, .2, .3]
    node._solve_and_publish(target, NS(sec=0, nanosec=0))
    np.testing.assert_array_equal(requested[0], target)
    assert published[0].position == solution.tolist()
    np.testing.assert_array_equal(accepted[0], solution)
