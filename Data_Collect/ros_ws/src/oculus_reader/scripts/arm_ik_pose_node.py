#!/usr/bin/env python3
import os
import json
import time
from typing import List

import casadi
import numpy as np
import pinocchio as pin
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from pinocchio import casadi as cpin
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
    

def xyzrpy_to_mat(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
    mat = np.eye(4)
    mat[:3, :3] = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    mat[:3, 3] = np.array([x, y, z])
    return mat


def pose_to_mat(pose: Pose) -> np.ndarray:
    mat = np.eye(4)
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    mat[:3, :3] = Rotation.from_quat(quat).as_matrix()
    mat[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return mat


def limit_tcp_step(previous, requested, translation_limit, rotation_limit):
    """Interpolate toward the current hand target; never queue old targets."""
    if requested.shape != (4, 4) or not np.isfinite(requested).all():
        raise ValueError("TCP 目标必须为有限的 4×4 位姿")
    delta = requested[:3, 3] - previous[:3, 3]
    rotation = Rotation.from_matrix(previous[:3, :3].T @ requested[:3, :3]).as_rotvec()
    distance, angle = np.linalg.norm(delta), np.linalg.norm(rotation)
    fraction = min(1., translation_limit / max(distance, 1e-12), rotation_limit / max(angle, 1e-12))
    result = previous.copy()
    result[:3, 3] += fraction * delta
    result[:3, :3] = previous[:3, :3] @ Rotation.from_rotvec(fraction * rotation).as_matrix()
    return result


class ArmIK:
    def __init__(
        self,
        urdf_path: str,
        package_dirs: List[str],
        locked_joints: List[str],
        ee_parent_joint: str,
        ee_frame_name: str,
        tool_pre_rot_rpy: List[float],
        tool_translation_xyz: List[float],
        collision_pairs_flat: List[int],
        w_pos: float,
        w_ori: float,
        w_reg: float,
        w_smooth: float,
        ipopt_max_iter: int,
        ipopt_tol: float,
        enable_visualization: bool,
        viewer_open_browser: bool,
        viewer_model_name: str,
        viewer_target_frame_name: str,
        viewer_axis_length: float,
        viewer_axis_width: float,
        tcp_offset_rpy=None,
        position_tolerance_m=0.01,
        rotation_tolerance_rad=0.0872664626,
    ):
        if not all(np.isfinite(v) and v > 0 for v in (position_tolerance_m, rotation_tolerance_rad, w_pos, w_ori)):
            raise ValueError("IK 权重和精度门限必须为有限正数")
        if not all(np.isfinite(v) and v >= 0 for v in (w_reg, w_smooth)):
            raise ValueError("IK 正则和平滑权重必须为有限非负数")
        self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path, package_dirs=package_dirs)
        unique_locked_joints = self._deduplicate_locked_joints(locked_joints)
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=unique_locked_joints,
            reference_configuration=np.zeros(self.robot.model.nq),
        )

        first = xyzrpy_to_mat(0.0, 0.0, 0.0, tool_pre_rot_rpy[0], tool_pre_rot_rpy[1], tool_pre_rot_rpy[2])
        tcp_offset_rpy = [0.0, 0.0, 0.0] if tcp_offset_rpy is None else tcp_offset_rpy
        second = xyzrpy_to_mat(tool_translation_xyz[0], tool_translation_xyz[1], tool_translation_xyz[2], *tcp_offset_rpy)
        ee_mat = first @ second
        quat = Rotation.from_matrix(ee_mat[:3, :3]).as_quat()  # x y z w

        self.reduced_robot.model.addFrame(
            pin.Frame(
                ee_frame_name,
                self.reduced_robot.model.getJointId(ee_parent_joint),
                pin.SE3(
                    pin.Quaternion(quat[3], quat[0], quat[1], quat[2]),
                    np.array(ee_mat[:3, 3]),
                ),
                pin.FrameType.OP_FRAME,
            )
        )
        # addFrame changes the model frame count; rebuild cached data before FK.
        self.reduced_robot.data = self.reduced_robot.model.createData()

        self.geom_model = self.reduced_robot.collision_model
        for i in range(0, len(collision_pairs_flat), 2):
            a = collision_pairs_flat[i]
            b = collision_pairs_flat[i + 1]
            self.geom_model.addCollisionPair(pin.CollisionPair(a, b))
        self.geometry_data = pin.GeometryData(self.geom_model)

        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        self.ctf = casadi.SX.sym("tf", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)
        self.ee_id = self.reduced_robot.model.getFrameId(ee_frame_name)

        self.error = casadi.Function(
            "error",
            [self.cq, self.ctf],
            [
                casadi.vertcat(
                    cpin.log6(self.cdata.oMf[self.ee_id].inverse() * cpin.SE3(self.ctf)).vector
                )
            ],
        )
        self.position_error = casadi.Function(
            "position_error", [self.cq, self.ctf],
            [self.cdata.oMf[self.ee_id].translation - self.ctf[:3, 3]],
        )

        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.param_q_prev = self.opti.parameter(self.reduced_robot.model.nq)
        self.param_q_feedback = self.opti.parameter(self.reduced_robot.model.nq)
        self.param_tf = self.opti.parameter(4, 4)

        error_vec = self.error(self.var_q, self.param_tf)
        pos_error = self.position_error(self.var_q, self.param_tf)
        ori_error = error_vec[3:]
        total_cost = casadi.sumsqr(w_pos * pos_error) + casadi.sumsqr(w_ori * ori_error)
        regularization = casadi.sumsqr(self.var_q - self.param_q_feedback)
        smooth_cost = casadi.sumsqr(self.var_q - self.param_q_prev)
        self.opti.minimize(total_cost + w_reg * regularization + w_smooth * smooth_cost)
        # Precision must be feasible inside the optimization, rather than a
        # conflicting check applied only after accepting a soft-cost solution.
        self.opti.subject_to(casadi.sumsqr(pos_error) <= position_tolerance_m ** 2)
        self.opti.subject_to(casadi.sumsqr(ori_error) <= rotation_tolerance_rad ** 2)

        self.opti.subject_to(
            self.opti.bounded(
                self.reduced_robot.model.lowerPositionLimit,
                self.var_q,
                self.reduced_robot.model.upperPositionLimit,
            )
        )

        self.opti.solver(
            "ipopt",
            {"ipopt": {"print_level": 0, "max_iter": ipopt_max_iter, "tol": ipopt_tol}, "print_time": False},
        )

        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.history_data = np.zeros(self.reduced_robot.model.nq)
        self.feedback_data = self.history_data.copy()
        self._state_synced = False
        self.last_solve = {}
        self.enable_visualization = enable_visualization
        self.vis = None
        self.viewer_target_frame_name = viewer_target_frame_name
        if self.enable_visualization:
            self._init_visualizer(
                open_browser=viewer_open_browser,
                viewer_model_name=viewer_model_name,
                target_frame_name=viewer_target_frame_name,
                axis_length=viewer_axis_length,
                axis_width=viewer_axis_width,
            )

    def _deduplicate_locked_joints(self, locked_joints: List[str]) -> List[str]:
        unique_names: List[str] = []
        seen_joint_ids = set()
        for joint_name in locked_joints:
            try:
                joint_id = self.robot.model.getJointId(joint_name)
            except Exception:
                continue
            if joint_id <= 0:
                continue
            if joint_id in seen_joint_ids:
                continue
            seen_joint_ids.add(joint_id)
            unique_names.append(joint_name)
        return unique_names

    @property
    def nq(self) -> int:
        return self.reduced_robot.model.nq

    def active_joint_names(self) -> List[str]:
        names = [n for n in self.reduced_robot.model.names if n != "universe"]
        return names

    def sync_state(self, q_current: List[float]) -> None:
        q = np.array(q_current, dtype=float)
        if q.shape != (self.nq,) or not np.isfinite(q).all():
            raise ValueError("IK 实测关节格式非法")
        self.init_data = q.copy()
        self.feedback_data = q.copy()
        if not self._state_synced:
            self.history_data = q.copy()
            self._state_synced = True

    def accept(self, q):
        """Commit continuity state only after all checks and publication pass."""
        self.init_data = np.asarray(q, dtype=float).copy()
        self.history_data = self.init_data.copy()

    def solve(self, target_pose: np.ndarray) -> np.ndarray:
        self.opti.set_initial(self.var_q, self.init_data)
        self.opti.set_value(self.param_q_prev, self.history_data)
        self.opti.set_value(self.param_q_feedback, self.feedback_data)
        self.opti.set_value(self.param_tf, target_pose)
        self.display_target(target_pose)

        begin = time.monotonic()
        self.last_solve = {}
        try:
            self.opti.solve_limited()
            if not self.opti.stats().get("success", False):
                raise ValueError("IK 未收敛")
            sol_q = np.array(self.opti.value(self.var_q)).reshape(-1)
            if not np.isfinite(sol_q).all():
                raise ValueError("IK 产生非有限关节角")
        finally:
            self.last_solve["solve_ms"] = round((time.monotonic() - begin) * 1000, 3)
            try:
                self.last_solve["solver_status"] = self.opti.stats().get("return_status")
                debug_q = np.asarray(self.opti.debug.value(self.var_q)).reshape(-1)
                if not np.isfinite(debug_q).all():
                    raise ValueError("求解器没有有限候选值")
                pose = self.end_effector_pose(debug_q)
                self.last_solve["position_error_mm"] = float(np.linalg.norm(pose[:3, 3] - target_pose[:3, 3]) * 1000)
                self.last_solve["rotation_error_deg"] = float(np.rad2deg(Rotation.from_matrix(pose[:3, :3].T @ target_pose[:3, :3]).magnitude()))
            except (RuntimeError, ValueError):
                pass
        self.display_solution(sol_q)
        return sol_q

    def check_self_collision(self, q: np.ndarray) -> bool:
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, q)
        pin.updateGeometryPlacements(self.reduced_robot.model, self.reduced_robot.data, self.geom_model, self.geometry_data)
        return pin.computeCollisions(self.geom_model, self.geometry_data, False)

    def end_effector_pose(self, q: np.ndarray) -> np.ndarray:
        """Return the base-frame pose of the configured IK end effector."""
        q_array = np.asarray(q, dtype=float)
        if q_array.shape != (self.nq,):
            raise ValueError(f"Expected {self.nq} joint values, got shape {q_array.shape}.")
        pin.framesForwardKinematics(self.reduced_robot.model, self.reduced_robot.data, q_array)
        return self.reduced_robot.data.oMf[self.ee_id].homogeneous.copy()

    def _init_visualizer(
        self,
        open_browser: bool,
        viewer_model_name: str,
        target_frame_name: str,
        axis_length: float,
        axis_width: float,
    ) -> None:
        import meshcat.geometry as mg
        from pinocchio.visualize import MeshcatVisualizer

        self.vis = MeshcatVisualizer(self.reduced_robot.model, self.reduced_robot.collision_model, self.reduced_robot.visual_model)
        self.vis.initViewer(open=open_browser)
        self.vis.loadViewerModel(viewer_model_name)
        self.vis.display(pin.neutral(self.reduced_robot.model))

        frame_axis_positions = (
            np.array([[0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 1, 0], [0, 0, 0], [0, 0, 1]]).astype(np.float32).T
        )
        frame_axis_colors = (
            np.array([[1, 0, 0], [1, 0.6, 0], [0, 1, 0], [0.6, 1, 0], [0, 0, 1], [0, 0.6, 1]]).astype(np.float32).T
        )
        self.vis.viewer[target_frame_name].set_object(
            mg.LineSegments(
                mg.PointsGeometry(position=axis_length * frame_axis_positions, color=frame_axis_colors),
                mg.LineBasicMaterial(linewidth=axis_width, vertexColors=True),
            )
        )

    def display_target(self, target_pose: np.ndarray) -> None:
        if self.vis is not None:
            self.vis.viewer[self.viewer_target_frame_name].set_transform(target_pose)

    def display_solution(self, q: np.ndarray) -> None:
        if self.vis is not None:
            self.vis.display(q)


class ArmIKPoseNode(Node):
    def __init__(self):
        super().__init__("arm_ik_pose_node")

        self.declare_parameter("robot_description_package", "nero_description")
        self.declare_parameter("urdf_relative_path", "urdf/nero.urdf")
        self.declare_parameter("locked_joints", ["joint8"])
        self.declare_parameter("ee_parent_joint", "joint7")
        self.declare_parameter("ee_frame_name", "ee")
        self.declare_parameter("tool_pre_rot_rpy", [-1.57, 0.0, -1.57])
        self.declare_parameter("tool_translation_xyz", [0.0, 0.023, 0.064])
        self.declare_parameter("tcp_offset_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("require_start_alignment", False)
        self.declare_parameter("alignment_position_tolerance_m", 0.01)
        self.declare_parameter("alignment_rotation_tolerance_rad", 0.0872664626)
        self.declare_parameter("max_tcp_step_m", 0.002)
        self.declare_parameter("max_tcp_step_rad", 0.01745329252)
        self.declare_parameter("collision_pairs_flat", [5, 0, 5, 1, 5, 2, 5, 3])
        self.declare_parameter("enable_collision_check", False)

        self.declare_parameter("w_pos", 20.0)
        self.declare_parameter("w_ori", 2.0)
        self.declare_parameter("w_reg", 0.01)
        self.declare_parameter("w_smooth", 2.0)
        self.declare_parameter("ipopt_max_iter", 50)
        self.declare_parameter("ipopt_tol", 1e-4)
        self.declare_parameter("enable_visualization", False)
        self.declare_parameter("viewer_open_browser", True)
        self.declare_parameter("viewer_model_name", "pinocchio")
        self.declare_parameter("viewer_target_frame_name", "ee_target")
        self.declare_parameter("viewer_axis_length", 0.1)
        self.declare_parameter("viewer_axis_width", 10.0)

        self.declare_parameter("pose_stamped_topic", "")
        self.declare_parameter("feedback_joint_topic", "")
        self.declare_parameter("pin_joint_status_topic", "pin_joint_status")
        self.declare_parameter("simulation_feedback", False)
        self.declare_parameter("simulation_feedback_joint_topic", "")
        self.declare_parameter("simulation_feedback_tcp_pose_topic", "")
        self.declare_parameter("simulation_feedback_rate_hz", 30.0)
        # NOTE: Empty list default is inferred as BYTE_ARRAY in rclpy.
        # Use string array default to keep YAML STRING_ARRAY override compatible.
        self.declare_parameter("output_joint_names", [""])

        package_name = self.get_parameter("robot_description_package").value
        urdf_rel = self.get_parameter("urdf_relative_path").value
        locked_joints = list(self.get_parameter("locked_joints").value)
        ee_parent_joint = self.get_parameter("ee_parent_joint").value
        ee_frame_name = self.get_parameter("ee_frame_name").value
        tool_pre_rot_rpy = list(self.get_parameter("tool_pre_rot_rpy").value)
        tool_translation_xyz = list(self.get_parameter("tool_translation_xyz").value)
        collision_pairs_flat = [int(v) for v in self.get_parameter("collision_pairs_flat").value]
        if len(collision_pairs_flat) % 2 != 0:
            raise ValueError("collision_pairs_flat length must be even, e.g. [5,0,5,1].")
        enable_collision_check = bool(self.get_parameter("enable_collision_check").value)

        w_pos = float(self.get_parameter("w_pos").value)
        w_ori = float(self.get_parameter("w_ori").value)
        w_reg = float(self.get_parameter("w_reg").value)
        w_smooth = float(self.get_parameter("w_smooth").value)
        ipopt_max_iter = int(self.get_parameter("ipopt_max_iter").value)
        ipopt_tol = float(self.get_parameter("ipopt_tol").value)
        enable_visualization = bool(self.get_parameter("enable_visualization").value)
        viewer_open_browser = bool(self.get_parameter("viewer_open_browser").value)
        viewer_model_name = self.get_parameter("viewer_model_name").value
        viewer_target_frame_name = self.get_parameter("viewer_target_frame_name").value
        viewer_axis_length = float(self.get_parameter("viewer_axis_length").value)
        viewer_axis_width = float(self.get_parameter("viewer_axis_width").value)

        pose_stamped_topic = str(self.get_parameter("pose_stamped_topic").value).strip()
        feedback_joint_topic = str(self.get_parameter("feedback_joint_topic").value).strip()
        pin_joint_status_topic = str(self.get_parameter("pin_joint_status_topic").value).strip()
        self.simulation_feedback = bool(self.get_parameter("simulation_feedback").value)
        simulation_feedback_joint_topic = str(
            self.get_parameter("simulation_feedback_joint_topic").value
        ).strip()
        simulation_feedback_tcp_pose_topic = str(
            self.get_parameter("simulation_feedback_tcp_pose_topic").value
        ).strip()
        simulation_feedback_rate_hz = float(
            self.get_parameter("simulation_feedback_rate_hz").value
        )

        package_path = get_package_share_directory(package_name)
        urdf_path = os.path.join(package_path, urdf_rel)
        self.ik = ArmIK(
            urdf_path=urdf_path,
            package_dirs=[os.path.dirname(package_path), package_path],
            locked_joints=locked_joints,
            ee_parent_joint=ee_parent_joint,
            ee_frame_name=ee_frame_name,
            tool_pre_rot_rpy=tool_pre_rot_rpy,
            tool_translation_xyz=tool_translation_xyz,
            collision_pairs_flat=collision_pairs_flat,
            w_pos=w_pos,
            w_ori=w_ori,
            w_reg=w_reg,
            w_smooth=w_smooth,
            ipopt_max_iter=ipopt_max_iter,
            ipopt_tol=ipopt_tol,
            enable_visualization=enable_visualization,
            viewer_open_browser=viewer_open_browser,
            viewer_model_name=viewer_model_name,
            viewer_target_frame_name=viewer_target_frame_name,
            viewer_axis_length=viewer_axis_length,
            viewer_axis_width=viewer_axis_width,
            tcp_offset_rpy=list(self.get_parameter("tcp_offset_rpy").value),
            position_tolerance_m=float(self.get_parameter("alignment_position_tolerance_m").value),
            rotation_tolerance_rad=float(self.get_parameter("alignment_rotation_tolerance_rad").value),
        )
        dedup_locked = self.ik._deduplicate_locked_joints(locked_joints)
        self.get_logger().info(f"locked_joints(raw)={locked_joints}, locked_joints(dedup)={dedup_locked}")

        output_joint_names = list(self.get_parameter("output_joint_names").value)
        #self.output_joint_names = output_joint_names if len(output_joint_names) == self.ik.nq else self.ik.active_joint_names()
        self.output_joint_names = output_joint_names
        self.require_start_alignment = bool(self.get_parameter("require_start_alignment").value)
        self.alignment_position_tolerance = float(self.get_parameter("alignment_position_tolerance_m").value)
        self.alignment_rotation_tolerance = float(self.get_parameter("alignment_rotation_tolerance_rad").value)
        self._teleop_active = not self.require_start_alignment
        self._needs_alignment = True
        self._feedback_q = None
        self._feedback_received_ns = 0
        self._reference_correction = np.eye(4)
        self._reference_q = None
        self._reference_pose = None
        self._last_accepted_pose = None
        self._ik_warmed = False
        self.max_tcp_step_m = float(self.get_parameter("max_tcp_step_m").value)
        self.max_tcp_step_rad = float(self.get_parameter("max_tcp_step_rad").value)
        if not all(np.isfinite(v) and v > 0 for v in (self.max_tcp_step_m, self.max_tcp_step_rad)):
            raise ValueError("TCP 步长必须为有限正数")
        self.create_subscription(Bool, "/teleop/enabled", self.teleop_enabled_callback, 10)
        
        self.pub_joint = self.create_publisher(JointState, pin_joint_status_topic, 10)
        self.pub_collision = self.create_publisher(Bool, f"{pin_joint_status_topic}_collision", 10)
        self.enable_collision_check = enable_collision_check

        if pose_stamped_topic:
            self.create_subscription(PoseStamped, pose_stamped_topic, self.pose_stamped_callback, 10)
        if feedback_joint_topic:
            self.create_subscription(JointState, feedback_joint_topic, self.feedback_joint_callback, 10)
        if not pose_stamped_topic:
            raise ValueError("pose_stamped_topic cannot be empty.")

        self._simulated_q = np.zeros(self.ik.nq)
        self.pub_sim_joint = None
        self.pub_sim_tcp = None
        self.simulation_feedback_timer = None
        if self.simulation_feedback:
            simulation_feedback_joint_topic = simulation_feedback_joint_topic or feedback_joint_topic
            if not simulation_feedback_joint_topic or not simulation_feedback_tcp_pose_topic:
                raise ValueError(
                    "simulation_feedback requires simulation_feedback_joint_topic and "
                    "simulation_feedback_tcp_pose_topic."
                )
            self.pub_sim_joint = self.create_publisher(JointState, simulation_feedback_joint_topic, 10)
            self.pub_sim_tcp = self.create_publisher(PoseStamped, simulation_feedback_tcp_pose_topic, 10)
            self.simulation_feedback_timer = self.create_timer(
                1.0 / max(simulation_feedback_rate_hz, 1.0),
                self._publish_simulation_feedback,
            )
            self._publish_simulation_feedback()

        self.get_logger().info(
            f"IK node ready. URDF={urdf_path}, input=({pose_stamped_topic}), "
            f"output={pin_joint_status_topic}, nq={self.ik.nq}"
        )
        if self.simulation_feedback:
            self.get_logger().info(
                "RViz-only simulation feedback is enabled; no CAN driver is used by this node."
            )

    def feedback_joint_callback(self, msg: JointState) -> None:
        values = dict(zip(msg.name, msg.position))
        names = self.ik.active_joint_names()
        if all(name in values for name in names):
            q = np.array([values[name] for name in names], dtype=float)
            if np.isfinite(q).all():
                self._feedback_q = q
                self._feedback_received_ns = self.get_clock().now().nanoseconds
                self.ik.sync_state(q.tolist())
                if not self._ik_warmed and not self._teleop_active:
                    # Compile/load the solver before A can start a 250 ms target watchdog.
                    self._ik_warmed = True
                    try:
                        self.ik.solve(self.ik.end_effector_pose(q))
                        self.get_logger().info("IK 已完成静止反馈预热，未发布运动目标")
                    except Exception as exc:
                        self._ik_warmed = False
                        self.get_logger().warning(f"IK 预热未通过：{exc}")

    def teleop_enabled_callback(self, msg: Bool) -> None:
        self._teleop_active = msg.data
        self._needs_alignment = True
        self._last_accepted_pose = None

    def pose_stamped_callback(self, msg: PoseStamped) -> None:
        self._solve_and_publish(pose_to_mat(msg.pose), stamp=msg.header.stamp)

    def _publish_simulation_feedback(self, stamp=None) -> None:
        """Publish simulated state so delta teleoperation can rebase without a real arm."""
        if self.pub_sim_joint is None or self.pub_sim_tcp is None:
            return

        if stamp is None:
            stamp = self.get_clock().now().to_msg()

        joint_msg = JointState()
        joint_msg.header.stamp = stamp
        joint_msg.name = self.output_joint_names
        joint_msg.position = self._output_positions(self._simulated_q)
        self.pub_sim_joint.publish(joint_msg)

        tcp_matrix = self.ik.end_effector_pose(self._simulated_q)
        quat = Rotation.from_matrix(tcp_matrix[:3, :3]).as_quat()
        tcp_msg = PoseStamped()
        tcp_msg.header.stamp = stamp
        tcp_msg.header.frame_id = "base_link"
        tcp_msg.pose.position.x = float(tcp_matrix[0, 3])
        tcp_msg.pose.position.y = float(tcp_matrix[1, 3])
        tcp_msg.pose.position.z = float(tcp_matrix[2, 3])
        tcp_msg.pose.orientation.x = float(quat[0])
        tcp_msg.pose.orientation.y = float(quat[1])
        tcp_msg.pose.orientation.z = float(quat[2])
        tcp_msg.pose.orientation.w = float(quat[3])
        self.pub_sim_tcp.publish(tcp_msg)

    def _output_positions(self, q: np.ndarray) -> List[float]:
        if len(self.output_joint_names) > len(q):
            raise ValueError(
                f"output_joint_names has {len(self.output_joint_names)} entries, "
                f"but IK produced only {len(q)} joint values."
            )
        return q[: len(self.output_joint_names)].tolist()

    def _solve_and_publish(self, target_pose: np.ndarray, stamp) -> None:
        if not self._teleop_active:
            return
        requested_pose = target_pose.copy()
        limited_pose = None
        self.ik.last_solve = {}
        try:
            first_alignment = False
            if self.require_start_alignment:
                if self._feedback_q is None or not 0 <= self.get_clock().now().nanoseconds - self._feedback_received_ns <= 250_000_000:
                    raise ValueError("无新鲜六轴反馈，拒绝 IK 输出")
                if self._needs_alignment:
                    model_pose = self.ik.end_effector_pose(self._feedback_q)
                    distance = np.linalg.norm(model_pose[:3, 3] - target_pose[:3, 3])
                    angle = Rotation.from_matrix(model_pose[:3, :3].T @ target_pose[:3, :3]).magnitude()
                    if distance > self.alignment_position_tolerance or angle > self.alignment_rotation_tolerance:
                        raise ValueError(f"启动 TCP 与模型不符：{distance * 1000:.1f} mm / {np.rad2deg(angle):.2f}°")
                    self._reference_correction = model_pose @ np.linalg.inv(target_pose)
                    self._reference_q = self._feedback_q.copy()
                    self._reference_pose = model_pose.copy()
                    self._last_accepted_pose = model_pose.copy()
                    self.ik.accept(self._feedback_q)
                    self._needs_alignment = False
                    first_alignment = True
                    self.get_logger().info("IK 已对齐实测关节与当前 TCP；首条输出保持实际关节角")
                target_pose = self._reference_correction @ target_pose
                requested_pose = target_pose.copy()
                if first_alignment:
                    sol_q = self._reference_q.copy()
                    target_pose = self._reference_pose.copy()
                else:
                    target_pose = limit_tcp_step(self._last_accepted_pose, target_pose, self.max_tcp_step_m, self.max_tcp_step_rad)
                    limited_pose = target_pose.copy()
                    sol_q = self.ik.solve(target_pose)
            else:
                previous = self._last_accepted_pose if self._last_accepted_pose is not None else self.ik.end_effector_pose(self.ik.history_data)
                target_pose = limit_tcp_step(previous, target_pose, self.max_tcp_step_m, self.max_tcp_step_rad)
                limited_pose = target_pose.copy()
                sol_q = self.ik.solve(target_pose)
            model_pose = self.ik.end_effector_pose(sol_q)
            distance = np.linalg.norm(model_pose[:3, 3] - target_pose[:3, 3])
            angle = Rotation.from_matrix(model_pose[:3, :3].T @ target_pose[:3, :3]).magnitude()
            if distance > self.alignment_position_tolerance or angle > self.alignment_rotation_tolerance:
                raise ValueError(f"IK 末端误差超限: {distance*1000:.3f} mm / {np.rad2deg(angle):.3f}°")
            joint_msg = JointState()
            joint_msg.header.stamp = stamp
            joint_msg.name = self.output_joint_names
            joint_msg.position = self._output_positions(sol_q)
            self.pub_joint.publish(joint_msg)
            self.ik.accept(sol_q)
            self._last_accepted_pose = model_pose.copy()

            if self.simulation_feedback:
                self._simulated_q = sol_q
                self._publish_simulation_feedback(stamp)

            if self.enable_collision_check:
                col_msg = Bool()
                col_msg.data = self.ik.check_self_collision(sol_q)
                self.pub_collision.publish(col_msg)
        except Exception as e:
            def description(pose):
                if pose is None or not np.isfinite(pose).all():
                    return None
                return {"xyz_m": pose[:3, 3].round(6).tolist(), "quat_xyzw": Rotation.from_matrix(pose[:3, :3]).as_quat().round(6).tolist()}
            details = {
                "reason": str(e), "requested_tcp": description(requested_pose),
                "limited_tcp": description(limited_pose), "limit_mm": self.alignment_position_tolerance*1000,
                "limit_deg": float(np.rad2deg(self.alignment_rotation_tolerance)),
                "feedback_age_ms": (self.get_clock().now().nanoseconds-self._feedback_received_ns)/1e6 if self._feedback_received_ns else None,
                **self.ik.last_solve,
            }
            self.get_logger().warning("IK solve failed: " + json.dumps(details, ensure_ascii=False))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArmIKPoseNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
