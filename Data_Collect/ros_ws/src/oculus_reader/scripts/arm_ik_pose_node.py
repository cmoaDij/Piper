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
    ):
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
        """Commit continuity state after publishing the solved joint target."""
        self.init_data = np.asarray(q, dtype=float).copy()
        self.history_data = self.init_data.copy()

    def rebase(self, q):
        """Start a clutch segment at the joint setpoint actually held downstream."""
        self.sync_state(q)
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
            sol_q = np.array(self.opti.value(self.var_q)).reshape(-1)
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
        # Optional: other Quest launch files keep their absolute-pose interface.
        self.declare_parameter("reference_joint_topic", "")
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
        reference_joint_topic = str(self.get_parameter("reference_joint_topic").value).strip()
        self.use_clutch_reference = bool(reference_joint_topic)
        self.reference_joint = None
        self.reference_model_pose = None
        self.reference_input_pose = None
        self.reference_stamp_ns = 0
        self.reference_key = ""
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
        )
        dedup_locked = self.ik._deduplicate_locked_joints(locked_joints)
        self.get_logger().info(f"locked_joints(raw)={locked_joints}, locked_joints(dedup)={dedup_locked}")

        output_joint_names = list(self.get_parameter("output_joint_names").value)
        #self.output_joint_names = output_joint_names if len(output_joint_names) == self.ik.nq else self.ik.active_joint_names()
        self.output_joint_names = output_joint_names
        self._ik_warmed = False

        self.pub_joint = self.create_publisher(JointState, pin_joint_status_topic, 10)
        self.pub_collision = self.create_publisher(Bool, f"{pin_joint_status_topic}_collision", 10)
        self.enable_collision_check = enable_collision_check

        if pose_stamped_topic:
            self.create_subscription(PoseStamped, pose_stamped_topic, self.pose_stamped_callback, 10)
        if feedback_joint_topic:
            self.create_subscription(JointState, feedback_joint_topic, self.feedback_joint_callback, 10)
        if reference_joint_topic:
            self.create_subscription(JointState, reference_joint_topic, self.reference_joint_callback, 10)
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
                self.ik.sync_state(q.tolist())
                if not self._ik_warmed:
                    # Compile/load once without publishing a motion target.
                    self._ik_warmed = True
                    try:
                        self.ik.solve(self.ik.end_effector_pose(q))
                        self.get_logger().info("IK 已完成静止反馈预热，未发布运动目标")
                    except Exception as exc:
                        self._ik_warmed = False
                        self.get_logger().warning(f"IK 预热未通过：{exc}")

    def reference_joint_callback(self, msg: JointState) -> None:
        values = dict(zip(msg.name, msg.position))
        names = self.ik.active_joint_names()
        if not all(name in values for name in names):
            return
        q = np.array([values[name] for name in names], dtype=float)
        if not np.isfinite(q).all():
            return
        self.ik.rebase(q)
        self.reference_joint = q.copy()
        self.reference_model_pose = self.ik.end_effector_pose(q)
        self.reference_input_pose = None
        self.reference_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.reference_key = msg.header.frame_id
        self.get_logger().info("RG：IK 已对齐下游保持目标，等待本段首个手柄目标")

    def pose_stamped_callback(self, msg: PoseStamped) -> None:
        target = pose_to_mat(msg.pose)
        hold_joint = None
        if self.use_clutch_reference:
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if self.reference_joint is None or stamp_ns < self.reference_stamp_ns:
                return
            if self.reference_input_pose is None:
                self.reference_input_pose = target.copy()
                self.get_logger().info("RG：首帧重复原关节目标，无零增量 IK 运动")
            # The input already maps controller motion into base axes using the
            # existing TCP formula. Preserve those increments exactly, replacing
            # only their absolute origin with FK of the held joint command.
            rebased = self.reference_model_pose.copy()
            rebased[:3, 3] += target[:3, 3] - self.reference_input_pose[:3, 3]
            rebased[:3, :3] = (
                target[:3, :3] @ self.reference_input_pose[:3, :3].T
                @ self.reference_model_pose[:3, :3]
            )
            # Roundoff tolerance, not a controller motion deadband. Avoid solving
            # a zero increment back into a different (regularized) joint target.
            if np.allclose(target, self.reference_input_pose, rtol=0., atol=1e-12):
                hold_joint = self.reference_joint
            target = rebased
        self._solve_and_publish(target, stamp=msg.header.stamp, hold_joint=hold_joint)

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

    def _solve_and_publish(self, target_pose: np.ndarray, stamp, hold_joint=None) -> None:
        try:
            sol_q = self.ik.solve(target_pose) if hold_joint is None else hold_joint.copy()
            joint_msg = JointState()
            joint_msg.header.stamp = stamp
            joint_msg.header.frame_id = getattr(self, "reference_key", "")
            joint_msg.name = self.output_joint_names
            joint_msg.position = self._output_positions(sol_q)
            self.pub_joint.publish(joint_msg)
            self.ik.accept(sol_q)
            if self.simulation_feedback:
                self._simulated_q = sol_q
                self._publish_simulation_feedback(stamp)
            if self.enable_collision_check:
                col_msg = Bool()
                col_msg.data = self.ik.check_self_collision(sol_q)
                self.pub_collision.publish(col_msg)
        except Exception as exc:
            self.get_logger().warning("IK solve failed: " + json.dumps(
                {"reason": str(exc), **self.ik.last_solve}, ensure_ascii=False
            ))


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
