from typing import Optional
from typing_extensions import Literal

from .....msgs.core import MessageAbstract
from .....msgs.nero.default import (
    ArmMsgFeedbackHighSpd,
)
from .....msgs.piper.default import ArmMsgFeedbackCPVResponse
from ...versions.v112.driver import Driver as V112Driver


class Driver(V112Driver):
    """Nero CAN driver for firmware == v120 (1.20).

    Terminology
    -----------
    `flange`:
    - The mounting face / connection interface on the robotic arm's last link
      (mechanical tool interface).

    Common conventions
    ------------------
    `timeout` (for request/response style APIs):
    - `timeout < 0.0` raises ValueError.
    - `timeout == 0.0`: non-blocking; evaluate readiness once and return
      immediately.
    - `timeout > 0.0`: blocking; poll until ready or timeout expires.

    `joint_index`:
    - `joint_index == 255` means "all joints".

    `set_*` return semantics:
    - Many `set_*` APIs are ACK-only: True means the controller acknowledged the
      request.
      This does not strictly guarantee the setting is already applied.
    - Some `set_*` APIs additionally verify by reading back state; their
      docstrings will mention the verification method if applicable.
    """

    def get_motor_states(self, joint_index: Literal[1, 2, 3, 4, 5, 6, 7]):
        """Get high-speed motor state feedback.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]
        - 1~7: get the motor state of the specified joint

        Returns
        -------
        MessageAbstract[ArmMsgFeedbackHighSpd] | None
            The specified joint's motor state, or None if not available.

        Message
        -------
        `position`: Current motor position, unit: rad

        `velocity`: Current motor speed, unit: rad/s

        `current`: Current motor current, unit: A

        `torque`: Current motor torque, unit: N·m

        Examples
        --------
        >>> ms = robot.get_motor_states(1)
        >>> if ms is not None:
        >>>     print(ms.msg.position, ms.msg.velocity, ms.msg.torque)
        >>>     print(ms.hz, ms.timestamp)
        """
        if joint_index not in self._JOINT_INDEX_LIST[:-1]:
            raise ValueError(
                f"Joint index should be {self._JOINT_INDEX_LIST[:-1]}")

        motor_state: Optional[
            MessageAbstract[ArmMsgFeedbackHighSpd]
        ] = getattr(self._parser, f"motor_state_{joint_index}", None)
        if motor_state is not None:
            motor_state.hz = self._ctx.fps.get_fps(motor_state.msg_type)
            return motor_state
        else:
            return None

    def set_joint_acc_limits(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7, 255] = 255,
        max_joint_acc: Optional[float] = None,
        timeout: float = 1.0,
    ):
        """Set the joint acceleration limits.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7, 255]
        - 1~7: set the joint acceleration limits of the specified joint.
        - 255: set the joint acceleration limits of all joints.

        `max_joint_acc`: float
        - The maximum joint acceleration in rad/s^2.
            (Numerical precision: 1e-3 rad/s^2)

        `timeout`: float, optional
        - Timeout in seconds. Default is 1.0.

        Returns
        -------
        bool
            True if the maximum joint acceleration is set successfully, False
            otherwise.
        """
        self._ctx._validate_timeout(timeout)
        if joint_index not in self._JOINT_INDEX_LIST:
            raise ValueError(f"Joint index should be {self._JOINT_INDEX_LIST}")

        if joint_index == 255:
            return self._all_joints_bool(
                lambda i: self.set_joint_acc_limits(i, max_joint_acc)
            )

        max_joint_acc = (
            0x7FFF if max_joint_acc is None else round(abs(max_joint_acc) * 1e2)
        )

        def request() -> None:
            self._send_msg(
                self._MSG_JointConfig(
                    joint_index=joint_index,
                    acc_param_config_is_effective_or_not=0xAE,
                    max_joint_acc=max_joint_acc,
                )
            )

        def check() -> bool:
            res = self.get_joint_acc_limits(joint_index)
            return not (
                res is None
                or max_joint_acc != 0x7FFF
                and max_joint_acc != round(abs(res.msg.max_joint_acc) * 1e2)
            )

        return self._check_set_by_readback(
            request=request,
            check=check,
            timeout=timeout,
            stamp_key=f"set_joint_acc_limits:{joint_index}",
        )

    # -------------------------- CPV --------------------------

    def _cpv_write_ack_received(
        self,
        msg: MessageAbstract[ArmMsgFeedbackCPVResponse],
        type_: Literal['ac', 'dc', 'vv', 'pp', 'kp', 'ki'],
    ) -> bool:
        return msg.msg.write_ack

    def _clear_cpv_write_ack(
        self,
        msg: MessageAbstract[ArmMsgFeedbackCPVResponse],
        type_: Literal['ac', 'dc', 'vv', 'pp', 'kp', 'ki'],
    ) -> None:
        msg.msg.write_ack = False

    def get_cpv_vel(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6, 7],
        timeout: float = 1.0,
        min_interval: float = 1.0,
    ) -> Optional[float]:
        """Read joint velocity from the CPV feedback channel.

        Issues a CPV read request and waits for the corresponding response
        field on the parser.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6, 7]

        `timeout`: float, optional
        - Wait time in seconds. Default is 1.0.

        `min_interval`: float, optional
        - Minimum spacing between requests. Default is 1.0.

        Returns
        -------
        Optional[float]
            Velocity in rad/s, or None on timeout.
        """
        return self._get_cpv(
            joint_index=joint_index,
            type_='sp',
            timeout=timeout,
            min_interval=min_interval,
        )
