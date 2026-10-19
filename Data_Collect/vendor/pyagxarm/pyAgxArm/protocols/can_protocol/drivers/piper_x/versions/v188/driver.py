from typing_extensions import Literal

from ....piper.versions.v188.driver import Driver as PiperDriverV188


class Driver(PiperDriverV188):
    """
    PiperX CAN driver for firmware == v188 (S-V1.8-8).

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
      request, but it does not strictly guarantee the setting is already applied.
    - Some `set_*` APIs additionally verify by reading back state; their
      docstrings will mention the verification method if applicable.
    """

    def move_mit(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        p_des: float = 0.0,
        v_des: float = 0.0,
        kp: float = 10.0,
        kd: float = 0.8,
        t_ff: float = 0.0,
    ):
        # TODO: remove this after the bug is fixed
        # corrected in version S-V1.8-9
        if joint_index in [4, 5]:
            p_des = -p_des
            v_des = -v_des
            t_ff = -t_ff
        super().move_mit(
            joint_index,
            p_des=p_des,
            v_des=v_des,
            kp=kp,
            kd=kd,
            t_ff=t_ff,
        )

    def move_cpv_pos(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        pos: float,
    ) -> None:
        # TODO: remove this after the bug is fixed
        if joint_index in [4, 5]:
            pos = -pos
        super().move_cpv_pos(
            joint_index,
            pos,
        )

    def move_cpv_vel(
        self,
        joint_index: Literal[1, 2, 3, 4, 5, 6],
        vel: float,
    ) -> None:
        """Command joint velocity reference in CPV motion mode.

        Parameters
        ----------
        `joint_index`: Literal[1, 2, 3, 4, 5, 6]

        `vel`: float
        - Desired joint velocity in rad/s.
        """
        # TODO: remove this after the bug is fixed
        if joint_index in (2, 3):
            vel *= -1
        self._move_cpv(
            joint_index=joint_index,
            type_='sp',
            value=vel,
        )
