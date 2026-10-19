#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing import Optional, TYPE_CHECKING
from dataclasses import dataclass
import time

if TYPE_CHECKING:
    from pyAgxArm.protocols.can_protocol.drivers.core.arm_driver_abstract import ArmDriverAbstract
    from pyAgxArm.protocols.can_protocol.drivers import AgxGripperDriverDefault


@dataclass
class GripperStatus:
    width: float = 0.0                  # Current gripper opening width, unit: m
    force: float = 0.0                  # Current gripping force, unit: N
    voltage_too_low: bool = False       # Voltage too low
    motor_overheating: bool = False     # Motor overheating
    driver_overcurrent: bool = False    # Driver overcurrent
    driver_overheating: bool = False    # Driver overheating
    sensor_status: bool = False         # Sensor status
    driver_error_status: bool = False   # Driver error status
    driver_enable_status: bool = False  # Driver enable status
    homing_status: bool = False         # Homing/zero position status
    hz: float = 0.0                     # Message receiving frequency
    timestamp: float = 0.0              # Timestamp


@dataclass
class GripperCtrlStatus:
    width: float = 0.0          # Current gripper opening width, unit: m
    force: float = 0.0          # Current gripping force, unit: N
    status_code: int = 0        # Status code
    set_zero: int = 0           # Homing/zeroing field
    hz: float = 0.0             # Message receiving frequency
    timestamp: float = 0.0      # Timestamp


@dataclass
class GripperTeachingParam:
    teaching_range_per: int = 100   # Teaching range (percentage), range: [100, 200]
    max_range_config: float = 0.0   # Maximum stroke configuration, unit: m, only supports: 0/0.07/0.1
    teaching_friction: int = 1      # Teaching friction, range: 1..10
    hz: float = 0.0
    timestamp: float = 0.0


class AgxGripperWrapper:
    
    # Gripper control parameter ranges
    WIDTH_MIN: float = 0.0      # minimum width, unit: m
    WIDTH_MAX: float = 0.1      # maximum width, unit: m
    FORCE_MIN: float = 0.5      # minimum force, unit: N
    FORCE_MAX: float = 3.0      # maximum force, unit: N
    
    def __init__(self, agx_arm):
        self._agx_arm: Optional[ArmDriverAbstract] = agx_arm
        self._effector: Optional[AgxGripperDriverDefault] = None
        self._initialized = False
        self._last_enable_clear_time = float("-inf")
    
    def initialize(self) -> bool:
        if self._initialized:
            return True
        
        try:
            self._effector = self._agx_arm.init_effector(
                self._agx_arm.OPTIONS.EFFECTOR.AGX_GRIPPER
            )
            self._initialized = True
            return True
        except Exception as e:
            print(f"[AgxGripperWrapper] initialize failed: {e}")
            return False
    
    def is_ok(self) -> bool:
        if not self._initialized or self._effector is None:
            return False
        return self._effector.is_ok()
    
    def get_fps(self) -> float:
        if not self._initialized or self._effector is None:
            return 0.0
        return self._effector.get_fps()
    
    def get_status(self) -> Optional[GripperStatus]:
        if not self._initialized or self._effector is None:
            return None
        
        gs = self._effector.get_gripper_status()
        if gs is None:
            return None
        
        status = GripperStatus(
            width=gs.msg.value,
            force=gs.msg.force,
            voltage_too_low=gs.msg.foc_status.voltage_too_low,
            motor_overheating=gs.msg.foc_status.motor_overheating,
            driver_overcurrent=gs.msg.foc_status.driver_overcurrent,
            driver_overheating=gs.msg.foc_status.driver_overheating,
            sensor_status=gs.msg.foc_status.sensor_status,
            driver_error_status=gs.msg.foc_status.driver_error_status,
            driver_enable_status=gs.msg.foc_status.driver_enable_status,
            homing_status=gs.msg.foc_status.homing_status,
            hz=gs.hz,
            timestamp=gs.timestamp
        )
        return status
    
    def get_ctrl_states(self) -> Optional[GripperCtrlStatus]:
        if not self._initialized or self._effector is None:
            return None
        
        gcs = self._effector.get_gripper_ctrl_states()
        if gcs is None:
            return None
        
        ctrl_status = GripperCtrlStatus(
            width=gcs.msg.value,
            force=gcs.msg.force,
            status_code=gcs.msg.status_code,
            set_zero=gcs.msg.set_zero,
            hz=gcs.hz,
            timestamp=gcs.timestamp
        )
        return ctrl_status
    
    def get_teaching_param(
        self, 
        timeout: float = 1.0, 
        min_interval: float = 1.0
    ) -> Optional[GripperTeachingParam]:
        if not self._initialized or self._effector is None:
            return None
        
        param = self._effector.get_gripper_teaching_pendant_param(
            timeout=timeout, 
            min_interval=min_interval
        )
        if param is None:
            return None
        
        teaching_pendant_param = GripperTeachingParam(
            teaching_range_per=param.msg.teaching_range_per,
            max_range_config=param.msg.max_range_config,
            teaching_friction=param.msg.teaching_friction,
            hz=param.hz,
            timestamp=param.timestamp
        )
        return teaching_pendant_param
    
    def move(
        self,
        width: float = 0.0,
        force: float = 1.0,
        clear_disabled: bool = True,
    ) -> bool:
        if not self._initialized or self._effector is None:
            return False
        
        # Parameter range check
        if not (self.WIDTH_MIN <= width <= self.WIDTH_MAX):
            raise ValueError(
                f"width must be in range [{self.WIDTH_MIN}, {self.WIDTH_MAX}], current value: {width}"
            )
        if not (self.FORCE_MIN <= force <= self.FORCE_MAX):
            raise ValueError(
                f"force must be in range [{self.FORCE_MIN}, {self.FORCE_MAX}], current value: {force}"
            )
        
        try:
            status = self.get_status()
            now = time.monotonic()
            if (
                clear_disabled
                and status is not None
                and not status.driver_enable_status
                and now - self._last_enable_clear_time >= 0.5
            ):
                # Clear a previously disabled/error state before sending the
                # normal enable+width command. Repeating this at most twice a
                # second avoids keeping the gripper disabled between commands.
                self._effector.reset_gripper()
                self._last_enable_clear_time = now
            self._effector.move_gripper_m(value=width, force=force)
            return True
        except Exception as e:
            print(f"[AgxGripperWrapper] Control gripper failed: {e}")
            return False
    
    def open(self, width: float = 0.1, force: float = 1.0) -> bool:
        return self.move(width=width, force=force)
    
    def close(self, force: float = 1.0) -> bool:
        return self.move(width=0.0, force=force)
    
    def disable(self) -> bool:
        if not self._initialized or self._effector is None:
            return False
        
        return self._effector.disable_gripper()
    
    def calibrate(self, timeout: float = 1.0) -> bool:
        if not self._initialized or self._effector is None:
            return False
        
        return self._effector.calibrate_gripper(timeout=timeout)
    
    def set_teaching_param(
        self,
        teaching_range_per: int = 100,
        max_range_config: float = 0.0,
        teaching_friction: int = 1,
        timeout: float = 1.0
    ) -> bool:
        if not self._initialized or self._effector is None:
            return False
        
        return self._effector.set_gripper_teaching_pendant_param(
            teaching_range_per=teaching_range_per,
            max_range_config=max_range_config,
            teaching_friction=teaching_friction,
            timeout=timeout
        )
