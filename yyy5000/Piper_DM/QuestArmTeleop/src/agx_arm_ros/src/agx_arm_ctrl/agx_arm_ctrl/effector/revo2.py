#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing import Optional, List, Literal, TYPE_CHECKING, Dict
from dataclasses import dataclass

if TYPE_CHECKING:
    from pyAgxArm.protocols.can_protocol.drivers.core.arm_driver_abstract import ArmDriverAbstract
    from pyAgxArm.protocols.can_protocol.drivers import Revo2DriverDefault

@dataclass
class HandStatus:
    left_or_right: int = 0      # Hand flag: 01 left hand; 02 right hand
    thumb_tip: int = 0          # Thumb tip motor status: 0 idle; 1 running; 2 blocked
    thumb_base: int = 0         # Thumb base motor status: 0 idle; 1 running; 2 blocked
    index_finger: int = 0       # Index finger motor status: 0 idle; 1 running; 2 blocked
    middle_finger: int = 0      # Middle finger motor status: 0 idle; 1 running; 2 blocked
    ring_finger: int = 0        # Ring finger motor status: 0 idle; 1 running; 2 blocked
    pinky_finger: int = 0       # Pinky finger motor status: 0 idle; 1 running; 2 blocked
    hz: float = 0.0             # Message receiving frequency
    timestamp: float = 0.0      # Timestamp

@dataclass
class FingerPosition:
    thumb_tip: int = 0          # Thumb tip position, range: [0, 100]
    thumb_base: int = 0         # Thumb base position, range: [0, 100]
    index_finger: int = 0       # Index finger position, range: [0, 100]
    middle_finger: int = 0      # Middle finger position, range: [0, 100]
    ring_finger: int = 0        # Ring finger position, range: [0, 100]
    pinky_finger: int = 0       # Pinky finger position, range: [0, 100]
    hz: float = 0.0             # Message receiving frequency
    timestamp: float = 0.0      # Timestamp

@dataclass
class FingerSpeed:
    thumb_tip: int = 0          # Thumb tip speed, range: [-100, 100]
    thumb_base: int = 0         # Thumb base speed, range: [-100, 100]
    index_finger: int = 0       # Index finger speed, range: [-100, 100]
    middle_finger: int = 0      # Middle finger speed, range: [-100, 100]
    ring_finger: int = 0        # Ring finger speed, range: [-100, 100]
    pinky_finger: int = 0       # Pinky finger speed, range: [-100, 100]
    hz: float = 0.0             # Message receiving frequency
    timestamp: float = 0.0      # Timestamp

@dataclass
class FingerCurrent:
    thumb_tip: int = 0          # Thumb tip current, range: [-100, 100]
    thumb_base: int = 0         # Thumb base current, range: [-100, 100]
    index_finger: int = 0       # Index finger current, range: [-100, 100]
    middle_finger: int = 0      # Middle finger current, range: [-100, 100]
    ring_finger: int = 0        # Ring finger current, range: [-100, 100]
    pinky_finger: int = 0       # Pinky finger current, range: [-100, 100]
    hz: float = 0.0             # Message receiving frequency
    timestamp: float = 0.0      # Timestamp


class Revo2Wrapper:

    # Finger name list
    FINGER_NAMES: List[str] = [
        'thumb_tip', 'thumb_base',
        'index_finger', 'middle_finger',
        'ring_finger', 'pinky_finger'
    ]
    
    # Position/current/speed control parameter ranges
    POSITION_MIN: int = 0
    POSITION_MAX: int = 100
    SPEED_MIN: int = -100
    SPEED_MAX: int = 100
    CURRENT_MIN: int = -100
    CURRENT_MAX: int = 100
    TIME_MIN: int = 0
    TIME_MAX: int = 255  # Unit: 10ms
    DEFAULT_POSITION_TIME: int = 5
    
    # Motor status constants
    MOTOR_STATUS_IDLE: int = 0      # Idle
    MOTOR_STATUS_RUNNING: int = 1   # Running
    MOTOR_STATUS_BLOCKED: int = 2   # Blocked
    
    # Hand flags
    HAND_LEFT: int = 1
    HAND_RIGHT: int = 2
    
    def __init__(self, agx_arm):
        self._agx_arm: Optional[ArmDriverAbstract] = agx_arm
        self._effector: Optional[Revo2DriverDefault] = None
        self._initialized: bool = False
    
    def initialize(self) -> bool:
        if self._initialized:
            return True
        
        try:
            self._effector = self._agx_arm.init_effector(
                self._agx_arm.OPTIONS.EFFECTOR.REVO2
            )
            self._initialized = True
            return True
        except Exception as e:
            print(f"[Revo2Wrapper] Initialization failed: {e}")
            return False
    
    def is_ok(self) -> bool:
        if not self._initialized or self._effector is None:
            return False
        return self._effector.is_ok()
    
    def get_fps(self) -> float:
        if not self._initialized or self._effector is None:
            return 0.0
        return self._effector.get_fps()
    
    def get_status(self) -> Optional[HandStatus]:
        if not self._initialized or self._effector is None:
            return None
        
        hs = self._effector.get_hand_status()
        if hs is None:
            return None
        
        status = HandStatus(
            left_or_right=hs.msg.left_or_right,
            thumb_tip=hs.msg.thumb_tip,
            thumb_base=hs.msg.thumb_base,
            index_finger=hs.msg.index_finger,
            middle_finger=hs.msg.middle_finger,
            ring_finger=hs.msg.ring_finger,
            pinky_finger=hs.msg.pinky_finger,
            hz=hs.hz,
            timestamp=hs.timestamp
        )
        return status
    
    def get_finger_position(self) -> Optional[FingerPosition]:
        if not self._initialized or self._effector is None:
            return None
        
        fp = self._effector.get_finger_pos()
        if fp is None:
            return None
        
        position = FingerPosition(
            thumb_tip=fp.msg.thumb_tip,
            thumb_base=fp.msg.thumb_base,
            index_finger=fp.msg.index_finger,
            middle_finger=fp.msg.middle_finger,
            ring_finger=fp.msg.ring_finger,
            pinky_finger=fp.msg.pinky_finger,
            hz=fp.hz,
            timestamp=fp.timestamp
        )
        return position
    
    def get_finger_speed(self) -> Optional[FingerSpeed]:
        if not self._initialized or self._effector is None:
            return None
        
        fs = self._effector.get_finger_spd()
        if fs is None:
            return None
        
        speed = FingerSpeed(
            thumb_tip=fs.msg.thumb_tip,
            thumb_base=fs.msg.thumb_base,
            index_finger=fs.msg.index_finger,
            middle_finger=fs.msg.middle_finger,
            ring_finger=fs.msg.ring_finger,
            pinky_finger=fs.msg.pinky_finger,
            hz=fs.hz,
            timestamp=fs.timestamp
        )
        return speed
    
    def get_finger_current(self) -> Optional[FingerCurrent]:
        if not self._initialized or self._effector is None:
            return None
        
        fc = self._effector.get_finger_current()
        if fc is None:
            return None
        
        current = FingerCurrent(
            thumb_tip=fc.msg.thumb_tip,
            thumb_base=fc.msg.thumb_base,
            index_finger=fc.msg.index_finger,
            middle_finger=fc.msg.middle_finger,
            ring_finger=fc.msg.ring_finger,
            pinky_finger=fc.msg.pinky_finger,
            hz=fc.hz,
            timestamp=fc.timestamp
        )
        return current
    
    def _validate_finger_values(self, value_type: str, **fingers) -> Dict[str, Optional[int]]:
        """Validate finger values for specified control type"""
        ranges = {
            'position': (self.POSITION_MIN, self.POSITION_MAX),
            'speed': (self.SPEED_MIN, self.SPEED_MAX),
            'current': (self.CURRENT_MIN, self.CURRENT_MAX),
            'time': (self.TIME_MIN, self.TIME_MAX),
        }
        
        if value_type not in ranges:
            raise ValueError(f"Unknown value type: {value_type}")
        
        min_val, max_val = ranges[value_type]
        result = {}
        for finger_name, value in fingers.items():
            if value is None:
                result[finger_name] = None
                continue  # Skip validation for None values
            if not (min_val <= value <= max_val):
                raise ValueError(
                    f"{finger_name} {value_type} must be in range [{min_val}, {max_val}], "
                    f"current value: {value}"
                )
            result[finger_name] = value
        return result

    def _fill_with_current_position(self, **fingers) -> Dict[str, int]:
        """Fill None values with current finger positions"""
        current_pos = self.get_finger_position()
        result = {}
        for finger_name in self.FINGER_NAMES:
            value = fingers.get(finger_name)
            if value is None:
                # Use current position if not specified
                if current_pos is not None:
                    result[finger_name] = getattr(current_pos, finger_name, 0)
                else:
                    result[finger_name] = 0
            else:
                result[finger_name] = value
        return result

    def position_ctrl(
        self,
        thumb_tip: Optional[int] = None,
        thumb_base: Optional[int] = None,
        index_finger: Optional[int] = None,
        middle_finger: Optional[int] = None,
        ring_finger: Optional[int] = None,
        pinky_finger: Optional[int] = None
    ) -> bool:
        if not self._initialized or self._effector is None:
            return False
        
        # Parameter validation
        finger_values = self._validate_finger_values('position',
            thumb_tip=thumb_tip,
            thumb_base=thumb_base,
            index_finger=index_finger,
            middle_finger=middle_finger,
            ring_finger=ring_finger,
            pinky_finger=pinky_finger
        )
        
        # Fill None values with current positions
        filled = self._fill_with_current_position(**finger_values)
        
        try:
            self._effector.position_time_ctrl(mode='pos', **filled)
            self._effector.position_time_ctrl(
                mode='time',
                **dict.fromkeys(self.FINGER_NAMES, self.DEFAULT_POSITION_TIME),
            )
            return True
        except Exception as e:
            print(f"[Revo2Wrapper] Position control failed: {e}")
            return False
    
    def speed_ctrl(
        self,
        thumb_tip: Optional[int] = None,
        thumb_base: Optional[int] = None,
        index_finger: Optional[int] = None,
        middle_finger: Optional[int] = None,
        ring_finger: Optional[int] = None,
        pinky_finger: Optional[int] = None
    ) -> bool:
        if not self._initialized or self._effector is None:
            return False
        
        # Parameter validation (skips None values)
        finger_values = self._validate_finger_values('speed',
            thumb_tip=thumb_tip,
            thumb_base=thumb_base,
            index_finger=index_finger,
            middle_finger=middle_finger,
            ring_finger=ring_finger,
            pinky_finger=pinky_finger
        )
        
        filled = {
            k: v if v is not None else 0
            for k, v in finger_values.items()
        }
        
        try:
            self._effector.speed_ctrl(**filled)
            return True
        except Exception as e:
            print(f"[Revo2Wrapper] Speed control failed: {e}")
            return False
    
    def current_ctrl(
        self,
        thumb_tip: Optional[int] = None,
        thumb_base: Optional[int] = None,
        index_finger: Optional[int] = None,
        middle_finger: Optional[int] = None,
        ring_finger: Optional[int] = None,
        pinky_finger: Optional[int] = None
    ) -> bool:
        if not self._initialized or self._effector is None:
            return False
        
        # Parameter validation (skips None values)
        finger_values = self._validate_finger_values('current',
            thumb_tip=thumb_tip,
            thumb_base=thumb_base,
            index_finger=index_finger,
            middle_finger=middle_finger,
            ring_finger=ring_finger,
            pinky_finger=pinky_finger
        )
        
        filled = {
            k: v if v is not None else 0
            for k, v in finger_values.items()
        }
        
        try:
            self._effector.current_ctrl(**filled)
            return True
        except Exception as e:
            print(f"[Revo2Wrapper] Current control failed: {e}")
            return False
    
    def position_time_ctrl(
        self,
        mode: Literal['pos', 'time'] = 'pos',
        thumb_tip: Optional[int] = None,
        thumb_base: Optional[int] = None,
        index_finger: Optional[int] = None,
        middle_finger: Optional[int] = None,
        ring_finger: Optional[int] = None,
        pinky_finger: Optional[int] = None
    ) -> bool:
        if not self._initialized or self._effector is None:
            return False
        
        if mode not in ['pos', 'time']:
            raise ValueError(f"mode must be 'pos' or 'time', current value: {mode}")
        
        # Validate based on mode (skips None values)
        validate_type = 'position' if mode == 'pos' else 'time'
        finger_values = self._validate_finger_values(validate_type,
            thumb_tip=thumb_tip,
            thumb_base=thumb_base,
            index_finger=index_finger,
            middle_finger=middle_finger,
            ring_finger=ring_finger,
            pinky_finger=pinky_finger
        )
        
        # Fill None values based on mode
        if mode == 'pos':
            # For position mode, use current positions
            filled = self._fill_with_current_position(**finger_values)
        else:
            # For time mode, use 0 as default
            filled = {
                k: v if v is not None else 0
                for k, v in finger_values.items()
            }

        try:
            self._effector.position_time_ctrl(mode=mode, **filled)
            return True
        except Exception as e:
            print(f"[Revo2Wrapper] Position/time hybrid control failed: {e}")
            return False
    
    def is_hand_left(self) -> bool:
        status = self.get_status()
        if status is None:
            return False
        return status.left_or_right == self.HAND_LEFT
    
    def is_hand_right(self) -> bool:
        status = self.get_status()
        if status is None:
            return False
        return status.left_or_right == self.HAND_RIGHT
