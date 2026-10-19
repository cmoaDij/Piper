# Copyright 2026 Piper Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pytest

from agx_arm_ctrl.teach_mode import (
    TeachCommand,
    send_teach_command,
    teach_status_name,
)


class FakeArm:
    def __init__(self):
        self.messages = []

    def _send_msg(self, message):
        self.messages.append(message)


@pytest.mark.parametrize(
    "command,value",
    [
        (TeachCommand.START_RECORD, 0x01),
        (TeachCommand.STOP_RECORD, 0x02),
        (TeachCommand.REPLAY, 0x03),
        (TeachCommand.PAUSE, 0x04),
        (TeachCommand.RESUME, 0x05),
        (TeachCommand.TERMINATE, 0x06),
        (TeachCommand.MOVE_TO_START, 0x07),
    ],
)
def test_send_teach_command_encodes_protocol_value(command, value):
    arm = FakeArm()

    send_teach_command(arm, command)

    assert len(arm.messages) == 1
    assert arm.messages[0].emergency_stop == 0
    assert arm.messages[0].track_ctrl == 0
    assert arm.messages[0].grag_teach_ctrl == value


def test_send_teach_command_rejects_invalid_command():
    with pytest.raises(TypeError):
        send_teach_command(FakeArm(), 3)


def test_send_teach_command_requires_existing_can_owner():
    with pytest.raises(RuntimeError):
        send_teach_command(object(), TeachCommand.REPLAY)


def test_teach_status_name_preserves_unknown_value():
    assert teach_status_name(3) == "replaying"
    assert teach_status_name(0x80) == "unknown_0x80"
