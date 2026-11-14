import struct

from scripts.piper_can_diagnostics import decode_frame


def test():
    status = decode_frame(0x2A1, bytes.fromhex('0001010000000000'))
    assert status['control_mode'] == 0 and status['arm_status'] == 1
    assert status['motion_mode'] == 1 and status['error_code'] == 0
    assert decode_frame(0x2A5, struct.pack('>ii', -2417, 2498)) == [-2.417, 2.498]
    driver = decode_frame(0x263, bytes.fromhex('00e600190a400000'))
    assert driver['voltage_v'] == 23.0 and driver['enabled']
    assert not driver['stall'] and not driver['collision']
    grip = decode_frame(0x2A8, struct.pack('>ihBB', 67000, 500, 0x40, 0))
    assert grip['width_mm'] == 67.0 and grip['force_feedback_n'] == .5
    assert grip['enabled'] and grip['width_mode']
    try:
        decode_frame(0x2A1, b'bad')
    except ValueError:
        pass
    else:
        raise AssertionError('Truncated frame must be rejected')
    print('Piper CAN diagnostic decoding passed')


if __name__ == '__main__':
    test()
