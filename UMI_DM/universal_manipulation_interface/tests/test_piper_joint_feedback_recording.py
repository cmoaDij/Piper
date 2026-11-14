import struct
from scripts.record_piper_joint_feedback import decode_joint_frame


def test():
    frame = struct.pack('=IB3x8s', 0x2a6, 8, struct.pack('>ii', -1234, 5678))
    assert decode_joint_frame(frame) == (0x2a6, -1.234, 5.678)
    assert decode_joint_frame(struct.pack('=IB3x8s', 0x2a4, 8, bytes(8))) is None
    assert decode_joint_frame(b'broken') is None
    print('Passive Piper feedback decoding passed')


if __name__ == '__main__':
    test()
