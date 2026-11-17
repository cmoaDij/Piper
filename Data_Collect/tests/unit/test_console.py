from data_collect.common.console import CollectionConsole


def test_concise_output_keeps_lifecycle_and_diagnostics_but_hides_bulk_info():
    output = []
    console = CollectionConsole(output.append)
    lines = [
        "[INFO] [123.4] [driver]: tcp_offset: [0, 0, .195, 0, 0, 0]",
        "[INFO] [123.4] [driver]: 时序诊断(ms): {lots of values}",
        "INFO:dmrobotics:Device img FPS:119.9",
        "[INFO] [123.4] [driver]: [采集] 开始 Recording episode_0003.h5",
        "[INFO] [123.4] [driver]: [状态] 未录制 | 未就绪: 左触觉",
        "[WARN] [123.4] [driver]: IK solve failed",
        "[ERROR] [123.4] [driver]: 无法开始录制，遥操继续",
        "[WARN] Device disconnected",
        "USB device disconnected",
        "[INFO] [123.4] [driver]: Moving home at 5% speed",
        "[INFO] [123.4] [driver]: Agx_arm moved to home position successfully",
        "[INFO] [123.4] [driver]: AgxGripper enabled and verified open at home",
    ]
    console.feed(10, 1, ("\n".join(lines) + "\n").encode())
    assert len(output) == 9
    assert output[0] == "[采集] 开始 Recording episode_0003.h5"
    assert "未就绪" in output[1]
    assert any("IK solve failed" in s for s in output)
    assert any("无法开始录制" in s for s in output)
    assert any("disconnected" in s for s in output)
    assert "USB device disconnected" in output
    assert output[-1] == "[回零] 夹爪已张开，到位检查通过"


def test_chunked_utf8_interleaved_processes_and_partial_tail_are_preserved():
    output = []
    console = CollectionConsole(output.append)
    data = "[采集] 已保存 测试.h5\n".encode()
    for byte in data:
        console.feed(1, 1, bytes([byte]))
    console.feed(2, 2, b"Traceback (most recent call last):\n  File 'sdk.py', line 8\n")
    console.feed(1, 1, b"INFO:dmrobotics:normal FPS\n")
    console.feed(2, 2, b"ValueError: disconnected")
    console.flush(2)
    assert output == ["[采集] 已保存 测试.h5", "Traceback (most recent call last):",
                      "  File 'sdk.py', line 8", "ValueError: disconnected"]


def test_unstructured_native_stderr_is_visible():
    output = []
    console = CollectionConsole(output.append)
    console.feed(4, 2, b"USB device unavailable\n")
    assert output == ["USB device unavailable"]
