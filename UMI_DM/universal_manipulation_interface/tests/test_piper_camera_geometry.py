"""Check HDMI bar removal and restoration of the training pixel geometry."""
import cv2
import numpy as np

from umi.real_world.piper_camera_geometry import restore_camera_content


def test_native_hdmi_content():
    content = np.arange(90 * 120 * 3, dtype=np.uint8).reshape(90, 120, 3)
    canvas = np.zeros((90, 160, 3), dtype=np.uint8)
    canvas[:, 20:140] = content
    restored = restore_camera_content(canvas, 'hdmi_pillarboxed_4_3')
    np.testing.assert_array_equal(restored, content)


def test_capture_canvas_with_changed_pixel_aspect():
    y, x = np.mgrid[:90, :120]
    content = np.stack([x, y, x + y], axis=-1).astype(np.uint8)
    canvas = np.zeros((90, 160, 3), dtype=np.uint8)
    canvas[:, 20:140] = content
    captured = cv2.resize(canvas, (160, 120), interpolation=cv2.INTER_LINEAR)
    restored = restore_camera_content(captured, 'hdmi_pillarboxed_4_3')
    expected = cv2.resize(content, (160, 120), interpolation=cv2.INTER_LINEAR)
    assert restored.shape == (120, 160, 3)
    np.testing.assert_allclose(restored, expected, atol=1)


def test_explicit_geometry_selection():
    frame = np.zeros((96, 128, 3), dtype=np.uint8)
    assert restore_camera_content(frame) is frame
    try:
        restore_camera_content(frame, 'unknown')
    except ValueError:
        pass
    else:
        raise AssertionError('Unknown image geometry must be reported')


if __name__ == '__main__':
    test_native_hdmi_content()
    test_capture_canvas_with_changed_pixel_aspect()
    test_explicit_geometry_selection()
    print('Piper camera HDMI geometry checks passed')
