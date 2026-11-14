"""Restore GoPro HDMI content before applying the training image transform."""
import cv2
import numpy as np


def restore_camera_content(frame, mode='native_4_3'):
    """Return the 4:3 sensor image from the configured capture canvas.

    ``hdmi_pillarboxed_4_3`` describes a 16:9 HDMI canvas containing centered
    4:3 content. Some capture modes stretch that entire canvas into 1280x960.
    The content still occupies its middle 75% horizontally. Removing those
    bars and restoring a 4:3 pixel aspect ratio recovers the training geometry.
    """
    if mode == 'native_4_3':
        return frame
    if mode != 'hdmi_pillarboxed_4_3':
        raise ValueError(f'Unknown camera image geometry: {mode!r}')
    height, width = frame.shape[:2]
    content_width = round(width * 3 / 4)
    left = (width - content_width) // 2
    content = frame[:, left:left + content_width]
    output_width = round(height * 4 / 3)
    if content_width == output_width:
        return np.ascontiguousarray(content)
    return cv2.resize(content, (output_width, height), interpolation=cv2.INTER_LINEAR)


def preprocess(frame_bgr, mask, *, camera_image_geometry='native_4_3'):
    """Apply deployment's deterministic geometry, tag removal and training mask."""
    from umi.common.cv_util import get_image_transform, inpaint_tag
    frame_bgr = restore_camera_content(frame_bgr, camera_image_geometry)
    h, w = frame_bgr.shape[:2]
    if abs(w / h - 4 / 3) > .001:
        raise ValueError(f'Expected 4:3 live frame for provisional mapping, received {w}x{h}')
    image = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    detected = []
    if ids is not None:
        for marker, marker_id in zip(corners, ids.flatten()):
            if int(marker_id) in (0, 1):
                image = inpaint_tag(image, marker.reshape(4, 2))
                detected.append(int(marker_id))
    ref_w, ref_h = mask['reference_resolution']
    poly = np.asarray(mask['polygon'], dtype=float) * [w / ref_w, h / ref_h]
    cv2.fillPoly(image, [np.rint(poly).astype(np.int32)], (0, 0, 0))
    return get_image_transform((w, h), (224, 224))(image), detected
