"""
Lens undistortion for working-size frames, from HATCI's IMX490 calibration.

The calibration (Brown-Conrady, at the sensor's native 2880x1860) is applied
after the preprocess resize, as one remap per frame. The output is a plain
pinhole image with exactly the camera model offroad_autonomy builds from the
config: focal length from beamng.camera.sensor.fov_h, centre at the frame
centre, square pixels. Every stage that turns pixels into ground metres
(planner, BEV grid, controller, dashboard) therefore stays correct with no
change to the library.

Only the camera matrix is scaled to the working size. The distortion
coefficients act on normalised coordinates, so they do not depend on size.
"""

from dataclasses import dataclass
import math

import cv2
import numpy as np
import yaml


@dataclass(frozen=True)
class Calibration:
    """Intrinsics at the native sensor size, and Brown-Conrady coefficients."""

    native_size: tuple
    camera_matrix: tuple
    dist_coeffs: tuple


def load_calibration(config_path):
    """
    Return the config's enabled undistortion section, or None.

    The section is top-level and read only here; offroad_autonomy's loader
    ignores keys it does not know, so adding it changes nothing else.
    """
    with open(config_path) as fh:
        raw = yaml.safe_load(fh) or {}
    section = raw.get('undistortion')
    if not section or not bool(section.get('enabled', False)):
        return None
    size = tuple(int(v) for v in section['native_size'])
    matrix = tuple(float(v) for v in section['camera_matrix'])
    coeffs = tuple(float(v) for v in section['dist_coeffs'])
    if len(size) != 2 or min(size) <= 0:
        raise ValueError('undistortion.native_size must be [width, height]')
    if len(matrix) != 4 or not all(map(math.isfinite, matrix)) or min(matrix[:2]) <= 0:
        raise ValueError('undistortion.camera_matrix must be finite [fx, fy, cx, cy]')
    if len(coeffs) not in (4, 5, 8) or not all(map(math.isfinite, coeffs)):
        raise ValueError('undistortion.dist_coeffs must be [k1, k2, p1, p2] or longer')
    return Calibration(size, matrix, coeffs)


def scaled_matrix(calibration, size):
    """Return the 3x3 camera matrix for frames resized to size (w, h)."""
    sx = size[0] / calibration.native_size[0]
    sy = size[1] / calibration.native_size[1]
    fx, fy, cx, cy = calibration.camera_matrix
    # OpenCV puts pixel centres on integers, so the centre scales about -0.5.
    return np.array([
        [fx * sx, 0.0, (cx + 0.5) * sx - 0.5],
        [0.0, fy * sy, (cy + 0.5) * sy - 0.5],
        [0.0, 0.0, 1.0],
    ])


def _float_maps(calibration, size, focal_px, cx, cy):
    target = np.array([[focal_px, 0.0, cx], [0.0, focal_px, cy], [0.0, 0.0, 1.0]])
    return cv2.initUndistortRectifyMap(
        scaled_matrix(calibration, size), np.asarray(calibration.dist_coeffs, dtype=np.float64),
        None, target, tuple(size), cv2.CV_32FC1)


def outside_fraction(map_x, map_y, size):
    """Share of output pixels that sample outside the source frame (they turn black)."""
    w, h = size
    outside = (map_x < 0) | (map_y < 0) | (map_x > w - 1) | (map_y > h - 1)
    return float(outside.mean())


def build_maps(calibration, size, focal_px, cx, cy):
    """
    Return (map1, map2, outside) for cv2.remap onto the given pinhole model.

    focal_px, cx and cy must be the CameraModel's own values, so the
    undistorted frame and every ground projection agree.
    """
    map_x, map_y = _float_maps(calibration, size, focal_px, cx, cy)
    outside = outside_fraction(map_x, map_y, size)
    map1, map2 = cv2.convertMaps(map_x, map_y, cv2.CV_16SC2)
    return map1, map2, outside


def widest_fov_deg(calibration, size, cx, cy):
    """Widest horizontal field of view whose output has no black border."""
    lo, hi = 0.05 * size[0], 10.0 * size[0]
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        if outside_fraction(*_float_maps(calibration, size, mid, cx, cy), size) > 0.0:
            lo = mid
        else:
            hi = mid
    return math.degrees(2.0 * math.atan((size[0] / 2.0) / hi))
