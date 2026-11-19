"""
Undistortion lands frames on the camera model every stage projects with.

The checks: no distortion means no change; a point the lens moved is moved
back to where the pinhole model puts it; the shipped calibration and fov_h
leave no black border; and a missing or disabled section turns it off.
"""

import math
import pathlib
import sys

import cv2
import numpy as np
import pytest

PKG = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))

from perception_suite import undistort  # noqa: E402

SIZE = (640, 413)
CX, CY = SIZE[0] / 2.0, SIZE[1] / 2.0
HATCI = undistort.Calibration(
    native_size=(2880, 1860),
    camera_matrix=(1514.45894838158, 1522.09931279667, 1419.01898306367, 912.475154507531),
    dist_coeffs=(-0.342147628921505, 0.0994968602340448,
                 0.00449211504689297, 0.000303143869552083),
)


def _focal(fov_deg):
    return (SIZE[0] / 2.0) / math.tan(math.radians(fov_deg) / 2.0)


def test_no_distortion_is_identity():
    # A centred pinhole camera with zero coefficients, mapped onto itself.
    f = _focal(90.0)
    sx, sy = SIZE[0] / 2880, SIZE[1] / 1860
    calibration = undistort.Calibration(
        (2880, 1860), (f / sx, f / sy, (CX + 0.5) / sx - 0.5, (CY + 0.5) / sy - 0.5),
        (0.0, 0.0, 0.0, 0.0))
    map_x, map_y = undistort._float_maps(calibration, SIZE, f, CX, CY)
    u, v = np.meshgrid(np.arange(SIZE[0]), np.arange(SIZE[1]))
    assert np.abs(map_x - u).max() < 1e-3
    assert np.abs(map_y - v).max() < 1e-3


@pytest.mark.parametrize('ray', [(0.0, 0.0), (0.5, 0.2), (-0.8, 0.3), (0.9, -0.4)])
def test_distorted_point_maps_back_to_pinhole(ray):
    # Where the lens images a ray, the map must sample for the pixel where the
    # pinhole model (focal f, centre CX, CY) puts that same ray.
    f = _focal(95.4)
    k = undistort.scaled_matrix(HATCI, SIZE)
    obj = np.array([[[ray[0], ray[1], 1.0]]])
    lens_px, _ = cv2.projectPoints(obj, np.zeros(3), np.zeros(3), k,
                                   np.asarray(HATCI.dist_coeffs))
    map_x, map_y = undistort._float_maps(HATCI, SIZE, f, CX, CY)
    u, v = CX + f * ray[0], CY + f * ray[1]
    # Bilinear lookup of the map at the (sub-pixel) pinhole position.
    sample = cv2.remap(np.dstack([map_x, map_y]), np.float32([[u]]), np.float32([[v]]),
                       cv2.INTER_LINEAR)[0, 0]
    assert np.allclose(sample, lens_px.reshape(2), atol=0.05)


def test_shipped_fov_has_no_black_border():
    widest = undistort.widest_fov_deg(HATCI, SIZE, CX, CY)
    assert 95.4 <= widest < 96.0
    _, _, outside = undistort.build_maps(HATCI, SIZE, _focal(95.4), CX, CY)
    assert outside == 0.0
    _, _, outside = undistort.build_maps(HATCI, SIZE, _focal(100.0), CX, CY)
    assert outside > 0.0


def test_config_section(tmp_path):
    cfg = tmp_path / 'c.yaml'
    cfg.write_text('beamng: {}\n')
    assert undistort.load_calibration(cfg) is None
    cfg.write_text('undistortion:\n  enabled: false\n')
    assert undistort.load_calibration(cfg) is None
    cfg.write_text(
        'undistortion:\n  enabled: true\n  native_size: [2880, 1860]\n'
        '  camera_matrix: [1514.4, 1522.0, 1419.0, 912.4]\n'
        '  dist_coeffs: [-0.34, 0.099, 0.0045, 0.0003]\n')
    calibration = undistort.load_calibration(cfg)
    assert calibration.native_size == (2880, 1860)
    assert calibration.dist_coeffs == (-0.34, 0.099, 0.0045, 0.0003)
    cfg.write_text(
        'undistortion:\n  enabled: true\n  native_size: [2880, 1860]\n'
        '  camera_matrix: [1514.4, 1522.0, 1419.0]\n  dist_coeffs: [0, 0, 0, 0]\n')
    with pytest.raises(ValueError):
        undistort.load_calibration(cfg)
