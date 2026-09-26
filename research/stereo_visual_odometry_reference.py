#!/usr/bin/env python3
"""Offline stereo visual-odometry reference lab.

Milestone 6A scope:
- Synthetic rectified stereo only.
- OpenCV triangulation / solvePnP / solvePnPRansac education only.
- No ROS topics, TF publishers, Isaac ROS, cuVSLAM, Nav2, or camera drivers.

Frame convention used here:
- W is the reference/world frame and is exactly the time-k robot body frame B0.
- B is the robot body / base_link frame: +X forward, +Y left, +Z up.
- C is the left camera optical frame: +X right, +Y down, +Z forward.

The fixed body-to-camera optical rotation is:

    p_C = R_CB @ p_B + t_CB

    R_CB = [[ 0, -1,  0],
            [ 0,  0, -1],
            [ 1,  0,  0]]

Robot yaw changes T_W_B and therefore T_W_C. It does not change fixed T_C_B.
Stereo triangulation at time k produces points in C0; this lab explicitly
converts P_C0 to P_W through T_W_C0 before passing objectPoints to solvePnP.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import cv2
import numpy as np


RNG_SEED = 42


@dataclass(frozen=True)
class StereoCalibration:
    """Synthetic placeholder calibration for ideal rectified stereo.

    Unit convention:
    - 3D positions and stereo baseline are meters.
    - fx, fy, cx, cy, skew, u, v, and disparity are pixels.
    - fx = fy and skew = 0 are synthetic simplifications, not general
      camera requirements.
    - No lens distortion is modeled in this reference lab.
    """

    image_width: int = 1280
    image_height: int = 720
    fx: float = 700.0
    fy: float = 700.0
    skew: float = 0.0
    cx: float = 640.0
    cy: float = 360.0
    baseline_m: float = 0.12

    @property
    def K(self) -> np.ndarray:
        return np.array(
            [
                [self.fx, self.skew, self.cx],
                [0.0, self.fy, self.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )


@dataclass(frozen=True)
class Transform:
    """Rigid transform T_A_B: maps coordinates from frame B into frame A."""

    R: np.ndarray
    t: np.ndarray

    def apply(self, points: np.ndarray) -> np.ndarray:
        return (self.R @ points.T).T + self.t.reshape(1, 3)

    def inverse(self) -> "Transform":
        R_inv = self.R.T
        return Transform(R_inv, -R_inv @ self.t)

    def compose(self, other: "Transform") -> "Transform":
        """Return self * other, so T_A_C = T_A_B * T_B_C."""
        return Transform(self.R @ other.R, self.R @ other.t + self.t)

    def as_matrix(self) -> np.ndarray:
        out = np.eye(4, dtype=np.float64)
        out[:3, :3] = self.R
        out[:3, 3] = self.t
        return out


@dataclass
class PoseEstimateResult:
    name: str
    ok: bool
    message: str
    rvec: np.ndarray | None = None
    tvec: np.ndarray | None = None
    inliers: np.ndarray | None = None
    refined: bool = False


def deg(rad: float) -> float:
    return math.degrees(rad)


def rad(deg_value: float) -> float:
    return math.radians(deg_value)


def rotation_z(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def rotation_angle_deg(R_est: np.ndarray, R_true: np.ndarray) -> float:
    R_err = R_est @ R_true.T
    cos_angle = (np.trace(R_err) - 1.0) / 2.0
    return deg(math.acos(float(np.clip(cos_angle, -1.0, 1.0))))


def transform_to_rvec_tvec(T_camera_world: Transform) -> tuple[np.ndarray, np.ndarray]:
    rvec, _ = cv2.Rodrigues(T_camera_world.R)
    return rvec.reshape(3, 1), T_camera_world.t.reshape(3, 1)


def rvec_tvec_to_transform(rvec: np.ndarray, tvec: np.ndarray) -> Transform:
    R, _ = cv2.Rodrigues(rvec)
    return Transform(R.astype(np.float64), tvec.reshape(3).astype(np.float64))


def body_camera_extrinsics(camera_position_body: np.ndarray | None = None) -> tuple[Transform, Transform]:
    """Return fixed T_C_B and T_B_C for a level forward-facing camera.

    camera_position_body is the camera origin expressed in body coordinates.
    T_B_C maps camera coordinates into body coordinates:
        p_B = R_BC @ p_C + t_BC
    T_C_B is its inverse:
        p_C = R_CB @ p_B + t_CB
    """
    if camera_position_body is None:
        camera_position_body = np.zeros(3, dtype=np.float64)
    else:
        camera_position_body = np.asarray(camera_position_body, dtype=np.float64)
    R_CB = np.array(
        [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]],
        dtype=np.float64,
    )
    R_BC = R_CB.T
    T_B_C = Transform(R_BC, camera_position_body)
    T_C_B = T_B_C.inverse()
    return T_C_B, T_B_C


def ground_truth_poses(
    delta_x_body: float = 0.10,
    delta_yaw_body: float = rad(5.0),
    camera_position_body: np.ndarray | None = None,
) -> dict[str, Transform]:
    """Build the synthetic ground-truth poses.

    W is B0, so T_W_B0 is identity. Time-k stereo triangulation starts in C0
    and is explicitly converted to W through T_W_C0.
    """
    T_C_B, T_B_C = body_camera_extrinsics(camera_position_body)
    T_W_B0 = Transform(np.eye(3), np.zeros(3))
    T_W_C0 = T_W_B0.compose(T_B_C)

    T_B0_B1 = Transform(
        rotation_z(delta_yaw_body),
        np.array([delta_x_body, 0.0, 0.0], dtype=np.float64),
    )
    T_W_B1 = T_W_B0.compose(T_B0_B1)
    T_W_C1 = T_W_B1.compose(T_B_C)
    T_C1_W = T_W_C1.inverse()
    return {
        "T_C_B": T_C_B,
        "T_B_C": T_B_C,
        "T_W_C0": T_W_C0,
        "T_W_B0": T_W_B0,
        "T_B0_B1": T_B0_B1,
        "T_W_B1": T_W_B1,
        "T_W_C1": T_W_C1,
        "T_C1_W": T_C1_W,
    }


def project_left(points_c: np.ndarray, calib: StereoCalibration) -> tuple[np.ndarray, np.ndarray]:
    z = points_c[:, 2]
    valid = z > 1e-9
    pixels = np.full((len(points_c), 2), np.nan, dtype=np.float64)
    pixels[valid, 0] = (
        calib.fx * points_c[valid, 0] / z[valid]
        + calib.skew * points_c[valid, 1] / z[valid]
        + calib.cx
    )
    pixels[valid, 1] = calib.fy * points_c[valid, 1] / z[valid] + calib.cy
    return pixels, valid


def project_homogeneous(points_w: np.ndarray, T_camera_world: Transform, calib: StereoCalibration) -> tuple[np.ndarray, np.ndarray]:
    """Project world points with x_h = K [R_C_W | t_C_W] X_W_h.

    Homogeneous normalization is mandatory:
        u = x_h[0] / x_h[2]
        v = x_h[1] / x_h[2]
    """
    points_h = np.column_stack([points_w, np.ones(len(points_w), dtype=np.float64)])
    extrinsic = np.hstack([T_camera_world.R, T_camera_world.t.reshape(3, 1)])
    projected_h = (calib.K @ extrinsic @ points_h.T).T
    valid = np.abs(projected_h[:, 2]) > 1e-12
    pixels = np.full((len(points_w), 2), np.nan, dtype=np.float64)
    pixels[valid, 0] = projected_h[valid, 0] / projected_h[valid, 2]
    pixels[valid, 1] = projected_h[valid, 1] / projected_h[valid, 2]
    return pixels, valid


def project_right_from_left(points_c_left: np.ndarray, calib: StereoCalibration) -> tuple[np.ndarray, np.ndarray]:
    points_right = points_c_left.copy()
    points_right[:, 0] -= calib.baseline_m
    return project_left(points_right, calib)


def inside_image(pixels: np.ndarray, calib: StereoCalibration) -> np.ndarray:
    return (
        np.isfinite(pixels[:, 0])
        & np.isfinite(pixels[:, 1])
        & (pixels[:, 0] >= 0.0)
        & (pixels[:, 0] < calib.image_width)
        & (pixels[:, 1] >= 0.0)
        & (pixels[:, 1] < calib.image_height)
    )


def generate_landmarks_c0(count: int, calib: StereoCalibration) -> tuple[np.ndarray, dict[str, int]]:
    """Generate deterministic landmarks in the time-k left camera frame C0."""
    rng = np.random.default_rng(RNG_SEED)
    candidates = np.column_stack(
        [
            rng.uniform(-2.0, 2.0, count * 4),
            rng.uniform(-1.0, 1.0, count * 4),
            rng.uniform(2.0, 8.0, count * 4),
        ]
    )
    left_px, left_depth_ok = project_left(candidates, calib)
    right_px, right_depth_ok = project_right_from_left(candidates, calib)
    disparity = left_px[:, 0] - right_px[:, 0]
    valid = (
        left_depth_ok
        & right_depth_ok
        & inside_image(left_px, calib)
        & inside_image(right_px, calib)
        & (disparity > 1e-6)
    )
    selected = candidates[valid][:count]
    diagnostics = {
        "candidate_count": len(candidates),
        "kept_count": len(selected),
        "outside_or_invalid_count": int(len(candidates) - np.count_nonzero(valid)),
    }
    if len(selected) < count:
        raise RuntimeError(f"not enough valid landmarks: requested={count}, kept={len(selected)}")
    return selected, diagnostics


def add_image_noise(points: np.ndarray, sigma_px: float, rng: np.random.Generator) -> np.ndarray:
    if sigma_px <= 0.0:
        return points.copy()
    return points + rng.normal(0.0, sigma_px, points.shape)


def triangulate_disparity(left_px: np.ndarray, right_px: np.ndarray, calib: StereoCalibration) -> tuple[np.ndarray, np.ndarray]:
    disparity = left_px[:, 0] - right_px[:, 0]
    valid = np.isfinite(disparity) & (disparity > 1e-9)
    points = np.full((len(left_px), 3), np.nan, dtype=np.float64)
    points[valid, 2] = calib.fx * calib.baseline_m / disparity[valid]
    points[valid, 0] = (left_px[valid, 0] - calib.cx) * points[valid, 2] / calib.fx
    points[valid, 1] = (left_px[valid, 1] - calib.cy) * points[valid, 2] / calib.fy
    return points, valid


def triangulate_opencv(left_px: np.ndarray, right_px: np.ndarray, calib: StereoCalibration) -> tuple[np.ndarray, np.ndarray]:
    P_left = calib.K @ np.hstack([np.eye(3), np.zeros((3, 1))])
    P_right = calib.K @ np.hstack([np.eye(3), np.array([[-calib.baseline_m], [0.0], [0.0]])])
    points_4d = cv2.triangulatePoints(P_left, P_right, left_px.T, right_px.T)
    valid = np.abs(points_4d[3, :]) > 1e-12
    points = np.full((len(left_px), 3), np.nan, dtype=np.float64)
    points[valid] = (points_4d[:3, valid] / points_4d[3, valid]).T
    return points, valid


def rmse(values: np.ndarray) -> float:
    values = np.asarray(values, dtype=np.float64)
    if values.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(values * values)))


def triangulation_metrics(name: str, points: np.ndarray, truth: np.ndarray, valid: np.ndarray) -> str:
    valid_points = valid & np.all(np.isfinite(points), axis=1)
    if not np.any(valid_points):
        return f"{name}: no valid points"
    error = points[valid_points] - truth[valid_points]
    pos_rmse = rmse(np.linalg.norm(error, axis=1))
    depth_rmse = rmse(error[:, 2])
    invalid_count = int(len(points) - np.count_nonzero(valid_points))
    return f"{name}: 3D RMSE={pos_rmse:.9f} m, depth RMSE={depth_rmse:.9f} m, invalid={invalid_count}"


def transform_points_with_valid(points: np.ndarray, transform: Transform) -> np.ndarray:
    out = np.full_like(points, np.nan, dtype=np.float64)
    valid = np.all(np.isfinite(points), axis=1)
    if np.any(valid):
        out[valid] = transform.apply(points[valid])
    return out


def correspondence_spread(pixels: np.ndarray) -> tuple[float, float]:
    if len(pixels) == 0:
        return 0.0, 0.0
    return float(np.ptp(pixels[:, 0])), float(np.ptp(pixels[:, 1]))


def collinearity_ratio(points: np.ndarray) -> float:
    centered = points - np.mean(points, axis=0, keepdims=True)
    _, s, _ = np.linalg.svd(centered, full_matrices=False)
    if s[0] <= 1e-12:
        return float("inf")
    return float(s[-1] / s[0])


def corrupt_observations(
    image_points: np.ndarray,
    ratio: float,
    calib: StereoCalibration,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray]:
    corrupted = image_points.copy()
    count = int(round(len(image_points) * ratio))
    indices = np.arange(len(image_points))
    if count == 0:
        return corrupted, np.array([], dtype=np.int32)
    outlier_indices = rng.choice(indices, size=count, replace=False)
    corrupted[outlier_indices, 0] = rng.uniform(0.0, calib.image_width, count)
    corrupted[outlier_indices, 1] = rng.uniform(0.0, calib.image_height, count)
    return corrupted, np.sort(outlier_indices.astype(np.int32))


def solve_plain_pnp(object_points: np.ndarray, image_points: np.ndarray, calib: StereoCalibration) -> PoseEstimateResult:
    if len(object_points) < 4:
        return PoseEstimateResult("solvePnP", False, "too few correspondences")
    try:
        ok, rvec, tvec = cv2.solvePnP(
            object_points.astype(np.float64),
            image_points.astype(np.float64),
            calib.K,
            None,
            flags=cv2.SOLVEPNP_EPNP,
        )
    except cv2.error as exc:
        return PoseEstimateResult("solvePnP", False, f"OpenCV error: {exc}")
    return PoseEstimateResult("solvePnP", bool(ok), "ok" if ok else "failed", rvec, tvec)


def solve_ransac_pnp(object_points: np.ndarray, image_points: np.ndarray, calib: StereoCalibration) -> PoseEstimateResult:
    if len(object_points) < 4:
        return PoseEstimateResult("solvePnPRansac", False, "too few correspondences")
    cv2.setRNGSeed(RNG_SEED)
    try:
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            object_points.astype(np.float64),
            image_points.astype(np.float64),
            calib.K,
            None,
            iterationsCount=100,
            reprojectionError=3.0,
            confidence=0.99,
            flags=cv2.SOLVEPNP_EPNP,
        )
    except cv2.error as exc:
        return PoseEstimateResult("solvePnPRansac", False, f"OpenCV error: {exc}")
    if inliers is None:
        inliers = np.empty((0, 1), dtype=np.int32)
    return PoseEstimateResult("solvePnPRansac", bool(ok), "ok" if ok else "failed", rvec, tvec, inliers)


def refine_lm(
    result: PoseEstimateResult,
    object_points: np.ndarray,
    image_points: np.ndarray,
    calib: StereoCalibration,
) -> PoseEstimateResult:
    if not result.ok or result.rvec is None or result.tvec is None:
        return PoseEstimateResult("solvePnPRefineLM", False, "no valid initial estimate")
    if not hasattr(cv2, "solvePnPRefineLM"):
        return PoseEstimateResult("solvePnPRefineLM", False, "not available")
    indices = result.inliers.reshape(-1) if result.inliers is not None and len(result.inliers) > 0 else np.arange(len(object_points))
    if len(indices) < 4:
        return PoseEstimateResult("solvePnPRefineLM", False, "too few inliers")
    rvec = result.rvec.copy()
    tvec = result.tvec.copy()
    try:
        cv2.solvePnPRefineLM(
            object_points[indices].astype(np.float64),
            image_points[indices].astype(np.float64),
            calib.K,
            None,
            rvec,
            tvec,
        )
    except cv2.error as exc:
        return PoseEstimateResult("solvePnPRefineLM", False, f"OpenCV error: {exc}")
    return PoseEstimateResult("solvePnPRefineLM", True, "ok", rvec, tvec, result.inliers, refined=True)


def reprojection_errors(
    rvec: np.ndarray,
    tvec: np.ndarray,
    object_points: np.ndarray,
    observed_points: np.ndarray,
) -> np.ndarray:
    projected, _ = cv2.projectPoints(object_points.astype(np.float64), rvec, tvec, StereoCalibration().K, None)
    projected = projected.reshape(-1, 2)
    return np.linalg.norm(projected - observed_points, axis=1)


def pose_metrics(result: PoseEstimateResult, poses: dict[str, Transform], object_points: np.ndarray, image_points: np.ndarray) -> str:
    if not result.ok or result.rvec is None or result.tvec is None:
        return f"{result.name}: failed ({result.message})"

    T_C1_W_est = rvec_tvec_to_transform(result.rvec, result.tvec)
    T_W_C1_est = T_C1_W_est.inverse()
    T_C_B = poses["T_C_B"]
    T_W_B1_est = T_W_C1_est.compose(T_C_B)

    T_W_C1_true = poses["T_W_C1"]
    T_W_B1_true = poses["T_W_B1"]
    T_W_B0 = poses["T_W_B0"]

    translation_error = float(np.linalg.norm(T_W_C1_est.t - T_W_C1_true.t))
    rotation_error = rotation_angle_deg(T_W_C1_est.R, T_W_C1_true.R)

    T_B0_B1_est = T_W_B0.inverse().compose(T_W_B1_est)
    T_B0_B1_true = poses["T_B0_B1"]
    yaw_est = math.atan2(T_B0_B1_est.R[1, 0], T_B0_B1_est.R[0, 0])
    yaw_true = math.atan2(T_B0_B1_true.R[1, 0], T_B0_B1_true.R[0, 0])
    yaw_error = deg(abs(math.atan2(math.sin(yaw_est - yaw_true), math.cos(yaw_est - yaw_true))))
    body_xy_error = T_B0_B1_est.t[:2] - T_B0_B1_true.t[:2]

    all_errors = reprojection_errors(result.rvec, result.tvec, object_points, image_points)
    all_summary = (
        f"all reproj mean={np.mean(all_errors):.4f}px, "
        f"rmse={rmse(all_errors):.4f}px, max={np.max(all_errors):.4f}px"
    )
    if result.inliers is not None and len(result.inliers) > 0:
        inlier_idx = result.inliers.reshape(-1)
        inlier_errors = all_errors[inlier_idx]
        inlier_summary = (
            f"inliers={len(inlier_idx)}/{len(object_points)} "
            f"({len(inlier_idx)/len(object_points):.1%}), "
            f"inlier reproj mean={np.mean(inlier_errors):.4f}px, "
            f"rmse={rmse(inlier_errors):.4f}px, max={np.max(inlier_errors):.4f}px"
        )
    else:
        inlier_summary = "inliers=n/a"

    return (
        f"{result.name}: translation error={translation_error:.9f} m, "
        f"rotation error={rotation_error:.9f} deg, AMR yaw error={yaw_error:.9f} deg, "
        f"body dx error={body_xy_error[0]:.9f} m, body dy error={body_xy_error[1]:.9f} m, "
        f"{all_summary}, {inlier_summary}"
    )


def run_experiment(
    name: str,
    pixel_noise_sigma: float,
    outlier_ratio: float,
    landmark_count: int = 120,
    camera_position_body: np.ndarray | None = None,
) -> list[str]:
    calib = StereoCalibration()
    poses = ground_truth_poses(camera_position_body=camera_position_body)
    rng = np.random.default_rng(RNG_SEED + int(pixel_noise_sigma * 1000) + int(outlier_ratio * 1000))

    landmarks_c0, landmark_diag = generate_landmarks_c0(landmark_count, calib)
    landmarks_w = poses["T_W_C0"].apply(landmarks_c0)
    left0, depth_left0 = project_left(landmarks_c0, calib)
    right0, depth_right0 = project_right_from_left(landmarks_c0, calib)

    left0_noisy = add_image_noise(left0, pixel_noise_sigma, rng)
    right0_noisy = add_image_noise(right0, pixel_noise_sigma, rng)
    tri_disp, valid_disp = triangulate_disparity(left0_noisy, right0_noisy, calib)
    tri_cv, valid_cv = triangulate_opencv(left0_noisy, right0_noisy, calib)
    tri_disp_w = transform_points_with_valid(tri_disp, poses["T_W_C0"])

    T_C1_W = poses["T_C1_W"]
    landmarks_c1 = T_C1_W.apply(landmarks_w)
    current_left, current_depth_ok = project_left(landmarks_c1, calib)
    current_visible = current_depth_ok & inside_image(current_left, calib)
    current_left_noisy = add_image_noise(current_left, pixel_noise_sigma, rng)

    valid = (
        valid_disp
        & valid_cv
        & depth_left0
        & depth_right0
        & current_visible
        & np.all(np.isfinite(tri_disp), axis=1)
    )
    object_points = tri_disp_w[valid].astype(np.float64)
    image_points = current_left_noisy[valid].astype(np.float64)
    image_points, outlier_indices = corrupt_observations(image_points, outlier_ratio, calib, rng)

    plain = solve_plain_pnp(object_points, image_points, calib)
    ransac = solve_ransac_pnp(object_points, image_points, calib)
    refined = refine_lm(ransac, object_points, image_points, calib)

    spread_u, spread_v = correspondence_spread(image_points)
    col_ratio = collinearity_ratio(object_points)

    lines = [
        f"\n{name}",
        "-" * len(name),
        f"landmark candidates={landmark_diag['candidate_count']}, selected={landmark_diag['kept_count']}, usable correspondences={len(object_points)}",
        f"pixel_noise_sigma={pixel_noise_sigma:.3f}px, outlier_ratio={outlier_ratio:.0%}, deterministic outliers={len(outlier_indices)}",
        f"image spread: u={spread_u:.1f}px, v={spread_v:.1f}px; collinearity smallest/largest singular ratio={col_ratio:.6f}",
        triangulation_metrics("disparity triangulation in C0", tri_disp, landmarks_c0, valid_disp),
        triangulation_metrics("cv2.triangulatePoints in C0", tri_cv, landmarks_c0, valid_cv),
        pose_metrics(plain, poses, object_points, image_points),
        pose_metrics(ransac, poses, object_points, image_points),
    ]
    if refined.ok:
        lines.append(pose_metrics(refined, poses, object_points, image_points))
    else:
        lines.append(f"solvePnPRefineLM: skipped ({refined.message})")
    return lines


def extrinsic_sanity_lines() -> list[str]:
    T_C_B, _ = body_camera_extrinsics()
    tests = [
        ("R_CB @ body +X", np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])),
        ("R_CB @ body +Y", np.array([0.0, 1.0, 0.0]), np.array([-1.0, 0.0, 0.0])),
        ("R_CB @ body +Z", np.array([0.0, 0.0, 1.0]), np.array([0.0, -1.0, 0.0])),
    ]
    lines = ["Fixed extrinsic sanity tests"]
    for label, vector, expected in tests:
        actual = T_C_B.R @ vector
        lines.append(f"{label} = {actual.tolist()} expected {expected.tolist()} error={np.linalg.norm(actual - expected):.3e}")
    yaw_axis_camera = T_C_B.R @ np.array([0.0, 0.0, 1.0])
    lines.append(f"+Z_body yaw axis expressed in camera optical basis = {yaw_axis_camera.tolist()}")
    return lines


def transform_round_trip_lines() -> list[str]:
    poses = ground_truth_poses()
    T_W_C1 = poses["T_W_C1"]
    T_C1_W = T_W_C1.inverse()
    rng = np.random.default_rng(RNG_SEED)
    points_w = rng.uniform([-1.0, -0.5, 2.0], [1.0, 0.5, 5.0], size=(10, 3))
    points_c = T_C1_W.apply(points_w)
    recovered_w = T_W_C1.apply(points_c)
    point_error = float(np.max(np.linalg.norm(points_w - recovered_w, axis=1)))
    identity_error = float(np.max(np.abs(T_W_C1.compose(T_C1_W).as_matrix() - np.eye(4))))
    return [
        "\nTransform round-trip tests",
        f"max ||X_W - recovered X_W|| = {point_error:.3e}",
        f"max |T_W_C1 * T_C1_W - I| = {identity_error:.3e}",
    ]


def projection_consistency_lines() -> list[str]:
    """Compare manual pinhole projection with K [R|t] homogeneous projection."""
    calib = StereoCalibration()
    poses = ground_truth_poses()
    rng = np.random.default_rng(RNG_SEED)
    points_c0 = rng.uniform([-1.0, -0.5, 2.0], [1.0, 0.5, 6.0], size=(40, 3))
    points_w = poses["T_W_C0"].apply(points_c0)

    for label, T_camera_world in (("C0", poses["T_W_C0"].inverse()), ("C1", poses["T_C1_W"])):
        points_c = T_camera_world.apply(points_w)
        manual_px, manual_valid = project_left(points_c, calib)
        homog_px, homog_valid = project_homogeneous(points_w, T_camera_world, calib)
        valid = manual_valid & homog_valid & np.all(np.isfinite(manual_px), axis=1) & np.all(np.isfinite(homog_px), axis=1)
        max_error = float(np.max(np.linalg.norm(manual_px[valid] - homog_px[valid], axis=1))) if np.any(valid) else float("nan")
        yield f"{label}: manual pinhole vs K[R|t] homogeneous max pixel error = {max_error:.3e}px"


def nonzero_lever_arm_lines() -> list[str]:
    """Verify the full SE(3) chain with a translated camera mount."""
    camera_position_body = np.array([0.15, 0.0, 0.20], dtype=np.float64)
    lines = [
        "\nNon-zero camera lever-arm test",
        f"camera position in body frame = {camera_position_body.tolist()} m",
    ]
    result_lines = run_experiment(
        "Lever-arm ideal geometry",
        pixel_noise_sigma=0.0,
        outlier_ratio=0.0,
        landmark_count=120,
        camera_position_body=camera_position_body,
    )
    for line in result_lines:
        if line.startswith("solvePnP:") or line.startswith("solvePnPRansac:") or line.startswith("solvePnPRefineLM:"):
            lines.append(line)
    return lines


def deterministic_repeat_lines() -> list[str]:
    first = run_experiment("Determinism check noisy 30% outliers", 0.5, 0.30, 120)
    second = run_experiment("Determinism check noisy 30% outliers", 0.5, 0.30, 120)
    identical = first == second
    max_line_delta = 0.0 if identical else float("nan")
    return [
        "\nDeterminism check",
        f"same seeded noisy/RANSAC experiment identical text output: {identical}",
        f"maximum observed numeric difference: {max_line_delta:.3e}",
    ]


def failure_tests() -> list[str]:
    calib = StereoCalibration()
    lines = ["\nFailure / diagnostic tests"]

    too_few = np.zeros((3, 3), dtype=np.float64)
    too_few_img = np.zeros((3, 2), dtype=np.float64)
    lines.append(f"too few correspondences: {solve_plain_pnp(too_few, too_few_img, calib).message}")

    left = np.array([[640.0, 360.0], [641.0, 360.0], [660.0, 370.0]], dtype=np.float64)
    right = left.copy()
    tri, valid = triangulate_disparity(left, right, calib)
    lines.append(f"zero disparity: valid={int(np.count_nonzero(valid))}/{len(valid)}, diagnostic=invalid disparity")

    tiny_right = left.copy()
    tiny_right[:, 0] -= 0.05
    tiny_tri, tiny_valid = triangulate_disparity(left, tiny_right, calib)
    tiny_depth = tiny_tri[tiny_valid, 2]
    lines.append(
        "very small disparity: "
        f"valid={int(np.count_nonzero(tiny_valid))}/{len(tiny_valid)}, "
        f"depth range={np.min(tiny_depth):.1f}..{np.max(tiny_depth):.1f} m diagnostic=poor depth conditioning"
    )

    behind = np.array([[0.0, 0.0, -1.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    _, depth_ok = project_left(behind, calib)
    lines.append(f"behind-camera geometry: positive-depth mask={depth_ok.tolist()}")

    outside = np.array([[10.0, 0.0, 2.0], [0.0, 0.0, 2.0]], dtype=np.float64)
    px, depth_ok = project_left(outside, calib)
    lines.append(f"outside-image visibility: inside mask={(inside_image(px, calib) & depth_ok).tolist()}")

    collinear = np.column_stack([np.linspace(0.0, 1.0, 20), np.zeros(20), np.linspace(2.0, 4.0, 20)])
    lines.append(f"nearly collinear 3D geometry: singular ratio={collinearity_ratio(collinear):.3e} diagnostic=degenerate distribution")

    for ratio in (0.6, 0.8):
        result_lines = run_experiment(f"robust-estimation stress {int(ratio * 100)}% outliers", 0.5, ratio, 80)
        summary = [line for line in result_lines if line.startswith("solvePnPRansac:")]
        lines.append(f"excessive outlier ratio {ratio:.0%}: {summary[0] if summary else 'no RANSAC result'}")

    lines.append("large inter-frame motion diagnostic: report as insufficient visibility overlap / poor correspondence geometry, not generic PnP failure")
    return lines


def main() -> None:
    np.set_printoptions(precision=6, suppress=True)
    print("Milestone 6A: Offline Stereo Visual Odometry Reference Lab")
    print("No ROS topics, no TF publishers, no Docker/image modifications.")
    print(f"OpenCV: {cv2.__version__}")
    print(f"NumPy: {np.__version__}")
    print(f"solvePnPRefineLM available: {hasattr(cv2, 'solvePnPRefineLM')}")
    print("\nSynthetic calibration placeholders:")
    print(StereoCalibration())
    print("K =")
    print(StereoCalibration().K)
    print("Assumptions: synthetic principal point, skew=0, fx/fy in pixels, no lens distortion.")

    for line in extrinsic_sanity_lines():
        print(line)
    for line in transform_round_trip_lines():
        print(line)
    print("\nProjection consistency tests")
    for line in projection_consistency_lines():
        print(line)

    experiments: Iterable[tuple[str, float, float]] = [
        ("Experiment A: ideal geometry sanity test", 0.0, 0.0),
        ("Experiment B: noisy 0% outliers", 0.5, 0.0),
        ("Experiment B: noisy 10% outliers", 0.5, 0.10),
        ("Experiment B: noisy 30% outliers", 0.5, 0.30),
    ]
    for name, noise, outliers in experiments:
        for line in run_experiment(name, noise, outliers):
            print(line)

    for line in nonzero_lever_arm_lines():
        print(line)

    for line in deterministic_repeat_lines():
        print(line)

    for line in failure_tests():
        print(line)

    print("\nInterpretation")
    print("- Ideal stereo triangulation and PnP should be near numerical precision.")
    print("- Pixel noise mainly perturbs disparity, depth, reprojection error, and pose.")
    print("- RANSAC is expected to preserve pose quality when deterministic outliers are injected.")
    print("- AMR yaw is recovered only after inverting solvePnP output and removing the fixed camera-body extrinsic.")
    print("- Future real deployment needs calibrated camera intrinsics/extrinsics and acquisition timestamps.")


if __name__ == "__main__":
    main()
