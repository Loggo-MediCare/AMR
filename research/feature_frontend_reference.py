#!/usr/bin/env python3
"""Offline synthetic feature-frontend reference lab.

Milestone 6B scope:
- Synthetic grayscale images only.
- Common stereo reconstruction stage, then controlled KLT vs ORB temporal
  association comparison.
- PnP/RANSAC uses noisy stereo-reconstructed objectPoints, never ground truth.
- No ROS topics, cv_bridge runtime, camera drivers, Isaac ROS, cuVSLAM, or TF.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import sys
from typing import Callable

import cv2
import numpy as np


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import stereo_visual_odometry_reference as svo  # noqa: E402


RNG_SEED = 4242
PATCH_SIZE = 31
MIN_CORRESPONDENCES = 20
RANSAC_ITERATIONS = 100
RANSAC_REPROJECTION_THRESHOLD_PX = 3.0
RANSAC_CONFIDENCE = 0.99
RANSAC_FLAGS = cv2.SOLVEPNP_EPNP
RANSAC_USE_EXTRINSIC_GUESS = False
DIST_COEFFS = np.zeros((4, 1), dtype=np.float64)


@dataclass
class FeatureRecord:
    landmark_id: int
    left_k: np.ndarray
    right_k: np.ndarray
    left_k1: np.ndarray
    point_w_noisy: np.ndarray
    point_w_true: np.ndarray
    stereo_distance: float
    disparity: float


@dataclass
class FrontendResult:
    name: str
    attempted: int
    retained: int
    correct: int
    fb_pass: int
    pixel_rmse: float
    pnp_lines: list[str]
    region_stats: dict[str, dict[str, int]]


def make_patch(landmark_id: int, repeated: bool = False, low_texture: bool = False) -> np.ndarray:
    """Create a deterministic local patch with corners and texture."""
    rng_id = 7 if repeated else 1000 + landmark_id
    rng = np.random.default_rng(rng_id)
    patch = np.full((PATCH_SIZE, PATCH_SIZE), 128, dtype=np.uint8)
    if low_texture:
        patch[:, :] = 150
        cv2.circle(patch, (PATCH_SIZE // 2, PATCH_SIZE // 2), 3, 170, -1)
        return patch

    cell = PATCH_SIZE // 5
    for row in range(5):
        for col in range(5):
            value = 45 if rng.random() < 0.5 else 220
            y0, y1 = row * cell, PATCH_SIZE if row == 4 else (row + 1) * cell
            x0, x1 = col * cell, PATCH_SIZE if col == 4 else (col + 1) * cell
            patch[y0:y1, x0:x1] = value
    cv2.line(patch, (3, 3), (PATCH_SIZE - 4, 3), 255, 2)
    cv2.line(patch, (3, 3), (3, PATCH_SIZE - 4), 30, 2)
    cv2.circle(patch, (PATCH_SIZE // 2, PATCH_SIZE // 2), 5 if repeated else 3 + (landmark_id % 4), 255, 1)
    return patch


def render_patch(image: np.ndarray, center: np.ndarray, patch: np.ndarray) -> bool:
    x = int(round(float(center[0])))
    y = int(round(float(center[1])))
    half = PATCH_SIZE // 2
    h, w = image.shape
    if x - half < 0 or y - half < 0 or x + half + 1 > w or y + half + 1 > h:
        return False
    roi = image[y - half : y + half + 1, x - half : x + half + 1]
    np.maximum(roi, patch, out=roi)
    return True


def assign_region(points: np.ndarray, image_width: int) -> np.ndarray:
    return np.where(points[:, 0] < image_width / 2.0, "left", "right")


def build_scene(
    count: int = 140,
    repeated_fraction: float = 0.0,
    low_texture_fraction: float = 0.0,
) -> dict:
    calib = svo.StereoCalibration()
    poses = svo.ground_truth_poses()
    landmarks_c0, _ = svo.generate_landmarks_c0(count, calib)
    landmarks_w = poses["T_W_C0"].apply(landmarks_c0)
    left_k, left_ok = svo.project_left(landmarks_c0, calib)
    right_k, right_ok = svo.project_right_from_left(landmarks_c0, calib)
    left_k1_c = poses["T_C1_W"].apply(landmarks_w)
    left_k1, left_k1_ok = svo.project_left(left_k1_c, calib)
    visible = (
        left_ok
        & right_ok
        & left_k1_ok
        & svo.inside_image(left_k, calib)
        & svo.inside_image(right_k, calib)
        & svo.inside_image(left_k1, calib)
    )
    indices = np.flatnonzero(visible)
    return {
        "calib": calib,
        "poses": poses,
        "ids": indices.astype(np.int32),
        "landmarks_c0": landmarks_c0[indices],
        "landmarks_w": landmarks_w[indices],
        "left_k": left_k[indices],
        "right_k": right_k[indices],
        "left_k1": left_k1[indices],
        "repeated_fraction": repeated_fraction,
        "low_texture_fraction": low_texture_fraction,
    }


def render_images(scene: dict) -> dict[str, np.ndarray]:
    calib: svo.StereoCalibration = scene["calib"]
    images = {
        "left_k": np.full((calib.image_height, calib.image_width), 24, dtype=np.uint8),
        "right_k": np.full((calib.image_height, calib.image_width), 24, dtype=np.uint8),
        "left_k1": np.full((calib.image_height, calib.image_width), 24, dtype=np.uint8),
    }
    total = len(scene["ids"])
    repeated_cutoff = int(round(total * scene["repeated_fraction"]))
    low_texture_cutoff = int(round(total * scene["low_texture_fraction"]))
    for order, landmark_id in enumerate(scene["ids"]):
        patch = make_patch(
            int(landmark_id),
            repeated=order < repeated_cutoff,
            low_texture=order < low_texture_cutoff,
        )
        render_patch(images["left_k"], scene["left_k"][order], patch)
        render_patch(images["right_k"], scene["right_k"][order], patch)
        render_patch(images["left_k1"], scene["left_k1"][order], patch)
    return images


def perturb_image(image: np.ndarray, mode: str) -> np.ndarray:
    out = image.astype(np.float32)
    if mode == "ideal":
        return image.copy()
    if mode == "brightness":
        out += 35.0
    elif mode == "contrast":
        out = 128.0 + 1.35 * (out - 128.0)
    elif mode == "noise":
        rng = np.random.default_rng(RNG_SEED)
        out += rng.normal(0.0, 8.0, out.shape)
    elif mode == "blur":
        return cv2.GaussianBlur(image, (7, 7), 1.6)
    elif mode == "occlusion":
        out = image.copy()
        h, w = out.shape
        out[h // 3 : 2 * h // 3, w // 3 : 2 * w // 3] = 24
        return out
    elif mode == "local_right_blur":
        out = image.copy()
        w = out.shape[1]
        out[:, w // 2 :] = cv2.GaussianBlur(out[:, w // 2 :], (9, 9), 2.0)
        return out
    else:
        raise ValueError(f"unknown perturbation mode: {mode}")
    return np.clip(out, 0, 255).astype(np.uint8)


def optional_ssim(reference: np.ndarray, degraded: np.ndarray) -> str:
    try:
        from skimage.metrics import structural_similarity as ssim  # type: ignore
    except Exception:
        return "unavailable"
    return f"{float(ssim(reference, degraded, data_range=255)):.6f}"


def keypoints_from_points(points: np.ndarray, size: float = 31.0) -> list[cv2.KeyPoint]:
    return [cv2.KeyPoint(float(p[0]), float(p[1]), size) for p in points]


def compute_orb_descriptors(image: np.ndarray, points: np.ndarray) -> tuple[list[cv2.KeyPoint], np.ndarray | None]:
    orb = cv2.ORB_create(nfeatures=1000, patchSize=31, edgeThreshold=12, fastThreshold=5)
    keypoints = keypoints_from_points(points)
    return orb.compute(image, keypoints)


def good_feature_points(image: np.ndarray, max_corners: int = 260) -> np.ndarray:
    pts = cv2.goodFeaturesToTrack(
        image,
        maxCorners=max_corners,
        qualityLevel=0.01,
        minDistance=7,
        blockSize=7,
        useHarrisDetector=False,
    )
    if pts is None:
        return np.empty((0, 2), dtype=np.float64)
    return pts.reshape(-1, 2).astype(np.float64)


def match_descriptors(desc_a: np.ndarray | None, desc_b: np.ndarray | None, ratio: float = 0.82) -> list[cv2.DMatch]:
    if desc_a is None or desc_b is None or len(desc_a) < 2 or len(desc_b) < 2:
        return []
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    raw = matcher.knnMatch(desc_a, desc_b, k=2)
    good: list[cv2.DMatch] = []
    for pair in raw:
        if len(pair) != 2:
            continue
        best, second = pair
        if best.distance < ratio * second.distance:
            good.append(best)
    return good


def common_stereo_frontend(scene: dict, images: dict[str, np.ndarray]) -> tuple[list[FeatureRecord], dict]:
    calib: svo.StereoCalibration = scene["calib"]
    left_seed_points = good_feature_points(images["left_k"], 260)
    right_seed_points = good_feature_points(images["right_k"], 260)
    kp_l, desc_l = compute_orb_descriptors(images["left_k"], left_seed_points)
    kp_r, desc_r = compute_orb_descriptors(images["right_k"], right_seed_points)
    matches = match_descriptors(desc_l, desc_r)
    used_right: set[int] = set()
    records: list[FeatureRecord] = []
    stereo_correct = 0
    stereo_incorrect = 0
    for match in matches:
        if match.trainIdx in used_right:
            continue
        left_idx = match.queryIdx
        right_idx = match.trainIdx
        left_px = np.array(kp_l[left_idx].pt, dtype=np.float64)
        right_px = np.array(kp_r[right_idx].pt, dtype=np.float64)
        left_landmark_idx = nearest_landmark_index(left_px, scene["left_k"], max_dist=16.0)
        right_landmark_idx = nearest_landmark_index(right_px, scene["right_k"], max_dist=16.0)
        if left_landmark_idx is None or right_landmark_idx is None:
            continue
        vertical_error = abs(left_px[1] - right_px[1])
        disparity = left_px[0] - right_px[0]
        if vertical_error > 2.0 or disparity < 2.0:
            continue
        tri_c0, valid = svo.triangulate_disparity(left_px.reshape(1, 2), right_px.reshape(1, 2), calib)
        if not bool(valid[0]) or not np.all(np.isfinite(tri_c0[0])):
            continue
        point_w_noisy = scene["poses"]["T_W_C0"].apply(tri_c0)[0]
        used_right.add(right_idx)
        correct = int(scene["ids"][left_landmark_idx]) == int(scene["ids"][right_landmark_idx])
        stereo_correct += int(correct)
        stereo_incorrect += int(not correct)
        offset = left_px - scene["left_k"][left_landmark_idx]
        expected_left_k1 = scene["left_k1"][left_landmark_idx] + offset
        expected_right = scene["right_k"][left_landmark_idx] + offset
        true_c0, true_valid = svo.triangulate_disparity(left_px.reshape(1, 2), expected_right.reshape(1, 2), calib)
        point_w_true = scene["poses"]["T_W_C0"].apply(true_c0)[0] if bool(true_valid[0]) else scene["landmarks_w"][left_landmark_idx]
        records.append(
            FeatureRecord(
                landmark_id=int(scene["ids"][left_landmark_idx]),
                left_k=left_px,
                right_k=right_px,
                left_k1=expected_left_k1.astype(np.float64),
                point_w_noisy=point_w_noisy,
                point_w_true=point_w_true.astype(np.float64),
                stereo_distance=float(match.distance),
                disparity=float(disparity),
            )
        )
    metrics = {
        "left_orb_keypoints": len(kp_l),
        "right_orb_keypoints": len(kp_r),
        "good_features_left": len(left_seed_points),
        "good_features_right": len(right_seed_points),
        "stereo_candidates": len(matches),
        "stereo_records": len(records),
        "stereo_correct": stereo_correct,
        "stereo_incorrect": stereo_incorrect,
        "stereo_precision": stereo_correct / max(1, stereo_correct + stereo_incorrect),
        "stereo_recall": stereo_correct / max(1, len(scene["ids"])),
        "triangulation_rmse": triangulation_rmse(records),
        "depth_rmse": depth_rmse(records, scene),
    }
    return records, metrics


def triangulation_rmse(records: list[FeatureRecord]) -> float:
    if not records:
        return float("nan")
    errors = [np.linalg.norm(r.point_w_noisy - r.point_w_true) for r in records]
    return float(np.sqrt(np.mean(np.square(errors))))


def depth_rmse(records: list[FeatureRecord], scene: dict) -> float:
    if not records:
        return float("nan")
    id_to_idx = {int(lid): i for i, lid in enumerate(scene["ids"])}
    errors = []
    T_C0_W = scene["poses"]["T_W_C0"].inverse()
    for record in records:
        idx = id_to_idx[record.landmark_id]
        point_c0_noisy = T_C0_W.apply(record.point_w_noisy.reshape(1, 3))[0]
        errors.append(point_c0_noisy[2] - scene["landmarks_c0"][idx, 2])
    return float(np.sqrt(np.mean(np.square(errors))))


def nearest_landmark_id(point: np.ndarray, targets: np.ndarray, ids: np.ndarray, max_dist: float = 6.0) -> int | None:
    distances = np.linalg.norm(targets - point.reshape(1, 2), axis=1)
    best = int(np.argmin(distances))
    if float(distances[best]) > max_dist:
        return None
    return int(ids[best])


def nearest_landmark_index(point: np.ndarray, targets: np.ndarray, max_dist: float = 16.0) -> int | None:
    distances = np.linalg.norm(targets - point.reshape(1, 2), axis=1)
    best = int(np.argmin(distances))
    if float(distances[best]) > max_dist:
        return None
    return best


def pnp_report(name: str, records: list[FeatureRecord], image_points: np.ndarray, poses: dict) -> list[str]:
    if len(records) < MIN_CORRESPONDENCES:
        return [f"{name}: skipped, too few correspondences ({len(records)} < {MIN_CORRESPONDENCES})"]
    object_points = np.array([r.point_w_noisy for r in records], dtype=np.float64)
    image_points = image_points.astype(np.float64)
    calib = svo.StereoCalibration()
    params = (
        f"params(iterationsCount={RANSAC_ITERATIONS}, "
        f"reprojectionError_px={RANSAC_REPROJECTION_THRESHOLD_PX:.1f}, "
        f"confidence={RANSAC_CONFIDENCE:.2f}, flags=SOLVEPNP_EPNP, "
        f"useExtrinsicGuess={RANSAC_USE_EXTRINSIC_GUESS}, distCoeffs=zeros)"
    )
    try:
        retval, rvec, tvec, inliers = cv2.solvePnPRansac(
            object_points,
            image_points,
            calib.K,
            DIST_COEFFS,
            iterationsCount=RANSAC_ITERATIONS,
            reprojectionError=RANSAC_REPROJECTION_THRESHOLD_PX,
            confidence=RANSAC_CONFIDENCE,
            flags=RANSAC_FLAGS,
            useExtrinsicGuess=RANSAC_USE_EXTRINSIC_GUESS,
        )
    except cv2.error as exc:
        return [f"{name}: solvePnPRansac failed with OpenCV error: {exc}; {params}"]
    if not retval or rvec is None or tvec is None or inliers is None or len(inliers) == 0:
        return [f"{name}: solvePnPRansac retval=False/no inliers; pose not reported; {params}"]

    result = svo.PoseEstimateResult("solvePnPRansac", True, "ok", rvec, tvec, inliers)
    lines = [f"solvePnPRansac {params}"]
    lines.append(f"RANSAC inliers={len(inliers)}/{len(object_points)} ({len(inliers)/len(object_points):.1%})")
    lines.append(svo.pose_metrics(result, poses, object_points, image_points))

    inlier_idx = inliers.reshape(-1)
    if hasattr(cv2, "solvePnPRefineLM") and len(inlier_idx) >= 4:
        refined_rvec = rvec.copy()
        refined_tvec = tvec.copy()
        try:
            cv2.solvePnPRefineLM(
                object_points[inlier_idx],
                image_points[inlier_idx],
                calib.K,
                DIST_COEFFS,
                refined_rvec,
                refined_tvec,
            )
            refined = svo.PoseEstimateResult("solvePnPRefineLM(inliers only)", True, "ok", refined_rvec, refined_tvec, inliers)
            lines.append(svo.pose_metrics(refined, poses, object_points, image_points))
        except cv2.error as exc:
            lines.append(f"{name}: solvePnPRefineLM failed on RANSAC inliers: {exc}")
    else:
        lines.append(f"{name}: solvePnPRefineLM skipped; inliers={len(inlier_idx)}")
    return [f"{name}: {line}" for line in lines]


def empty_region_stats() -> dict[str, dict[str, int]]:
    return {
        "left": {"retained": 0, "correct": 0, "inliers": 0},
        "right": {"retained": 0, "correct": 0, "inliers": 0},
    }


def ransac_inlier_indices(records: list[FeatureRecord], image_points: np.ndarray) -> set[int]:
    if len(records) < MIN_CORRESPONDENCES:
        return set()
    object_points = np.array([r.point_w_noisy for r in records], dtype=np.float64)
    try:
        retval, _, _, inliers = cv2.solvePnPRansac(
            object_points,
            image_points.astype(np.float64),
            svo.StereoCalibration().K,
            DIST_COEFFS,
            iterationsCount=RANSAC_ITERATIONS,
            reprojectionError=RANSAC_REPROJECTION_THRESHOLD_PX,
            confidence=RANSAC_CONFIDENCE,
            flags=RANSAC_FLAGS,
            useExtrinsicGuess=RANSAC_USE_EXTRINSIC_GUESS,
        )
    except cv2.error:
        return set()
    if not retval or inliers is None:
        return set()
    return set(int(i) for i in inliers.reshape(-1))


def build_region_stats(records: list[FeatureRecord], image_points: np.ndarray, correct_flags: list[bool]) -> dict[str, dict[str, int]]:
    stats = empty_region_stats()
    if not records:
        return stats
    inliers = ransac_inlier_indices(records, image_points) if len(records) == len(image_points) else set()
    for idx, record in enumerate(records):
        region = "left" if record.left_k1[0] < svo.StereoCalibration().image_width / 2.0 else "right"
        stats[region]["retained"] += 1
        stats[region]["correct"] += int(bool(correct_flags[idx]))
        stats[region]["inliers"] += int(idx in inliers)
    return stats


def klt_temporal(scene: dict, images: dict[str, np.ndarray], records: list[FeatureRecord]) -> FrontendResult:
    if not records:
        return FrontendResult("KLT", 0, 0, 0, 0, float("nan"), [], empty_region_stats())
    pts0 = np.array([r.left_k for r in records], dtype=np.float32).reshape(-1, 1, 2)
    pts1, status, err = cv2.calcOpticalFlowPyrLK(images["left_k"], images["left_k1"], pts0, None, winSize=(31, 31), maxLevel=3)
    back, status_b, _ = cv2.calcOpticalFlowPyrLK(images["left_k1"], images["left_k"], pts1, None, winSize=(31, 31), maxLevel=3)
    kept_records: list[FeatureRecord] = []
    image_points: list[np.ndarray] = []
    correct = 0
    fb_pass = 0
    pixel_errors = []
    correct_flags: list[bool] = []
    h, w = images["left_k1"].shape
    for idx, record in enumerate(records):
        if status[idx, 0] != 1 or status_b[idx, 0] != 1:
            continue
        pt = pts1[idx, 0].astype(np.float64)
        if pt[0] < 0 or pt[0] >= w or pt[1] < 0 or pt[1] >= h:
            continue
        fb_error = float(np.linalg.norm(back[idx, 0] - pts0[idx, 0]))
        if fb_error > 1.5:
            continue
        fb_pass += 1
        pixel_error = float(np.linalg.norm(pt - record.left_k1))
        pixel_errors.append(pixel_error)
        is_correct = pixel_error <= 3.0
        if is_correct:
            correct += 1
        kept_records.append(record)
        image_points.append(pt)
        correct_flags.append(is_correct)
    pnp_lines = pnp_report("KLT", kept_records, np.array(image_points), scene["poses"]) if image_points else ["KLT: no image points"]
    region_stats = build_region_stats(kept_records, np.array(image_points), correct_flags) if image_points else empty_region_stats()
    return FrontendResult(
        "KLT",
        attempted=len(records),
        retained=len(kept_records),
        correct=correct,
        fb_pass=fb_pass,
        pixel_rmse=float(np.sqrt(np.mean(np.square(pixel_errors)))) if pixel_errors else float("nan"),
        pnp_lines=pnp_lines,
        region_stats=region_stats,
    )


def orb_temporal(scene: dict, images: dict[str, np.ndarray], records: list[FeatureRecord]) -> FrontendResult:
    if not records:
        return FrontendResult("ORB", 0, 0, 0, 0, float("nan"), [], empty_region_stats())
    start_points = np.array([r.left_k for r in records], dtype=np.float64)
    target_points = good_feature_points(images["left_k1"], 320)
    if len(target_points) == 0:
        return FrontendResult("ORB", len(records), 0, 0, 0, float("nan"), ["ORB: no target features"], empty_region_stats())
    _, desc_start = compute_orb_descriptors(images["left_k"], start_points)
    kp_target, desc_target = compute_orb_descriptors(images["left_k1"], target_points)
    matches = match_descriptors(desc_start, desc_target, ratio=0.82)
    used_target: set[int] = set()
    kept_records: list[FeatureRecord] = []
    image_points: list[np.ndarray] = []
    correct = 0
    pixel_errors = []
    correct_flags: list[bool] = []
    for match in matches:
        if match.trainIdx in used_target:
            continue
        record = records[match.queryIdx]
        pt = np.array(kp_target[match.trainIdx].pt, dtype=np.float64)
        used_target.add(match.trainIdx)
        pixel_error = float(np.linalg.norm(pt - record.left_k1))
        is_correct = pixel_error <= 3.0
        if is_correct:
            correct += 1
        pixel_errors.append(pixel_error)
        kept_records.append(record)
        image_points.append(pt)
        correct_flags.append(is_correct)
    pnp_lines = pnp_report("ORB", kept_records, np.array(image_points), scene["poses"]) if image_points else ["ORB: no image points"]
    region_stats = build_region_stats(kept_records, np.array(image_points), correct_flags) if image_points else empty_region_stats()
    return FrontendResult(
        "ORB",
        attempted=len(records),
        retained=len(kept_records),
        correct=correct,
        fb_pass=0,
        pixel_rmse=float(np.sqrt(np.mean(np.square(pixel_errors)))) if pixel_errors else float("nan"),
        pnp_lines=pnp_lines,
        region_stats=region_stats,
    )


def apply_experiment(scene: dict, mode: str) -> tuple[dict[str, np.ndarray], dict[str, str]]:
    clean = render_images(scene)
    images = {key: value.copy() for key, value in clean.items()}
    diagnostics = {
        "mode": mode,
        "brightness_offset": "0",
        "contrast_factor": "1.0",
        "noise_sigma": "0",
        "blur": "none",
        "occlusion": "0%",
        "ssim_left_k1": "n/a",
    }
    if mode == "photometric_brightness":
        images["left_k1"] = perturb_image(images["left_k1"], "brightness")
        diagnostics["brightness_offset"] = "+35"
    elif mode == "photometric_contrast":
        images["left_k1"] = perturb_image(images["left_k1"], "contrast")
        diagnostics["contrast_factor"] = "1.35"
    elif mode == "structural_blur":
        images["left_k1"] = perturb_image(images["left_k1"], "blur")
        diagnostics["blur"] = "Gaussian 7x7 sigma=1.6"
    elif mode == "structural_occlusion":
        images["left_k1"] = perturb_image(images["left_k1"], "occlusion")
        diagnostics["occlusion"] = "central ROI"
    elif mode == "noise":
        images["left_k"] = perturb_image(images["left_k"], "noise")
        images["right_k"] = perturb_image(images["right_k"], "noise")
        images["left_k1"] = perturb_image(images["left_k1"], "noise")
        diagnostics["noise_sigma"] = "8 gray levels"
    elif mode == "local_right_blur":
        images["left_k1"] = perturb_image(images["left_k1"], "local_right_blur")
        diagnostics["blur"] = "right half Gaussian 9x9 sigma=2.0"
    elif mode == "ideal":
        pass
    else:
        raise ValueError(mode)
    diagnostics["ssim_left_k1"] = optional_ssim(clean["left_k1"], images["left_k1"])
    return images, diagnostics


def precision_recall(correct: int, retained: int, total: int) -> tuple[float, float]:
    return correct / max(1, retained), correct / max(1, total)


def summarize_frontend(result: FrontendResult, total: int) -> list[str]:
    precision, recall = precision_recall(result.correct, result.retained, total)
    lines = [
        (
            f"{result.name}: attempted={result.attempted}, retained={result.retained}, "
            f"correct={result.correct}, precision={precision:.3f}, recall={recall:.3f}, "
            f"pixel_rmse={result.pixel_rmse:.3f}px"
        )
    ]
    if result.name == "KLT":
        lines[0] += f", fb_pass={result.fb_pass}"
    lines.extend(f"  {line}" for line in result.pnp_lines)
    return lines


def region_report(scene: dict, records: list[FeatureRecord], klt: FrontendResult, orb: FrontendResult) -> list[str]:
    regions = assign_region(np.array([r.left_k1 for r in records]), scene["calib"].image_width)
    left_available = int(np.count_nonzero(regions == "left"))
    right_available = int(np.count_nonzero(regions == "right"))
    def fmt(name: str, result: FrontendResult, region: str) -> str:
        stats = result.region_stats[region]
        precision = stats["correct"] / max(1, stats["retained"])
        recall = stats["correct"] / max(1, left_available if region == "left" else right_available)
        return (
            f"{name} {region}: retained={stats['retained']}, correct={stats['correct']}, "
            f"precision={precision:.3f}, recall={recall:.3f}, ransac_inliers={stats['inliers']}"
        )
    return [
        f"region availability: left={left_available}, right={right_available}",
        fmt("KLT", klt, "left"),
        fmt("KLT", klt, "right"),
        fmt("ORB", orb, "left"),
        fmt("ORB", orb, "right"),
        "local degradation is temporal-only here; stereo-depth degradation would require a separate RIGHT_k local test",
    ]


def run_case(label: str, mode: str, scene: dict) -> list[str]:
    images, diagnostics = apply_experiment(scene, mode)
    records, stereo = common_stereo_frontend(scene, images)
    klt = klt_temporal(scene, images, records)
    orb = orb_temporal(scene, images, records)
    lines = [
        f"\n{label}",
        "-" * len(label),
        f"diagnostics: {diagnostics}",
        (
            "common stereo: "
            f"left_orb={stereo['left_orb_keypoints']}, right_orb={stereo['right_orb_keypoints']}, "
            f"gftt_left={stereo['good_features_left']}, candidates={stereo['stereo_candidates']}, "
            f"records={stereo['stereo_records']}, precision={stereo['stereo_precision']:.3f}, "
            f"recall={stereo['stereo_recall']:.3f}, 3d_rmse={stereo['triangulation_rmse']:.6f} m, "
            f"depth_rmse={stereo['depth_rmse']:.6f} m"
        ),
    ]
    lines.extend(summarize_frontend(klt, max(1, len(records))))
    lines.extend(summarize_frontend(orb, max(1, len(records))))
    if mode == "local_right_blur":
        lines.extend(region_report(scene, records, klt, orb))
    return lines


def failure_diagnostics(scene: dict) -> list[str]:
    blank = {
        "left_k": np.full((scene["calib"].image_height, scene["calib"].image_width), 24, dtype=np.uint8),
        "right_k": np.full((scene["calib"].image_height, scene["calib"].image_width), 24, dtype=np.uint8),
        "left_k1": np.full((scene["calib"].image_height, scene["calib"].image_width), 24, dtype=np.uint8),
    }
    records, stereo = common_stereo_frontend(scene, blank)
    return [
        "\nFailure diagnostics",
        f"blank images: stereo_records={len(records)}, diagnostic={'too few stereo matches' if len(records) < MIN_CORRESPONDENCES else 'unexpected'}",
        f"minimum correspondence policy: require >= {MIN_CORRESPONDENCES}, not a hard-coded theoretical PnP minimum",
        f"RANSAC reprojection threshold is experimental: {RANSAC_REPROJECTION_THRESHOLD_PX:.1f}px",
        "RANSAC inliers are geometrically consistent under the selected model/threshold, not guaranteed semantically correct",
    ]


def main() -> None:
    print("Milestone 6B: Synthetic Image Feature Frontend Reference Lab")
    print("Offline only. No ROS topics, no package changes, no Docker changes.")
    print(f"OpenCV: {cv2.__version__}")
    print(f"NumPy: {np.__version__}")
    print("SSIM:", "unavailable (no package installed)" if optional_ssim(np.zeros((8, 8), np.uint8), np.zeros((8, 8), np.uint8)) == "unavailable" else "available")
    print("\nStage separation: frontend filtering -> PnP set -> solvePnPRansac -> inlier RefineLM -> AMR body pose")

    base_scene = build_scene()
    cases = [
        ("Experiment A - ideal", "ideal", base_scene),
        ("Experiment B1 - brightness", "photometric_brightness", base_scene),
        ("Experiment B1 - contrast", "photometric_contrast", base_scene),
        ("Experiment B2 - blur", "structural_blur", base_scene),
        ("Experiment B2 - occlusion", "structural_occlusion", base_scene),
        ("Experiment B3 - Gaussian noise", "noise", base_scene),
        ("Experiment B4 - local temporal right-half blur", "local_right_blur", base_scene),
        ("Experiment B2 - local texture loss", "ideal", build_scene(low_texture_fraction=0.35)),
        ("Experiment C - repeated patch ambiguity", "ideal", build_scene(repeated_fraction=0.35)),
        ("Experiment C - strong repeated patch ambiguity", "ideal", build_scene(repeated_fraction=1.0)),
    ]
    for label, mode, scene in cases:
        for line in run_case(label, mode, scene):
            print(line)

    for line in failure_diagnostics(base_scene):
        print(line)

    print("\nBreakdown summary")
    print("- KLT tends to break when temporal blur/occlusion/noise prevents stable forward-backward tracks.")
    print("- ORB tends to break when descriptors become ambiguous or low-texture/repeated patches dominate.")
    print("- Image quality, feature survival, correspondence quality, RANSAC consistency, and pose accuracy are reported separately.")


if __name__ == "__main__":
    main()
