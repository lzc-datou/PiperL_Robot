"""Corner estimators and coordinate transforms. All 3D calculations use metres."""

from dataclasses import dataclass, field

import numpy as np


CORNERS = ('TL', 'TR', 'BR', 'BL')
METHODS = ('depth', 'pnp')


@dataclass
class Estimate:
    status: str
    points_m: np.ndarray | None = None
    metrics: dict = field(default_factory=dict)


@dataclass
class Frame:
    rgb_stamp_ns: int
    depth_stamp_ns: int | None
    estimates: dict[str, Estimate]
    pixels: np.ndarray | None = None
    pose_gap_ms: float | None = None


def transform_points(transform, points):
    return points @ transform[:3, :3].T + transform[:3, 3]


def marker_corners(side_m):
    return 0.5 * side_m * np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]])


class Detector:
    def __init__(self, cv, dictionary_name, marker_id):
        self.marker_id = marker_id
        aruco = cv.aruco
        dictionary = aruco.getPredefinedDictionary(getattr(aruco, dictionary_name))
        if not 0 <= marker_id < len(dictionary.bytesList):
            raise ValueError(f'ID {marker_id} 不在 {dictionary_name} 字典中')
        if hasattr(aruco, 'ArucoDetector'):
            parameters = aruco.DetectorParameters()
            parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            self.detect = aruco.ArucoDetector(dictionary, parameters).detectMarkers
        else:
            parameters = aruco.DetectorParameters_create()
            parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            self.detect = lambda image: aruco.detectMarkers(image, dictionary, parameters=parameters)

    def corners(self, image):
        corners, ids, _ = self.detect(image)
        matches = [] if ids is None else np.flatnonzero(ids.ravel() == self.marker_id)
        if len(matches) != 1:
            return None, 'tag_missing' if len(matches) == 0 else 'duplicate_tag'
        return np.asarray(corners[matches[0]], dtype=float).reshape(4, 2), 'ok'


def pnp_estimate(cv, pixels, camera, model_corners, max_rms_px):
    count, rotations, translations, _ = cv.solvePnPGeneric(
        model_corners, pixels, camera.k, camera.d, flags=cv.SOLVEPNP_IPPE_SQUARE)
    candidates = []
    for rotation, translation in zip(rotations, translations):
        if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
            continue
        matrix, _ = cv.Rodrigues(rotation)
        points = model_corners @ matrix.T + translation.ravel()
        if np.any(points[:, 2] <= 0):
            continue
        projected, _ = cv.projectPoints(model_corners, rotation, translation, camera.k, camera.d)
        rms = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - pixels) ** 2, axis=1))))
        if np.isfinite(rms):
            candidates.append((rms, points))
    candidates.sort(key=lambda candidate: candidate[0])
    metrics = {'pnp_candidates': int(count)}
    if not candidates:
        return Estimate('pnp_no_solution', metrics=metrics)
    rms, points = candidates[0]
    metrics['pnp_reprojection_rms_px'] = rms
    if len(candidates) > 1:
        metrics['pnp_second_rms_px'] = candidates[1][0]
    if rms > max_rms_px:
        return Estimate('pnp_reprojection', metrics=metrics)
    return Estimate('ok', points, metrics)


def unit_rays(cv, pixels, camera):
    xy = cv.undistortPoints(np.asarray(pixels, dtype=float).reshape(-1, 1, 2), camera.k, camera.d)
    return np.column_stack((xy.reshape(-1, 2), np.ones(len(xy))))


def depth_grid_rays(cv, camera, stride):
    rows, cols = np.mgrid[:camera.height:stride, :camera.width:stride]
    return unit_rays(cv, np.column_stack((cols.ravel(), rows.ravel())), camera)


def inside_quad(xy, polygon):
    edges = np.roll(polygon, -1, axis=0) - polygon
    delta = xy[:, None, :] - polygon
    cross = edges[:, 0] * delta[:, :, 1] - edges[:, 1] * delta[:, :, 0]
    return np.all(cross >= 0, axis=1) | np.all(cross <= 0, axis=1)


def ransac_plane(points, threshold_m, rng):
    """Fit a dominant plane, then refine with orthogonal least squares."""
    examples = points[::max(1, (len(points) + 1023) // 1024)]
    triples = examples[rng.integers(0, len(examples), size=(64, 3))]
    normals = np.cross(triples[:, 1] - triples[:, 0], triples[:, 2] - triples[:, 0])
    norms = np.linalg.norm(normals, axis=1)
    valid = norms > 1e-10
    if not np.any(valid):
        return None
    normals = normals[valid] / norms[valid, None]
    offsets = np.einsum('ij,ij->i', normals, triples[valid, 0])
    distances = np.abs(np.einsum('ij,kj->ik', examples, normals) - offsets)
    counts = np.sum(distances <= threshold_m, axis=0)
    best = int(np.argmax(counts))
    inliers = np.abs(points @ normals[best] - offsets[best]) <= threshold_m
    for _ in range(2):
        if np.count_nonzero(inliers) < 3:
            return None
        support = points[inliers]
        centre = np.mean(support, axis=0)
        _, spread, axes = np.linalg.svd(support - centre, full_matrices=False)
        if spread[1] <= 1e-8:
            return None
        normal, offset = axes[-1], float(axes[-1] @ centre)
        residual = points @ normal - offset
        inliers = np.abs(residual) <= threshold_m
    return normal, offset, inliers, residual


def depth_estimate(cv, corner_rays, depth_m, depth_rays, color_from_depth, settings, rng):
    polygon = corner_rays[:, :2].copy()
    polygon += settings['inset_fraction'] * (polygon.mean(axis=0) - polygon)
    min_m, max_m = settings['min_mm'] / 1000, settings['max_mm'] / 1000
    valid = np.isfinite(depth_m) & (depth_m >= min_m) & (depth_m <= max_m)
    points = transform_points(color_from_depth, depth_rays[valid] * depth_m[valid, None])
    points = points[points[:, 2] > 0]
    xy = points[:, :2] / points[:, 2, None]
    box = np.all((xy >= polygon.min(axis=0)) & (xy <= polygon.max(axis=0)), axis=1)
    points, xy = points[box], xy[box]
    inside = inside_quad(xy, polygon)
    points, xy = points[inside], xy[inside]
    metrics = {'depth_points': len(points)}
    if len(points) < settings['min_inliers']:
        return Estimate('depth_few_points', metrics=metrics)
    fit = ransac_plane(points, settings['inlier_threshold_mm'] / 1000, rng)
    if fit is None:
        return Estimate('depth_plane', metrics=metrics)
    normal, offset, inliers, residual = fit
    count = int(np.count_nonzero(inliers))
    ratio = count / len(points)
    metrics.update(depth_inliers=count, depth_inlier_ratio=ratio)
    if count < settings['min_inliers'] or ratio < settings['min_inlier_ratio']:
        return Estimate('depth_few_inliers', metrics=metrics)
    metrics['depth_plane_rms_mm'] = float(1000 * np.sqrt(np.mean(residual[inliers] ** 2)))
    rectangle = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32)
    homography = cv.getPerspectiveTransform(polygon.astype(np.float32), rectangle)
    support_uv = cv.perspectiveTransform(xy[inliers].reshape(-1, 1, 2), homography).reshape(-1, 2)
    span = float(np.min(np.ptp(support_uv, axis=0)))
    metrics['depth_span_fraction'] = span
    if not np.isfinite(span) or span < settings['min_span_fraction']:
        return Estimate('depth_coverage', metrics=metrics)
    cos_scale = corner_rays @ normal
    if np.any(np.abs(cos_scale) < 1e-6):
        return Estimate('depth_intersection', metrics=metrics)
    corner_z = offset / cos_scale
    if not np.isfinite(corner_z).all() or np.any(corner_z <= 0):
        return Estimate('depth_intersection', metrics=metrics)
    return Estimate('ok', corner_rays * corner_z[:, None], metrics)
