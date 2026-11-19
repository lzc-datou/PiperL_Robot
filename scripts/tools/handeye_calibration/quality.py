"""Pure geometry and source-time checks used before accepting hand-eye samples."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def checked_transform(value):
    transform = np.asarray(value, dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError('位姿必须是有限的 4×4 矩阵。')
    rotation = transform[:3, :3]
    if (not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6)):
        raise ValueError('位姿旋转必须是有效的右手正交矩阵。')
    return transform


def check_observability(robot_matrices, min_span_deg=15.0, min_axis_ratio=0.1):
    """Reject insufficient rotational excitation; this is not an accuracy score."""
    if not 0 < min_span_deg < 180 or not 0 < min_axis_ratio <= 1:
        raise ValueError('旋转跨度阈值应在 (0,180) 度，轴比例阈值应在 (0,1]。')
    if len(robot_matrices) < 3:
        raise ValueError('至少需要三个姿态，且包含绕两个非平行轴的充分旋转。')
    rotations = [Rotation.from_matrix(checked_transform(t)[:3, :3]) for t in robot_matrices]
    vectors = np.array([(a.inv() * b).as_rotvec() for i, a in enumerate(rotations)
                        for b in rotations[i + 1:]])
    span = float(np.degrees(np.max(np.linalg.norm(vectors, axis=1))))
    singular = np.linalg.svd(vectors, compute_uv=False)
    ratio = float(singular[1] / singular[0]) if singular[0] > 1e-12 else 0.0
    if span < min_span_deg or ratio < min_axis_ratio:
        raise ValueError(
            f'姿态旋转不足或退化：最大相对旋转 {span:.2f}°（要求 ≥{min_span_deg:g}°），'
            f'第二/第一轴比例 {ratio:.3f}（要求 ≥{min_axis_ratio:g}）。'
            '请增加绕两个非平行轴的旋转；重复姿态、纯平移和单轴旋转不能完成标定。')
    return {'samples': len(rotations), 'pairwise_rotation_max_deg': span,
            'rotation_axis_singular_values': singular.tolist(), 'second_axis_ratio': ratio,
            'min_span_deg': min_span_deg, 'min_axis_ratio': min_axis_ratio,
            'note': 'Rotational excitation heuristic only; not calibration accuracy.'}


def _history(history):
    if not history:
        raise ValueError('尚未收到机器人反馈。')
    times = np.asarray([entry[0] for entry in history], dtype=float)
    transforms = np.asarray([entry[1] for entry in history], dtype=float)
    if not np.isfinite(times).all():
        raise ValueError('反馈源时间戳不是有限值。')
    if transforms.shape != (len(history), 4, 4) or not np.isfinite(transforms).all():
        raise ValueError('位姿必须是有限的 4×4 矩阵。')
    rotations = transforms[:, :3, :3]
    if (not np.allclose(transforms[:, 3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(np.swapaxes(rotations, 1, 2) @ rotations, np.eye(3), atol=1e-6)
            or not np.allclose(np.linalg.det(rotations), 1.0, atol=1e-6)):
        raise ValueError('位姿旋转必须是有效的右手正交矩阵。')
    unique = {}
    for timestamp, transform in zip(times.tolist(), transforms):
        if timestamp in unique and not np.allclose(unique[timestamp], transform, atol=1e-9):
            raise ValueError('同一反馈源时间戳包含不同位姿。')
        unique[timestamp] = transform
    return sorted(unique.items())


def interpolate_transform(history, target_time, max_gap_s=0.05):
    """Interpolate translation/rotation between source stamps, never extrapolate."""
    if not np.isfinite(target_time) or not 0 < max_gap_s <= 1:
        raise ValueError('插值时间或反馈间隔配置无效。')
    samples = _history(history)
    times = np.array([entry[0] for entry in samples])
    index = int(np.searchsorted(times, target_time))
    if index < len(times) and times[index] == target_time:
        timestamp, value = samples[index]
        return value.copy(), {'before_stamp_s': timestamp, 'after_stamp_s': timestamp,
                              'bracket_gap_s': 0.0, 'interpolation_fraction': 0.0}
    if index == 0 or index == len(times):
        raise ValueError('图像曝光时间尚未被机器人反馈包围，禁止外推。')
    before_time, before = samples[index - 1]
    after_time, after = samples[index]
    gap = after_time - before_time
    if gap > max_gap_s + 1e-9:
        raise ValueError(f'曝光时刻附近反馈间隔 {gap:.4f} s 超过 {max_gap_s:g} s。')
    fraction = (target_time - before_time) / gap
    value = np.eye(4)
    value[:3, 3] = before[:3, 3] * (1 - fraction) + after[:3, 3] * fraction
    value[:3, :3] = Slerp([before_time, after_time], Rotation.from_matrix(
        np.stack([before[:3, :3], after[:3, :3]])))(target_time).as_matrix()
    return value, {'before_stamp_s': before_time, 'after_stamp_s': after_time,
                   'bracket_gap_s': gap, 'interpolation_fraction': fraction}


def stationary_window(history, end_time, duration_s=1.0, max_translation_m=0.001,
                      max_rotation_deg=0.2, max_gap_s=0.1):
    """Require full source-time coverage and small change from the window start."""
    limits = [duration_s, max_translation_m, max_rotation_deg, max_gap_s]
    if not np.isfinite([end_time, *limits]).all() or not all(v > 0 for v in limits):
        raise ValueError('停稳检查参数必须为有限正数。')
    samples = _history(history)
    start_time = end_time - duration_s
    if samples[0][0] > start_time or samples[-1][0] < end_time:
        raise ValueError(f'尚未覆盖完整 {duration_s:g} 秒停稳窗口。')
    start, _ = interpolate_transform(samples, start_time, max_gap_s)
    end, _ = interpolate_transform(samples, end_time, max_gap_s)
    window = [(start_time, start), *[(t, value) for t, value in samples
                                  if start_time < t < end_time], (end_time, end)]
    max_gap = float(np.max(np.diff([t for t, _ in window])))
    if max_gap > max_gap_s + 1e-9:
        raise ValueError(f'停稳窗口反馈缺失，最大间隔 {max_gap:.4f} s。')
    values = np.array([value for _, value in window])
    translation = float(np.max(np.linalg.norm(values[:, :3, 3] - start[:3, 3], axis=1)))
    rotation = float(np.degrees(np.max((Rotation.from_matrix(start[:3, :3]).inv()
                                      * Rotation.from_matrix(values[:, :3, :3])).magnitude())))
    if translation > max_translation_m or rotation > max_rotation_deg:
        raise ValueError(f'机械臂尚未停稳：窗口内平移变化 {translation * 1000:.3f} mm，'
                         f'旋转变化 {rotation:.3f}°；要求 ≤{max_translation_m * 1000:g} mm、'
                         f'≤{max_rotation_deg:g}°。')
    return {'start_stamp_s': start_time, 'end_stamp_s': end_time,
            'duration_s': duration_s, 'unique_source_samples': len(window),
            'max_source_gap_s': max_gap, 'translation_from_start_mm': translation * 1000,
            'rotation_from_start_deg': rotation, 'max_translation_m': max_translation_m,
            'max_rotation_deg': max_rotation_deg}
