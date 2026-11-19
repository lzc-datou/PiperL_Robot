"""Evaluate a saved hand-eye transform on held-out poses without ROS or refitting."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from quality import checked_transform, check_observability, interpolate_transform, stationary_window


GEOMETRY_FIELDS = ('mode', 'target', 'chessboard', 'base_frame', 'end_frame',
                   'camera_frame', 'color_profile', 'convention', 'transform', 'marker_id', 'marker_size_m')


def dataset(path, require_result=False):
    folder = Path(path).resolve()
    samples = json.loads((folder / 'samples.json').read_text())
    metadata = json.loads((folder / 'session.json').read_text())
    if not isinstance(samples, list) or len(samples) < 2:
        raise ValueError(f'{folder} 至少需要两组样本。')
    for sample in samples:
        checked_transform(sample['base_T_end'])
        checked_transform(sample['camera_T_marker'])
    result = checked_transform(json.loads((folder / 'T_end_camera.json').read_text())) if require_result else None
    return {'folder': folder, 'samples': samples, 'metadata': metadata, 'transform': result}


def compatible(training, validation):
    different = {key: {'calibration': training['metadata'].get(key),
                       'validation': validation['metadata'].get(key)}
                 for key in GEOMETRY_FIELDS
                 if training['metadata'].get(key) != validation['metadata'].get(key)}
    if different:
        raise ValueError(f'几何配置不一致，不能直接比较：{different}')


def board_poses(samples, transform):
    return np.array([np.asarray(sample['base_T_end']) @ transform @ np.asarray(sample['camera_T_marker'])
                     for sample in samples])


def spread(poses):
    positions = poses[:, :3, 3]
    mean = positions.mean(axis=0)
    distances = np.linalg.norm(positions - mean, axis=1) * 1000
    pairwise = np.linalg.norm(positions[:, None] - positions[None, :], axis=-1) * 1000
    rotations = Rotation.from_matrix(poses[:, :3, :3])
    mean_rotation = rotations.mean()
    angles = np.degrees((mean_rotation.inv() * rotations).magnitude())
    pairwise_angles = [np.degrees((a.inv() * b).magnitude())
                       for i, a in enumerate(rotations) for b in rotations[i + 1:]]
    return {'samples': len(poses), 'mean_position_m': mean.tolist(),
            'mean_orientation_xyzw': mean_rotation.as_quat().tolist(),
            'position_rms_mm': float(np.sqrt(np.mean(distances ** 2))),
            'position_max_mm': float(distances.max()),
            'position_pairwise_max_mm': float(pairwise.max()),
            'rotation_rms_deg': float(np.sqrt(np.mean(angles ** 2))),
            'rotation_max_deg': float(angles.max()),
            'rotation_pairwise_max_deg': float(max(pairwise_angles)),
            'per_sample': [{'sample': i + 1, 'board_position_m': positions[i].tolist(),
                            'position_deviation_mm': float(distance),
                            'rotation_deviation_deg': float(angles[i])}
                           for i, distance in enumerate(distances)]}


def input_hashes(data):
    names = ['session.json', 'samples.json']
    if data['transform'] is not None:
        names.append('T_end_camera.json')
    return {name: hashlib.sha256((data['folder'] / name).read_bytes()).hexdigest() for name in names}


def duplicate_pairs(training, validation):
    return [i + 1 for i, sample in enumerate(validation['samples'])
            if any(np.allclose(sample['base_T_end'], other['base_T_end'], atol=1e-12, rtol=0)
                   and np.allclose(sample['camera_T_marker'], other['camera_T_marker'], atol=1e-12, rtol=0)
                   for other in training['samples'])]


def capture_audit(data, require_observability=False):
    """Replay source history, rather than trusting saved gate summaries alone."""
    settings = data['metadata'].get('capture')
    if not settings or any('robot_history' not in sample.get('source', {}) for sample in data['samples']):
        return {'available': False, 'note': 'No complete source history in this dataset.'}
    maximum_difference, static_translations, static_rotations, bracket_gaps = [], [], [], []
    exact_headers, images = 0, 0
    for sample in data['samples']:
        source = sample['source']
        exposure = source['image']['sec'] + source['image']['nanosec'] * 1e-9
        history = [(item['header']['sec'] + item['header']['nanosec'] * 1e-9,
                    item['base_T_end']) for item in source['robot_history']]
        window = stationary_window(history, exposure, duration_s=settings['duration_s'],
                                   max_translation_m=settings['max_translation_m'],
                                   max_rotation_deg=settings['max_rotation_deg'],
                                   max_gap_s=settings['max_static_gap_s'])
        interpolated, pairing = interpolate_transform(history, exposure, max_gap_s=settings['max_robot_gap_s'])
        maximum_difference.append(float(np.max(np.abs(interpolated - sample['base_T_end']))))
        static_translations.append(window['translation_from_start_mm'])
        static_rotations.append(window['rotation_from_start_deg'])
        bracket_gaps.append(pairing['bracket_gap_s'] * 1000)
        exact_headers += source['marker'] == source['image'] == source['camera_info']
        images += (data['folder'] / sample['image_path']).is_file()
    try:
        observability = check_observability([sample['base_T_end'] for sample in data['samples']],
                                           min_span_deg=settings['min_span_deg'],
                                           min_axis_ratio=settings['min_axis_ratio'])
    except ValueError as error:
        if require_observability:
            raise
        # Validation does not solve for X, so it need not meet a solver's
        # excitation gate. Record limited coverage without discarding samples.
        observability = {'sufficient_for_calibration': False, 'reason': str(error)}
    return {'available': True, 'history_replayed_samples': len(data['samples']),
            'max_interpolation_matrix_element_difference': max(maximum_difference),
            'max_stationary_translation_mm': max(static_translations),
            'max_stationary_rotation_deg': max(static_rotations),
            'max_bracket_gap_ms': max(bracket_gaps), 'matched_camera_headers': exact_headers,
            'images_present': images, 'observability': observability,
            'note': 'Software input checks only; not absolute calibration accuracy.'}


def reference_deviations(poses, reference):
    positions = poses[:, :3, 3]
    distances = np.linalg.norm(positions - reference['mean_position_m'], axis=1) * 1000
    angles = np.degrees((Rotation.from_quat(reference['mean_orientation_xyzw']).inv()
                         * Rotation.from_matrix(poses[:, :3, :3])).magnitude())
    return {'position_rms_mm': float(np.sqrt(np.mean(distances ** 2))),
            'position_max_mm': float(distances.max()),
            'rotation_rms_deg': float(np.sqrt(np.mean(angles ** 2))),
            'rotation_max_deg': float(angles.max()),
            'mean_position_shift_mm': float(np.linalg.norm(positions.mean(axis=0)
                                                           - reference['mean_position_m']) * 1000),
            'per_sample': [{'sample': i + 1, 'position_deviation_mm': float(distance),
                            'rotation_deviation_deg': float(angles[i])}
                           for i, distance in enumerate(distances)]}


def training_spread(data, transform):
    """Center moved-board groups separately, so board movement is not residual."""
    if data['metadata'].get('fixed_board_groups', 1) == 1:
        return spread(board_poses(data['samples'], transform))
    groups = {}
    for sample in data['samples']:
        group = sample['combined_provenance']['fixed_board_group']
        groups.setdefault(group, []).append(sample)
    summaries = {str(group): spread(board_poses(samples, transform)) for group, samples in groups.items()}
    count = len(data['samples'])
    result = {'samples': count, 'fixed_board_groups': summaries,
              'reference': 'separate mean per fixed-board group; not a common board pose'}
    for field in ('position', 'rotation'):
        unit = 'mm' if field == 'position' else 'deg'
        rms, maximum = f'{field}_rms_{unit}', f'{field}_max_{unit}'
        result[rms] = float(np.sqrt(sum(s['samples'] * s[rms] ** 2 for s in summaries.values()) / count))
        result[maximum] = max(s[maximum] for s in summaries.values())
    return result


def evaluate(calibration, validation, reference_data=None):
    compatible(calibration, validation)
    if validation['metadata'].get('fixed_board_groups', 1) != 1:
        raise ValueError('验证集含多个棋盘格位置，请逐个固定板位置单独验证。')
    if reference_data is not None and reference_data['metadata'].get('fixed_board_groups', 1) != 1:
        raise ValueError('联合标定包含多个棋盘格位置，不能使用 --same-board-pose 作为单一参考；'
                         '请去掉该选项，验证新采样自身的一致性。')
    poses = board_poses(validation['samples'], calibration['transform'])
    result = {'directory': str(calibration['folder']), 'input_sha256': input_hashes(calibration),
              'transform': calibration['transform'].tolist(),
              'training': training_spread(calibration, calibration['transform']),
              'validation': spread(poses),
              'overlapping_validation_sample_numbers': duplicate_pairs(calibration, validation)}
    if reference_data is not None:
        reference = spread(board_poses(reference_data['samples'], calibration['transform']))
        result['validation_to_reference'] = {
            'reference_directory': str(reference_data['folder']),
            'reference': reference, **reference_deviations(poses, reference),
            'note': 'Reference is estimated from saved training poses, not external ground truth.'}
    return result


def markdown(report):
    current = report['current']
    baseline = report.get('baseline')
    count = report['validation_samples']
    rows = [('位置 RMS 散布', 'position_rms_mm', 'mm'),
            ('最大位置偏差', 'position_max_mm', 'mm'),
            ('最大两两位置差', 'position_pairwise_max_mm', 'mm'),
            ('方向 RMS 散布', 'rotation_rms_deg', '°'),
            ('最大方向偏差', 'rotation_max_deg', '°'),
            ('最大两两方向差', 'rotation_pairwise_max_deg', '°')]
    lines = ['# 手眼标定独立验证', '', f'新标定：`{current["directory"]}`',
             f'验证数据：`{report["validation_directory"]}`，共 {report["validation_samples"]} 组。', '',
             '使用保存的手眼矩阵，不重新拟合，不删除样本，不改尺寸或尺度。', '',
             '`base_T_board = base_T_end @ end_T_camera @ camera_T_board`', '',
             f'位置偏差相对这 {count} 组反算位置的自身均值，方向偏差相对其平均旋转。'
             '新旧矩阵都在同一验证集上使用这一口径。只有明确确认标定板位置未变时，'
             '另提供相对训练参考位置的检查。', '',
             f'适用前提：相机安装与 TCP 未改变，棋盘格在这 {count} 组内部保持固定；不同会话之间允许移动棋盘格。',
             report.get('context_note', ''), '',
             '| 指标 | 旧矩阵 | 新矩阵 | 改善 |', '| --- | ---: | ---: | ---: |']
    for label, key, unit in rows:
        new_value = current['validation'][key]
        old_value = baseline['validation'][key] if baseline else None
        old_text = f'{old_value:.4f} {unit}' if old_value is not None else '—'
        change = report['improvement_percent'].get(key)
        change_text = f'{change:.2f}%' if change is not None else '—'
        lines.append(f'| {label} | {old_text} | {new_value:.4f} {unit} | {change_text} |')
    lines += ['', '## 逐组位置偏差', '', '| 样本 | 旧矩阵 / mm | 新矩阵 / mm |', '| --- | ---: | ---: |']
    for index, item in enumerate(current['validation']['per_sample']):
        old = f'{baseline["validation"]["per_sample"][index]["position_deviation_mm"]:.4f}' if baseline else '—'
        lines.append(f'| {index + 1} | {old} | {item["position_deviation_mm"]:.4f} |')
    if 'validation_to_reference' in current:
        reference = current['validation_to_reference']
        lines += ['', '## 相对本次训练参考的偏差', '',
                  '操作者确认棋盘格在标定与验证之间未移动。参考位置和方向由本次训练样本反算后的均值估计，不是外部真值。', '',
                  f'- 位置 RMS 偏差：{reference["position_rms_mm"]:.4f} mm；最大：{reference["position_max_mm"]:.4f} mm。',
                  f'- 平均位置偏移：{reference["mean_position_shift_mm"]:.4f} mm。',
                  f'- 方向 RMS 偏差：{reference["rotation_rms_deg"]:.4f}°；最大：{reference["rotation_max_deg"]:.4f}°。', '',
                  '| 样本 | 相对验证均值 / mm | 相对训练参考 / mm |', '| --- | ---: | ---: |']
        for own, item in zip(current['validation']['per_sample'], reference['per_sample']):
            lines.append(f'| {item["sample"]} | {own["position_deviation_mm"]:.4f} | {item["position_deviation_mm"]:.4f} |')
    lines += ['', '## 本次训练与归档检查', '',
              f'- 新训练集 {current["training"]["samples"]} 组，训练位置 RMS 散布 '
              f'{current["training"]["position_rms_mm"]:.4f} mm，最大 '
              f'{current["training"]["position_max_mm"]:.4f} mm。训练集与旧版训练姿态不同，不直接用训练 RMS 判断改善。',
              f'- 与验证数据完全重复的矩阵对：{current["overlapping_validation_sample_numbers"]}。']
    if 'fixed_board_groups' in current['training']:
        lines.append('- 联合训练集按棋盘格固定位置分别去均值后汇总残差，棋盘格移动不计入散布。')
    audit = report['capture_audit']
    if audit['available']:
        lines += [f'- 按源反馈历史重放 {audit["history_replayed_samples"]} 组停稳与插值检查均通过。'
                  f'机器人矩阵最大元素差 {audit["max_interpolation_matrix_element_difference"]:.3g}。',
                  f'- 窗口内最大平移 {audit["max_stationary_translation_mm"]:.4f} mm，'
                  f'最大转角 {audit["max_stationary_rotation_deg"]:.4f}°，'
                  f'最大插值反馈间隔 {audit["max_bracket_gap_ms"]:.4f} ms。',
                  f'- 原图存在 {audit["images_present"]} 张；图像/内参/检测的精确时间戳与 frame '
                  f'逐组相同的样本有 {audit["matched_camera_headers"]} 组。']
    validation_audit = report['validation_capture_audit']
    if validation_audit['available']:
        lines += [f'- 本次验证组也按源历史重放 {validation_audit["history_replayed_samples"]} 组检查：'
                  f'最大平移 {validation_audit["max_stationary_translation_mm"]:.4f} mm、'
                  f'最大转角 {validation_audit["max_stationary_rotation_deg"]:.4f}°、'
                  f'最大插值间隔 {validation_audit["max_bracket_gap_ms"]:.4f} ms。',
                  f'- 验证原图存在 {validation_audit["images_present"]} 张，精确相机消息头匹配 '
                  f'{validation_audit["matched_camera_headers"]} 组；机器人矩阵最大重建元素差 '
                  f'{validation_audit["max_interpolation_matrix_element_difference"]:.3g}。']
    if baseline:
        lines.append(f'- 新旧外参平移向量差的范数 {report["transform_change"]["translation_vector_difference_mm"]:.4f} mm，'
                     f'旋转差 {report["transform_change"]["rotation_difference_deg"]:.4f}°。')
    rms = current['validation']['position_rms_mm']
    comparison = ''
    if baseline:
        change = report['improvement_percent'].get('position_rms_mm')
        if change is not None:
            comparison = f'同一验证集的位置 RMS 相比旧矩阵{"下降" if change >= 0 else "上升"} {abs(change):.2f}%。'
    scale_note = '当前仍是厘米级散布。' if rms >= 10 else ''
    lines += ['', '## RMS 散布与实际精度', '',
              '每个验证姿态都得到一个棋盘格在基座坐标系中的估计位置 `p_i`，取均值 `p_mean` 后计算：', '',
              '`RMS = sqrt(sum(||p_i - p_mean||²) / N)`', '',
              '这是三维位置到均值的距离的均方根，不是单轴误差，也不是角点的像素重投影 RMS。'
              '棋盘格固定时，这些位置本应相同；散布越大，说明换机械臂姿态后估计位置越容易漂移，'
              '可能使相机定位后给出的抓取点随姿态改变。', '',
              '它包含视觉板位姿、机械臂反馈及手眼外参的综合影响，不能只归因于手眼矩阵。'
              '均值也不是外部真值：若所有估计位置都共同偏了 20 mm，散布仍可很小。'
              '因此不能把 RMS 直接当成绝对抓取误差、机器人重复定位精度、最大误差或置信区间。', '',
              comparison + f'新矩阵的 RMS 散布为 {rms:.2f} mm。' + scale_note +
              '没有外部真值支持绝对定位精度结论，也没有设定应用验收阈值，因此不自动标为合格。', '',
              '已有报告若使用训练集的板位置作为参考，其数值与围绕验证集自身均值的散布口径不同，不能交叉比较。', '',
              ('本次验证有原图与源时间戳，可复盘其输入质量。后续精度验收仍需使用已知位置或尺寸的外部目标。'
               if validation_audit['available'] else
               '验证数据没有完整源历史，无法完整复盘其输入质量。建议另采 5–10 组有原始记录的新姿态，仅用于验证。'), '',
              '计算与输入 SHA256 记录在同名 JSON 文件；原结果、原样本和旧报告未修改。', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration', required=True, type=Path)
    parser.add_argument('--validation', required=True, type=Path)
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--context-note', default='', help='记录操作者确认的实际采样条件。')
    parser.add_argument('--same-board-pose', action='store_true',
                        help='操作者确认棋盘格在本次标定和验证之间未移动，增加相对训练参考的检查。')
    args = parser.parse_args()
    calibration = dataset(args.calibration, require_result=True)
    validation = dataset(args.validation)
    reference_data = calibration if args.same_board_pose else None
    current = evaluate(calibration, validation, reference_data)
    report = {'method': 'Fixed X; validation dispersion around its own mean; no refitting.',
              'validation_directory': str(validation['folder']),
              'validation_samples': len(validation['samples']), 'validation_input_sha256': input_hashes(validation),
              'physical_assumptions': {'camera_mount_and_tcp_unchanged': True,
                                       'board_fixed_within_validation_set': True,
                                       'board_position_may_differ_between_sessions': True},
              'current': current, 'improvement_percent': {},
              'capture_audit': capture_audit(calibration, require_observability=True),
              'validation_capture_audit': capture_audit(validation),
              'same_board_pose_as_calibration_confirmed': args.same_board_pose,
              'geometry_metadata_match': True,
              'context_note': args.context_note,
              'source_topic_note': 'Topic namespace differences do not establish a geometry difference.',
              'note': 'Pose consistency only; not absolute accuracy or depth validation.'}
    if args.baseline:
        baseline = dataset(args.baseline, require_result=True)
        report['baseline'] = evaluate(baseline, validation, reference_data)
        report['improvement_percent'] = {key: 100 * (1 - current['validation'][key] / value)
                                         for key, value in report['baseline']['validation'].items()
                                         if key.endswith(('_mm', '_deg')) and isinstance(value, float) and value > 0}
        report['transform_change'] = {
            'translation_vector_difference_mm': float(np.linalg.norm(
                calibration['transform'][:3, 3] - baseline['transform'][:3, 3]) * 1000),
            'rotation_difference_deg': float(np.degrees(Rotation.from_matrix(
                baseline['transform'][:3, :3].T @ calibration['transform'][:3, :3]).magnitude()))}
    prefix = calibration['folder'] / f'verification_against_{validation["folder"].name}'
    json_path, md_path = prefix.with_suffix('.json'), prefix.with_suffix('.md')
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    md_path.write_text(markdown(report))
    print(md_path)
    print(json.dumps(current['validation'], ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
