"""Fit one eye-in-hand transform across explicitly grouped fixed-board sessions."""
import argparse
from datetime import datetime
import hashlib
import itertools
import json
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np
import scipy
from scipy.spatial.transform import Rotation

from quality import checked_transform
from verify import GEOMETRY_FIELDS, board_poses, capture_audit, compatible, dataset, input_hashes, spread


def solve_grouped(groups):
    """Solve A_i X B_i = Y_g, with one X and a different fixed Y per group.

    Park rotation uses only within-group relative motions. Translation is an
    equal-observation-weight absolute least squares fit, with one board-position
    nuisance variable per group. No rotation/translation unit weighting is used.
    """
    if not groups or any(len(group) < 2 for group in groups):
        raise ValueError('每个固定板位置至少需要两组样本。')
    parsed = [[(checked_transform(s['base_T_end']), checked_transform(s['camera_T_marker']))
               for s in group] for group in groups]
    alpha, beta, pair_counts = [], [], []
    for group in parsed:
        count = 0
        for (ai, bi), (aj, bj) in itertools.combinations(group, 2):
            # inv(Aj) Ai X = X Bj inv(Bi). Never pair different board locations.
            a, b = np.linalg.inv(aj) @ ai, bj @ np.linalg.inv(bi)
            alpha.append(Rotation.from_matrix(a[:3, :3]).as_rotvec())
            beta.append(Rotation.from_matrix(b[:3, :3]).as_rotvec())
            count += 1
        pair_counts.append(count)
    alpha, beta = np.asarray(alpha), np.asarray(beta)
    excitation = []
    for label, vectors in (('robot', alpha), ('camera', beta)):
        singular = np.linalg.svd(vectors, compute_uv=False)
        ratio = float(singular[1] / singular[0]) if len(singular) >= 2 and singular[0] > 1e-12 else 0.
        span = float(np.degrees(np.linalg.norm(vectors, axis=1).max()))
        if span < 15 or ratio < 0.1:
            raise ValueError(f'{label} 组内旋转不足或退化：跨度 {span:.3f}°，第二/第一轴比例 {ratio:.4f}。')
        excitation.append({'source': label, 'max_rotation_deg': span,
                           'singular_values': singular.tolist(), 'second_axis_ratio': ratio})
    u, singular, vt = np.linalg.svd(alpha.T @ beta)
    if singular[1] <= singular[0] * 1e-10:
        raise ValueError('旋转对应关系退化，无法确定共享相机旋转。')
    rotation = u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt
    lhs, rhs = [], []
    for index, group in enumerate(parsed):
        for a, b in group:
            row = np.zeros((3, 3 + 3 * len(groups)))
            row[:, :3] = a[:3, :3]
            row[:, 3 + 3 * index:6 + 3 * index] = -np.eye(3)
            lhs.append(row)
            rhs.append(-a[:3, 3] - a[:3, :3] @ rotation @ b[:3, 3])
    lhs, rhs = np.concatenate(lhs), np.concatenate(rhs)
    solution, _, rank, translation_singular = np.linalg.lstsq(lhs, rhs, rcond=None)
    if rank != lhs.shape[1]:
        raise ValueError(f'平移求解退化：秩 {rank}，需要 {lhs.shape[1]}。')
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = rotation, solution[:3]
    checked_transform(transform)
    return transform, {
        'method': 'within_group_Park_rotation_and_equal_observation_absolute_translation_LS',
        'samples': sum(map(len, groups)), 'groups': len(groups),
        'within_group_pairs': pair_counts, 'cross_group_pairs': 0,
        'rotation_cross_covariance_singular_values': singular.tolist(),
        'excitation': excitation, 'translation_rank': int(rank),
        'translation_unknowns': lhs.shape[1],
        'translation_singular_values': translation_singular.tolist(),
        'translation_condition_number': float(translation_singular[0] / translation_singular[-1]),
        'board_positions_m': solution[3:].reshape(-1, 3).tolist(),
        'rotation_weighting': 'equal within-group relative-pair weight; pairs are correlated',
        'translation_weighting': 'equal weight for each original observation',
    }


def load_groups(paths):
    groups, sources, seen = [], [], set()
    reference = None
    for group_number, directories in enumerate(paths, 1):
        samples = []
        for directory in directories:
            data = dataset(directory)
            if data['folder'] in seen:
                raise ValueError(f'同一目录不能重复参与：{data["folder"]}')
            seen.add(data['folder'])
            if reference is None:
                reference = data
                if data['metadata'].get('mode') != 'eye_in_hand':
                    raise ValueError('分组求解仅适用于 eye_in_hand。')
            else:
                compatible(reference, data)
            source = {'directory': str(data['folder']), 'fixed_board_group': group_number,
                      'samples': len(data['samples']), 'input_sha256': input_hashes(data),
                      'capture_audit': capture_audit(data)}
            sources.append(source)
            for number, sample in enumerate(data['samples'], 1):
                # Keep source records; original image paths remain source-relative.
                samples.append({**sample, 'combined_provenance': {
                    'fixed_board_group': group_number, 'source_directory': str(data['folder']),
                    'source_sample_number': number}})
        groups.append(samples)
    return groups, sources, reference['metadata']


def markdown(report):
    lines = ['# 全部采样点分组手眼标定', '',
             f'生成时间：{report["created_at"]}', '',
             f'共 {report["diagnostics"]["samples"]} 个样本，按棋盘格固定位置分为 '
             f'{len(report["groups"])} 组，共享一个 `end_T_camera`。', '',
             report['context_note'], '',
             '## 输入', '', '| 固定板位置组 | 采样目录 | 样本数 |', '| --- | --- | ---: |']
    for source in report['sources']:
        lines.append(f'| {source["fixed_board_group"]} | `{source["directory"]}` | {source["samples"]} |')
    lines += ['', '仅使用各源目录 samples.json 中已保存的样本，没有删除点或按误差筛点。'
              '失败会话中的空样本列表不参与计算。输入文件 SHA256 和可用采样历史重放见 consistency.json。', '',
              '## 方法与方向', '',
              '`A_i X B_i = Y_g`，其中 `A_i = base_T_end`，`B_i = camera_T_board`，'
              '`X = end_T_camera`，每个固定板位置有自己的 `Y_g = base_T_board`。', '',
              '组内构造 `a = inv(A_j) A_i`、`b = B_j inv(B_i)`，用 `log(R_a) = R_X log(R_b)` '
              '的 SVD 正交拟合求旋转。旋转配对方向对照 '
              '[OpenCV 4.6 Park 实现](https://github.com/opencv/opencv/blob/4.6.0/modules/calib3d/src/calibration_handeye.cpp#L337-L367)。'
              '只配对同组姿态，跨组配对数为 0。', '',
              '固定旋转后，对所有原始样本等权求解 '
              '`R_Ai t_X - t_Yg = -t_Ai - R_Ai R_X t_Bi`。'
              '平移采用绝对观测的最小二乘，分别估计各组板位置。该步骤是本项目的分组扩展，'
              '不同于直接调用单板 OpenCV 求解器。旋转按组内相对配对等权，配对不是独立观测。', '',
              f'旋转配对数：{report["diagnostics"]["within_group_pairs"]}；平移系统秩 '
              f'{report["diagnostics"]["translation_rank"]}/{report["diagnostics"]["translation_unknowns"]}，'
              f'条件数 {report["diagnostics"]["translation_condition_number"]:.4f}。', '',
              f'输出变换：`{report["transform_name"]}`，平移单位为米。', '',
              '`p_tcp = R_tcp_camera @ p_camera + t_tcp_camera`', '',
              '```json', json.dumps(report['transform'], indent=2), '```', '',
              '## 参与拟合样本的一致性', '',
              '| 固定板位置组 | 样本数 | 位置 RMS / mm | 最大位置偏差 / mm | 方向 RMS / ° |',
              '| --- | ---: | ---: | ---: | ---: |']
    for group in report['groups']:
        s = group['spread']
        lines.append(f'| {group["fixed_board_group"]} | {s["samples"]} | '
                     f'{s["position_rms_mm"]:.4f} | {s["position_max_mm"]:.4f} | {s["rotation_rms_deg"]:.4f} |')
    lines += ['', '位置偏差分别相对每组反算板位置的自身均值，方向偏差相对每组平均旋转。'
              '不能把板移动前后的坐标混在一起计算散布。', '',
              '全部点都参与了本次拟合，包括原来的 9 组和 7 组验证点。上述指标是训练一致性，'
              '不能证明独立精度，也不能从中断言比旧矩阵更准。现有数据仍有厘米量级散布，'
              '需要另采未用于本次计算的新姿态作独立验证。', '',
              '旧数据没有完整原图和源时间历史，不能对这部分重新检测角点或重放停稳检查。'
              '相机安装和 TCP 定义未变、同组内棋盘格固定等物理条件由操作者确认。', '',
              '矩阵保存在 T_end_camera.json；完整诊断、各组及各源目录逐点偏差见 consistency.json；'
              '输入来源及配置见 session.json。samples.json 保留全部样本及源目录标记，'
              '其中原图相对路径以 combined_provenance.source_directory 为根目录。', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--group', action='append', nargs='+', required=True,
                        help='一次 --group 后填写同一个固定板位置的一个或多个采样目录；板移动后另起 --group')
    parser.add_argument('--output', type=Path, help='新输出目录，必须尚不存在')
    parser.add_argument('--context-note', required=True, help='相机/TCP 未变及棋盘格固定分组的操作者确认记录')
    args = parser.parse_args()
    groups, sources, metadata = load_groups(args.group)
    transform, diagnostics = solve_grouped(groups)
    now = datetime.now(ZoneInfo('Asia/Shanghai'))
    output = args.output or Path(__file__).resolve().parents[3] / 'outputs/handeye' / now.strftime('%Y%m%d_%H%M%S_%f_combined')
    output = output.resolve()
    report = {'created_at': now.isoformat(), 'context_note': args.context_note,
              'transform_name': metadata['transform'], 'transform': transform.tolist(),
              'diagnostics': diagnostics, 'sources': sources, 'groups': [],
              'assessment': 'training_consistency_only; all samples used in fitting',
              'environment': {'numpy': np.__version__, 'scipy': scipy.__version__}}
    for index, samples in enumerate(groups, 1):
        poses = board_poses(samples, transform)
        summary = spread(poses)
        board = np.eye(4)
        board[:3, :3] = Rotation.from_quat(summary['mean_orientation_xyzw']).as_matrix()
        board[:3, 3] = summary['mean_position_m']
        report['groups'].append({'fixed_board_group': index, 'spread': summary,
                                 'base_T_board_estimate': board.tolist()})
    # Per-source summaries share X but use their own mean, for comparison only.
    for source in sources:
        samples = [sample for group in groups for sample in group
                   if sample['combined_provenance']['source_directory'] == source['directory']]
        source['training_spread'] = spread(board_poses(samples, transform))
    source_path = Path(__file__)
    session = {key: metadata.get(key) for key in GEOMETRY_FIELDS}
    session.update({'solver': diagnostics['method'], 'sources': sources,
                    'fixed_board_groups': len(groups),
                    'context_note': args.context_note, 'created_at': now.isoformat(),
                    'tool_sources_sha256': {name: hashlib.sha256((source_path.parent / name).read_bytes()).hexdigest()
                                            for name in ('combined.py', 'verify.py', 'quality.py')}})
    # No output is created until all input checks and fitting have succeeded.
    output.mkdir(parents=True, exist_ok=False)
    for name, value in (('T_end_camera.json', transform.tolist()), ('session.json', session),
                        ('samples.json', [sample for group in groups for sample in group]),
                        ('consistency.json', report)):
        (output / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    (output / 'REPORT.md').write_text(markdown(report))
    print(f'结果目录: {output}')
    print(json.dumps(transform.tolist(), indent=2))
    for group in report['groups']:
        s = group['spread']
        print(f'组 {group["fixed_board_group"]}，{s["samples"]} 点：训练位置 RMS '
              f'{s["position_rms_mm"]:.4f} mm，方向 RMS {s["rotation_rms_deg"]:.4f}°')


if __name__ == '__main__':
    main()
