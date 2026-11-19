"""Choose an existing official TCP feedback topic without creating a control source."""
import math
import os
import re
import time
from collections.abc import Mapping


ROBOT_POSE_TOPICS = ('/feedback/tcp_pose', '/piper_servo/feedback/tcp_pose')
POSE_TYPE = 'geometry_msgs/msg/PoseStamped'
_ABSOLUTE_TOPIC = re.compile(r'/[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*')


def _explicit_topic(configured):
    if not isinstance(configured, str):
        raise ValueError('robot_pose_topic 必须是 auto 或绝对 ROS 话题名。')
    if configured != 'auto' and not _ABSOLUTE_TOPIC.fullmatch(configured):
        raise ValueError(f'robot_pose_topic={configured!r} 不是有效的绝对 ROS 话题名。')
    return configured


def _field(endpoint, name):
    return endpoint.get(name, '') if isinstance(endpoint, Mapping) else getattr(endpoint, name, '')


def _publisher_description(endpoint):
    namespace = _field(endpoint, 'node_namespace') or '/'
    name = _field(endpoint, 'node_name') or '<未知节点>'
    return f'{namespace.rstrip("/")}/{name} [{_field(endpoint, "topic_type") or "未知类型"}]'


def select_robot_pose_topic(configured, publishers):
    """Select from graph endpoint records; this function does not import or run ROS."""
    configured = _explicit_topic(configured)
    if configured != 'auto':
        return configured
    present = {topic: list(publishers.get(topic, ())) for topic in ROBOT_POSE_TOPICS
               if publishers.get(topic)}
    choices = '、'.join(ROBOT_POSE_TOPICS)
    if not present:
        raise ValueError(f'未发现机器人 TCP 位姿发布者；已检查 {choices}。'
                         '请确认本次使用的官方控制会话已启动；'
                         '若使用其他话题，请在 config/config.yaml 中明确填写 robot_pose_topic。')
    details = '；'.join(f'{topic}: ' + ', '.join(map(_publisher_description, endpoints))
                       for topic, endpoints in present.items())
    if any(_field(endpoint, 'topic_type') != POSE_TYPE
           for endpoints in present.values() for endpoint in endpoints):
        raise ValueError(f'机器人位姿话题类型不匹配，必须为 {POSE_TYPE}。发现：{details}。'
                         '请核对发布源，并在 config/config.yaml 中明确填写本次 robot_pose_topic。')
    if len(present) != 1:
        raise ValueError(f'发现多个机器人 TCP 位姿话题，无法自动选择：{details}。'
                         '请在 config/config.yaml 中明确填写本次控制会话的 robot_pose_topic，'
                         '不要混用两个机械臂反馈源。')
    topic, endpoints = next(iter(present.items()))
    if len(endpoints) != 1:
        raise ValueError(f'同一机器人位姿话题存在多个发布者，无法区分反馈：{details}。'
                         '请检查并保留本次使用的唯一反馈发布源，或将各源分配到不同话题后明确配置。')
    return topic


def resolve_robot_pose_topic(node, configured='auto', discovery_s=2.0):
    """Keep waiting for a unique TCP publisher; discovery_s is the poll window."""
    configured = _explicit_topic(configured)
    if configured != 'auto':
        node.get_logger().info(f'使用指定机器人位姿话题：{configured}')
        return configured
    if (isinstance(discovery_s, bool) or not isinstance(discovery_s, (int, float))
            or not math.isfinite(discovery_s) or discovery_s < 0):
        raise ValueError('discovery_s 必须是非负的有限秒数。')
    import rclpy

    domain = node.context.get_domain_id() if hasattr(node.context, 'get_domain_id') else os.environ.get('ROS_DOMAIN_ID', '0')
    logger = node.get_logger()
    logger.info(f'等待机器人 TCP 位姿反馈；当前 ROS Domain {domain}，Ctrl+C 取消。'
                f'检查话题：{"、".join(ROBOT_POSE_TOPICS)}')
    started = time.monotonic()
    next_check = started + discovery_s
    last_report, last_reason = started, None
    while rclpy.ok():
        now = time.monotonic()
        if now < next_check:
            rclpy.spin_once(node, timeout_sec=min(.05, next_check - now))
            continue
        publishers = {topic: node.get_publishers_info_by_topic(topic) for topic in ROBOT_POSE_TOPICS}
        try:
            topic = select_robot_pose_topic(configured, publishers)
        except ValueError as exc:
            reason = str(exc)
            if reason != last_reason or now - last_report >= 5.0:
                logger.warning(f'继续等待，已等待 {now - started:.0f} 秒：{reason}')
                last_report, last_reason = now, reason
            # Even a zero discovery window must yield so shutdown and callbacks
            # can be handled; graph state can recover while this process waits.
            next_check = now + max(discovery_s, .05)
            continue
        endpoint = publishers[topic][0]
        logger.info(f'自动选择机器人位姿话题：{topic}；发布节点：{_publisher_description(endpoint)}')
        return topic
    raise SystemExit(0)
