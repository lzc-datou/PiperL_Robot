#!/usr/bin/env python3
"""Passively record an already-running gamepad stack; never request motion."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time
from zoneinfo import ZoneInfo


def finite(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: finite(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite(item) for item in value]
    return value


def stamp(value):
    return value.sec + value.nanosec * 1e-9


def normalize(message, kind):
    if kind == "status":
        try:
            return finite(json.loads(message.data))
        except (ValueError, TypeError):
            return {"invalid_json": True}
    data = {"stamp_s": stamp(message.header.stamp)}
    if kind == "input":
        data.update({key: getattr(message, key) for key in
                     ("generation", "sequence", "connected", "link_verified", "mapping_verified", "lb", "b")})
        data.update(axes=list(message.axes), dpad=list(message.dpad))
    elif kind == "trajectory":
        data.update(joint_names=list(message.joint_names), points=[{
            "positions": list(point.positions), "velocities": list(point.velocities),
            "accelerations": list(point.accelerations), "time_from_start_s": stamp(point.time_from_start)
        } for point in message.points])
    elif kind == "jog":
        data.update(joint_names=list(message.joint_names), velocities=list(message.velocities),
                    duration_s=message.duration)
    else:
        data.update(joint_names=list(message.name), positions=list(message.position),
                    velocities=list(message.velocity))
    return finite(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10., help="采样秒数，默认 10，最大 60")
    parser.add_argument("--output-dir", type=Path, help="输出目录，默认 outputs/gamepad_control/traces/上海时间")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 60:
        parser.error("--duration 必须大于 0 且不超过 60 秒")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from control_msgs.msg import JointJog
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectory
    from piper_servo_tracking.msg import GamepadInput

    topics = {
        "gamepad/input": (GamepadInput, "input"),
        "servo_node/delta_joint_cmds": (JointJog, "jog"),
        "servo/trajectory": (JointTrajectory, "trajectory"),
        "commanded_joint_states": (JointState, "joints"),
        "control/joint_states": (JointState, "joints"),
        "feedback/joint_states": (JointState, "joints"),
        "gamepad/backend_status": (String, "status"), "guard/status": (String, "status"),
    }
    started_at = datetime.now(ZoneInfo("Asia/Shanghai"))
    output = args.output_dir or (Path(__file__).resolve().parents[3] / "outputs/gamepad_control/traces" /
                                 started_at.strftime("%Y%m%d_%H%M%S_%f"))
    output.mkdir(parents=True, exist_ok=False)
    stream = (output / "trace.jsonl").open("w", encoding="utf-8")
    stats = {"/piper_servo/" + topic: {"count": 0, "interval_sum_s": 0., "interval_max_s": 0.}
             for topic in topics}
    previous, last_seen, maxima = {}, {}, {}
    active_samples = active_jogs = 0
    backend_state, node, reason, started = "unknown", None, "duration_elapsed", time.monotonic()

    def receive(message, topic, kind):
        nonlocal active_samples, active_jogs, backend_state
        now, data = time.monotonic(), normalize(message, kind)
        item = stats[topic]
        if topic in last_seen:
            interval = now - last_seen[topic]
            item["interval_sum_s"] += interval
            item["interval_max_s"] = max(item["interval_max_s"], interval)
        last_seen[topic], item["count"] = now, item["count"] + 1
        if topic.endswith("/backend_status") and isinstance(data, dict):
            backend_state = data.get("state", "unknown")
            active_samples += int(backend_state == "ACTIVE")
        if kind == "jog":
            active_jogs += int(backend_state == "ACTIVE")
        field = "velocities" if kind == "jog" else "positions" if topic.endswith("/feedback/joint_states") else None
        if field:
            values = dict(zip(data["joint_names"], data[field]))
            for name, value in values.items():
                old = previous.get(topic, {}).get(name)
                if value is not None and old is not None:
                    maxima.setdefault(topic, {})[name] = max(maxima.get(topic, {}).get(name, 0.), abs(value - old))
            previous[topic] = values
        stream.write(json.dumps({"elapsed_s": now - started, "topic": topic, "data": data},
                                ensure_ascii=False, allow_nan=False) + "\n")

    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = Node("gamepad_trace", enable_rosout=False, start_parameter_services=False)
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        for suffix, (message_type, kind) in topics.items():
            topic = "/piper_servo/" + suffix
            node.create_subscription(message_type, topic, lambda msg, t=topic, k=kind: receive(msg, t, k), qos)
        print(f"只读采样 {args.duration:g} 秒，输出 {output}；请按现有步骤操作，采样器不会触发运动。")
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=min(.1, max(0., args.duration - (time.monotonic() - started))))
    except KeyboardInterrupt:
        reason = "interrupted"
    except Exception:
        reason = "error"
        raise
    finally:
        stream.close()
        for topic, item in stats.items():
            total = item.pop("interval_sum_s")
            item["interval_mean_s"] = total / (item["count"] - 1) if item["count"] > 1 else None
            if topic in maxima:
                item["max_abs_frame_change_by_joint"] = maxima[topic]
                item["max_abs_frame_change"] = max(maxima[topic].values(), default=None)
                item["frame_change_unit"] = "rad/s" if topic.endswith("/delta_joint_cmds") else "rad"
        summary = dict(started_at=started_at.isoformat(), elapsed_s=time.monotonic() - started, reason=reason,
                       active_status_samples=active_samples, active_jog_samples=active_jogs, topics=stats,
                       conclusion="仅采样，未自动评判抖动改善" if active_samples else "没有 ACTIVE 样本，不能判断抖动是否改善")
        (output / "summary.json").write_text(json.dumps(finite(summary), ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(summary["conclusion"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
