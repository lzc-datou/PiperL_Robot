"""Actual DDS bridge plus state-topic bursts; no control requests or CAN."""
import json
import os
import time
import unittest
from unittest.mock import patch

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from piper_servo_tracking.msg import GamepadInput

from scripts.tools.gamepad_control.ros_bridge import GamepadBridge


class GamepadRosBridgeTests(unittest.TestCase):
    def test_spin_drains_multiple_callbacks_without_an_unbounded_loop(self):
        with patch('scripts.tools.gamepad_control.ros_bridge.time.monotonic', return_value=1.), \
                patch('scripts.tools.gamepad_control.ros_bridge.rclpy.spin_once') as spin:
            marker = object()
            GamepadBridge.spin(marker)
        self.assertEqual(spin.call_count, 16)
        self.assertTrue(all(call.args == (marker,) and call.kwargs == {'timeout_sec': 0.}
                            for call in spin.call_args_list))

    def test_spin_checks_the_four_ms_budget_before_each_callback(self):
        with patch('scripts.tools.gamepad_control.ros_bridge.time.monotonic',
                   side_effect=[1., 1.001, 1.003, 1.0041]), \
                patch('scripts.tools.gamepad_control.ros_bridge.rclpy.spin_once') as spin:
            GamepadBridge.spin(object())
        self.assertEqual(spin.call_count, 2)

    def test_actual_ros_node_can_construct_and_destroy_the_ui_bridge(self):
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = None
        try:
            node = GamepadBridge()
            self.assertEqual(set(node.request_clients), {'start', 'end', 'stop', 'recover'})
            # Node.clients is a read-only public property. It must stay intact.
            self.assertEqual(len(list(node.clients)), 7)
            self.assertIn(node.selection_client, list(node.clients))
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()

    def test_twenty_hz_ui_drains_fifty_hz_input_and_ten_hz_status_topics(self):
        """Publish only state on an isolated, otherwise empty ROS domain."""
        if os.environ.get('ROS_DOMAIN_ID', '43') == '43':
            self.skipTest('state burst requires an explicit non-default isolated ROS domain')
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        bridge = publisher = None
        try:
            bridge = GamepadBridge()
            publisher = Node('gamepad_bridge_state_fixture')
            inputs = publisher.create_publisher(GamepadInput, '/piper_servo/gamepad/input', 10)
            statuses = publisher.create_publisher(String, '/piper_servo/gamepad/backend_status', 10)
            guards = publisher.create_publisher(String, '/piper_servo/guard/status', 10)
            deadline = time.monotonic()+3.
            while time.monotonic() < deadline:
                bridge.spin()
                if all(pub.get_subscription_count() for pub in (inputs, statuses, guards)):
                    break
                time.sleep(.01)
            self.assertTrue(all(pub.get_subscription_count() for pub in (inputs, statuses, guards)))
            self.assertFalse(any(namespace == '/piper_servo'
                                 for name, namespace in publisher.get_node_names_and_namespaces()),
                             'state fixture refuses a domain containing a control stack')
            sequence = 0; last_ui = last_input = last_status = 0.
            started = time.monotonic(); lags = []
            while time.monotonic()-started < 1.2:
                now = time.monotonic()
                if now-last_input >= .02:
                    sequence += 1
                    inputs.publish(GamepadInput(guid='state-burst', generation=1, sequence=sequence,
                        connected=True, axes=[0.]*4, dpad=[0]*4, lb=False, b=False, source='fixture'))
                    last_input = now
                if now-last_status >= .10:
                    snapshot = String(data=json.dumps({'sequence': sequence, 'state': 'WAITING'}))
                    statuses.publish(snapshot); guards.publish(snapshot); last_status = now
                if now-last_ui >= .05:
                    bridge.spin(); last_ui = now
                    if now-started > .3:
                        lags.append(sequence-(bridge.input or {}).get('sequence', 0))
                time.sleep(.001)
            self.assertTrue(lags)
            self.assertLessEqual(max(lags), 3, f'input queue accumulated: {lags}')
            self.assertGreaterEqual(bridge.status.get('sequence', 0), sequence-7)
            self.assertGreaterEqual(bridge.guard.get('sequence', 0), sequence-7)
        finally:
            if publisher is not None:
                publisher.destroy_node()
            if bridge is not None:
                bridge.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
