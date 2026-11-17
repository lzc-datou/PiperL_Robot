"""Dedicated RViz panel API; no changes to official MoveIt services."""
import json
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from piper_multi_point_rviz.srv import TaskCommand


class SessionAPI:
    def __init__(self, client, session):
        self.node, self.session = client.node, session
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.node.create_publisher(String, '/piper_multi_point/status', qos)
        self.service = self.node.create_service(TaskCommand, '/piper_multi_point/command', self.command)
        self.scene_markers = None
        if hasattr(session, 'scene_revision'):
            from .scene_markers import SceneMarkers
            self.scene_markers = SceneMarkers(self.node, session)
        self.timer = self.node.create_timer(0.2, self.publish)

    def command(self, request, response):
        response.accepted, response.message = self.session.submit(request.command, request.path)
        if not response.accepted:
            print(f"[面板] {response.message}", flush=True)
        return response

    def publish(self):
        if self.scene_markers is not None: self.scene_markers.refresh()
        self.publisher.publish(String(data=json.dumps(self.session.status(), ensure_ascii=False)))

    def close(self):
        self.node.destroy_timer(self.timer)
        if self.scene_markers is not None: self.scene_markers.close()
        self.node.destroy_service(self.service)
        self.node.destroy_publisher(self.publisher)
