"""RViz six-axis handles edit drafts only; applying collision geometry is explicit."""
import json
import math
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker


class SceneMarkers:
    def __init__(self, node, session):
        self.session = session
        self.server = InteractiveMarkerServer(node, '/multi_point/scene_editor')
        self.signature = None

    def refresh(self):
        state = self.session.status()
        signature = (state['scene_revision'], state['scene_editing'], state['busy'], state['scene_selected'])
        if signature == self.signature: return
        self.signature = signature
        self.server.clear()
        if state['scene_editing'] and not state['busy']:
            for obj in state['scene_objects']:
                selected = obj['id'] == state['scene_selected']
                marker = InteractiveMarker(name=obj['id'], description=obj['id'] if selected else '', scale=.22)
                marker.header.frame_id = obj['frame_id']
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = obj['position_m']
                q = marker.pose.orientation
                q.x, q.y, q.z, q.w = obj['orientation_xyzw']
                body = Marker()
                body.pose.orientation.w = 1.
                body.color.r, body.color.g, body.color.b, body.color.a = 1., .55, .05, .55
                if obj['shape'] == 'box':
                    body.type = Marker.CUBE
                    body.scale.x, body.scale.y, body.scale.z = obj['dimensions_m']
                elif obj['shape'] == 'sphere':
                    body.type = Marker.SPHERE
                    body.scale.x = body.scale.y = body.scale.z = 2*obj['dimensions_m'][0]
                else:
                    body.type = Marker.CYLINDER
                    body.scale.z = obj['dimensions_m'][0]
                    body.scale.x = body.scale.y = 2*obj['dimensions_m'][1]
                visible = InteractiveMarkerControl(name='select_object',always_visible=True,
                                                   interaction_mode=InteractiveMarkerControl.BUTTON)
                visible.markers = [body]
                marker.controls.append(visible)
                for axis, field in ((('x','x'), ('y','z'), ('z','y')) if selected else ()):
                    for label, mode in (('move', InteractiveMarkerControl.MOVE_AXIS),
                                        ('rotate', InteractiveMarkerControl.ROTATE_AXIS)):
                        handle = InteractiveMarkerControl(name=label+'_'+axis, interaction_mode=mode)
                        handle.orientation.w = math.sqrt(.5)
                        setattr(handle.orientation, field, math.sqrt(.5))
                        marker.controls.append(handle)
                self.server.insert(marker, feedback_callback=self.feedback)
        self.server.applyChanges()

    def feedback(self, feedback):
        state = self.session.status()
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            if state['scene_editing']:
                accepted, message = self.session.submit('scene_select', feedback.marker_name)
                self.signature = None
                if not accepted: print(f'[场景选择] {message}', flush=True)
            return
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP: return
        if feedback.control_name not in {'move_x','move_y','move_z','rotate_x','rotate_y','rotate_z'}: return
        if not state['scene_editing'] or feedback.marker_name != state['scene_selected']:
            self.signature = None
            return
        p, q = feedback.pose.position, feedback.pose.orientation
        accepted, message = self.session.submit('scene_pose', json.dumps(dict(id=feedback.marker_name,
            position_m=[p.x,p.y,p.z], orientation_xyzw=[q.x,q.y,q.z,q.w])))
        self.signature = None  # Also restore rejected edits to the authoritative draft.
        if not accepted: print(f'[场景编辑] {message}', flush=True)

    def close(self):
        self.server.shutdown()
