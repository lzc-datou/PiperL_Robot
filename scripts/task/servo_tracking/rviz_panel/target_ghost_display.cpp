#include <moveit/robot_state_rviz_plugin/robot_state_display.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_default_plugins/robot/robot_link.hpp>

namespace piper_servo_tracking {

// Reuse MoveIt's model, material highlighting and state rendering. This display
// receives target IK states only; the solid RobotModel keeps following feedback.
class TargetGhostDisplay : public moveit_rviz_plugin::RobotStateDisplay {
public:
  void update(float wall_dt, float ros_dt) override {
    RobotStateDisplay::update(wall_dt, ros_dt);
    if (!robot_) return;
    // Check after the upstream update so model loads/reloads are covered too.
    // RobotLink's API explicitly allows picking objects behind or inside a
    // nonselectable link, including the TCP sphere enclosed by the target wrist.
    for (const auto &entry : robot_->getRobot().getLinks()) {
      if (entry.second->getSelectable()) entry.second->setSelectable(false);
    }
  }
};

}  // namespace piper_servo_tracking

PLUGINLIB_EXPORT_CLASS(piper_servo_tracking::TargetGhostDisplay, rviz_common::Display)
