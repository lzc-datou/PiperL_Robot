#pragma once
#include <rviz_common/display.hpp>

namespace piper_multi_point_rviz {
inline void configureSceneMarkerDisplay(rviz_common::Display *display) {
  // Jazzy expects a server namespace, not the ROS 1 Update Topic property.
  display->subProp("Interactive Markers Namespace")->setValue("/multi_point/scene_editor");
  display->subProp("Show Visual Aids")->setValue(true);
}
}
