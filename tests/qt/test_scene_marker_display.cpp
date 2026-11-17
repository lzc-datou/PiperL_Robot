#include "scripts/task/multi_point/rviz_panel/scene_marker_display.hpp"
#include <pluginlib/class_loader.hpp>
#include <QApplication>
#include <QSignalBlocker>
#include <stdexcept>
#include <memory>

int main(int argc,char **argv) {
  QApplication app(argc,argv);
  pluginlib::ClassLoader<rviz_common::Display> loader("rviz_common","rviz_common::Display");
  auto display=loader.createSharedInstance("rviz_default_plugins/InteractiveMarkers");
  // Load the installed RViz plugin, but do not initialize a renderer or ROS node.
  auto *ns=display->subProp("Interactive Markers Namespace");
  auto *aids=display->subProp("Show Visual Aids");
  if(ns->getName()!="Interactive Markers Namespace" || aids->getName()!="Show Visual Aids")
    throw std::runtime_error("Installed RViz does not support scene editor properties");
  QSignalBlocker namespaceSignals(ns), aidsSignals(aids);
  piper_multi_point_rviz::configureSceneMarkerDisplay(display.get());
  if(ns->getValue().toString()!="/multi_point/scene_editor" || !aids->getValue().toBool())
    throw std::runtime_error("Scene editor namespace not configured");
}
