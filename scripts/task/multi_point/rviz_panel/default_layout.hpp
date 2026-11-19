#pragma once
#include <QDockWidget>
#include <QMainWindow>
#include <QTabWidget>
#include "../../common/rviz_window.hpp"
#include <algorithm>

namespace piper_multi_point_rviz {
// Called once for generated startup configurations, never on status updates.
inline bool applyDefaultLayout(QMainWindow *window,QWidget *task) {
  if(!window || !window->isVisible()) return false;
  QDockWidget *planning=nullptr,*task_dock=nullptr;
  const auto docks=window->findChildren<QDockWidget*>();
  for(auto *dock:docks) {
    if(dock->windowTitle()=="MotionPlanning" || dock->objectName()=="MotionPlanning") planning=dock;
    if(dock->isAncestorOf(task)) task_dock=dock;
  }
  if(!planning || !task_dock) return false; // MoveIt may create its pane later.
  piper_ui::installRvizWindowMenu(window);
  for(auto *dock:docks) {
    const auto name=dock->objectName();
    if(name=="Displays" || name=="Help" || name=="Views" || name.contains("Trajectory Slider"))
      dock->hide(); // Still available in the Panels menu.
  }
  planning->setFloating(false);task_dock->setFloating(false);
  window->addDockWidget(Qt::LeftDockWidgetArea,planning);
  window->addDockWidget(Qt::LeftDockWidgetArea,task_dock);
  window->tabifyDockWidget(planning,task_dock);
  planning->show();task_dock->show();task_dock->raise();
  for(auto *tabs:planning->findChildren<QTabWidget*>()) {
    for(int i=0;i<tabs->count();++i)
      if(tabs->tabText(i)=="Planning") tabs->setCurrentIndex(i);
  }
  const int width=std::clamp(int(window->width()*0.38),460,580);
  window->resizeDocks({task_dock},{width},Qt::Horizontal);
  return true;
}
}
