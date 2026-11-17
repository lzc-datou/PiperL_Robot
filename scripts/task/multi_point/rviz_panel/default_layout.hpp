#pragma once
#include <QDockWidget>
#include <QMainWindow>
#include <QTabWidget>
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
  for(auto *dock:docks) {
    const auto name=dock->objectName();
    if(name=="Displays" || name=="Help" || name=="Views" || name.contains("Trajectory Slider"))
      dock->hide(); // Still available in the Panels menu.
  }
  planning->setFloating(false);task_dock->setFloating(false);
  window->addDockWidget(Qt::LeftDockWidgetArea,planning);
  window->addDockWidget(Qt::LeftDockWidgetArea,task_dock);
  window->splitDockWidget(planning,task_dock,Qt::Vertical);
  planning->show();task_dock->show();
  for(auto *tabs:planning->findChildren<QTabWidget*>()) {
    for(int i=0;i<tabs->count();++i)
      if(tabs->tabText(i)=="Planning") tabs->setCurrentIndex(i);
  }
  const int width=std::max(planning->minimumSizeHint().width(),int(window->width()*0.42));
  window->resizeDocks({planning,task_dock},{width,width},Qt::Horizontal);
  window->resizeDocks({planning,task_dock},{int(window->height()*0.45),int(window->height()*0.55)},Qt::Vertical);
  return true;
}
}
