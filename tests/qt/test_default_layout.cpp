#include "scripts/task/multi_point/rviz_panel/default_layout.hpp"
#include <QApplication>
#include <QLabel>
#include <QVBoxLayout>
#include <stdexcept>

void check(bool ok,const char *message) {if(!ok) throw std::runtime_error(message);}
int main(int argc,char **argv) {
  QApplication app(argc,argv);
  QMainWindow window;window.resize(1600,1000);
  window.setCentralWidget(new QLabel("Scene"));
  auto *task=new QWidget();
  check(!piper_multi_point_rviz::applyDefaultLayout(&window,task),"Must wait for visible window");
  window.show();app.processEvents();
  check(!piper_multi_point_rviz::applyDefaultLayout(&window,task),"Must wait for both panes");
  auto *planning=new QDockWidget("MotionPlanning",&window);
  planning->setObjectName("MotionPlanning");
  auto *tabs=new QTabWidget();tabs->setMinimumSize(500,360);
  tabs->addTab(new QLabel("Context"),"Context");tabs->addTab(new QLabel("Commands"),"Planning");
  planning->setWidget(tabs);window.addDockWidget(Qt::RightDockWidgetArea,planning);
  auto *task_dock=new QDockWidget(QString::fromUtf8("多点任务"),&window);
  task_dock->setWidget(task);task->setMinimumSize(400,160);
  window.addDockWidget(Qt::BottomDockWidgetArea,task_dock);
  QList<QDockWidget*> extras;
  for(const char *name:{"Displays","Views","Help","MotionPlanning - Trajectory Slider"}) {
    auto *dock=new QDockWidget(name,&window);dock->setObjectName(name);dock->setWidget(new QLabel(name));
    window.addDockWidget(Qt::LeftDockWidgetArea,dock);extras.append(dock);
  }
  app.processEvents();
  check(piper_multi_point_rviz::applyDefaultLayout(&window,task),"Layout should apply");
  app.processEvents();
  check(window.dockWidgetArea(planning)==Qt::LeftDockWidgetArea,"Planning must be on left");
  check(window.dockWidgetArea(task_dock)==Qt::LeftDockWidgetArea,"Task must be on left");
  check(planning->isVisible() && task_dock->isVisible(),"Both panels must remain visible");
  check(planning->geometry().bottom()<task_dock->geometry().top(),"Planning must be above task");
  check(planning->geometry().left()==task_dock->geometry().left(),"Panels must align");
  check(window.tabifiedDockWidgets(planning).isEmpty(),"Panels must not be tabbed together");
  check(tabs->currentIndex()==1,"Planning tab must be selected");
  for(auto *dock:extras) check(!dock->isVisible(),"Extra panes must be hidden");
  return 0;
}
