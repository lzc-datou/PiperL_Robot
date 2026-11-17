#include "scripts/task/multi_point/rviz_panel/motion_controls.hpp"
#include <QApplication>
#include <stdexcept>
using piper_multi_point_rviz::MotionControls;
void check(bool ok,const char *msg){if(!ok)throw std::runtime_error(msg);}
int main(int argc,char **argv){
  QApplication app(argc,argv);MotionControls panel;panel.resize(430,220);panel.show();
  QJsonObject state{{"comparison_enabled",true},{"has_config",true}};
  auto update=[&]{panel.updateState(state,true,true,true,false);app.processEvents();};
  auto button=[&](const QString &name){return panel.findChild<QPushButton*>(name);};
  update();
  auto *grid=qobject_cast<QGridLayout*>(panel.layout());
  const QStringList names={"plan_baseline","plan_avoidance","preview_baseline","preview_avoidance",
    "execute_baseline","execute_avoidance","replay","stop","manual"};
  QString emitted;QObject::connect(&panel,&MotionControls::command,[&](const QString &name){emitted=name;});
  for(int i=0;i<names.size();++i){
    int row,col,rs,cs;auto *b=button(names[i]);grid->getItemPosition(grid->indexOf(b),&row,&col,&rs,&cs);
    check(row==i/2 && col==i%2 && cs==(i==8?2:1),"Wrong five-row layout");
    check(!b->isHidden(),"Missing comparison button");
    b->setEnabled(true);b->click();check(emitted==names[i],"Button dispatches wrong command");
  }
  check(button("plan")->isHidden() && button("execute")->isHidden(),"Legacy controls visible in comparison");
  update();check(!button("preview_baseline")->isEnabled(),"Unplanned preview enabled");
  state["has_baseline"]=true;update();
  check(button("preview_baseline")->isEnabled() && !button("execute_baseline")->isEnabled(),"Execution allowed before preview");
  state["can_execute_baseline"]=true;update();
  check(button("execute_baseline")->isEnabled() && !button("execute_avoidance")->isEnabled(),"Wrong mode execution enabled");
  state["scene_dirty"]=true;update();check(!button("execute_baseline")->isEnabled(),"Dirty scene execution enabled");
  state["scene_dirty"]=false;state["recovery_required"]=true;update();
  check(!button("execute_baseline")->isEnabled() && !button("recover")->isHidden() && button("stop")->isHidden(),"Recovery gate bypassed");
  int row,col,rs,cs;grid->getItemPosition(grid->indexOf(button("recover")),&row,&col,&rs,&cs);
  check(row==3 && col==1,"Recovery adds an extra row");
  panel.updateState(state,true,false,true,false);check(!button("plan_baseline")->isEnabled(),"Busy planning enabled");
  state["comparison_enabled"]=false;state["recovery_required"]=false;update();
  check(!button("plan")->isHidden() && button("plan_baseline")->isHidden(),"Simple task controls changed");
  state["comparison_enabled"]=true;update();
  if(argc>1)check(panel.grab().save(argv[1]),"Screenshot failed");
  return 0;
}
