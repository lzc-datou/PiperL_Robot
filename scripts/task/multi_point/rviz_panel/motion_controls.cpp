#include "motion_controls.hpp"

namespace piper_multi_point_rviz {
MotionControls::MotionControls(QWidget *parent) : QWidget(parent) {
  setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Fixed);
  grid_=new QGridLayout(this);grid_->setContentsMargins(0,0,0,0);grid_->setSpacing(8);
  const QMap<QString,QString> labels={
    {"plan_baseline","规划无避障"},{"plan_avoidance","规划有避障"},
    {"preview_baseline","预览无避障"},{"preview_avoidance","预览有避障"},
    {"execute_baseline","执行无避障"},{"execute_avoidance","执行有避障"},
    {"plan","规划预览"},{"execute","执行（实机）"},
    {"replay","重播"},{"stop","停止任务"},{"manual","返回单点"},{"recover","恢复控制"}};
  for(auto it=labels.begin();it!=labels.end();++it) {
    auto *button=new QPushButton(it.value(),this);button->setObjectName(it.key());
    button->setMinimumHeight(30);buttons_[it.key()]=button;
    connect(button,&QPushButton::clicked,this,[this,name=it.key()]{emit command(name);});
  }
  buttons_["execute_baseline"]->setToolTip("实机执行已预览的无避障轨迹；执行前检查起点与场景");
  buttons_["execute_avoidance"]->setToolTip("实机执行已预览的有避障轨迹；执行前检查起点与场景");
  buttons_["stop"]->setToolTip("取消本程序任务，不是硬件急停");
  buttons_["manual"]->setToolTip("退出多点预览，使用官方单点控制；停止后需先恢复控制");
  buttons_["recover"]->setToolTip("检查停稳反馈并恢复控制门；不恢复旧轨迹、不自动使能");
  arrange(false);
}
void MotionControls::arrange(bool comparison) {
  while(auto *item=grid_->takeAt(0)) delete item;
  for(auto *button:buttons_) button->hide();
  comparison_=comparison;
  auto add=[this](const QString &name,int row,int column,int span=1) {
    grid_->addWidget(buttons_[name],row,column,1,span);buttons_[name]->show();
  };
  if(comparison) {
    add("plan_baseline",0,0);add("plan_avoidance",0,1);
    add("preview_baseline",1,0);add("preview_avoidance",1,1);
    add("execute_baseline",2,0);add("execute_avoidance",2,1);
  } else {add("plan",0,0);add("execute",0,1);}
  const int row=comparison?3:1;
  add("replay",row,0);add("stop",row,1);add("recover",row,1);
  buttons_["recover"]->hide();
  add("manual",row+1,0,2);
}
void MotionControls::updateState(const QJsonObject &state,bool online,bool idle,
                                 bool selected_loaded,bool preview_visible) {
  if(comparison_!=state["comparison_enabled"].toBool()) arrange(!comparison_);
  const bool sceneReady=!state["scene_dirty"].toBool() && !state["scene_editing"].toBool();
  const bool ready=idle && selected_loaded && sceneReady;
  const bool gate=state["recovery_required"].toBool();
  buttons_["plan"]->setEnabled(ready && state["has_config"].toBool());
  buttons_["execute"]->setEnabled(ready && state["can_execute"].toBool() && !gate);
  for(const QString mode:{QString("baseline"),QString("avoidance")}) {
    buttons_["plan_"+mode]->setEnabled(ready && state["has_config"].toBool());
    buttons_["preview_"+mode]->setEnabled(ready && state["has_"+mode].toBool());
    buttons_["execute_"+mode]->setEnabled(ready && state["can_execute_"+mode].toBool() && !gate);
  }
  const bool recover=gate && !state["busy"].toBool();
  buttons_["recover"]->setVisible(recover);buttons_["recover"]->setEnabled(idle && gate);
  buttons_["stop"]->setVisible(!recover);buttons_["stop"]->setEnabled(online && state["busy"].toBool());
  buttons_["replay"]->setEnabled(idle && preview_visible);
  buttons_["manual"]->setEnabled(idle);
}
}
