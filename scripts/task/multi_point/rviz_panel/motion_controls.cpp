#include "motion_controls.hpp"
#include <QVBoxLayout>
#include <QStyle>

namespace piper_multi_point_rviz {
MotionControls::MotionControls(QWidget *parent) : QWidget(parent) {
  setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Fixed);
  auto *layout=new QVBoxLayout(this);layout->setContentsMargins(0,0,0,0);layout->setSpacing(8);layout->setAlignment(Qt::AlignTop);
  auto *title=new QLabel("规划与执行",this);title->setProperty("role","sectionTitle");layout->addWidget(title);
  grid_=new QGridLayout;grid_->setSpacing(8);layout->addLayout(grid_);
  grid_->setColumnStretch(0,1);grid_->setColumnStretch(1,1);
  baseline_=new QLabel("无避障路径",this);avoidance_=new QLabel("有避障路径",this);
  for(auto *label:{baseline_,avoidance_}) {label->setAlignment(Qt::AlignCenter);label->setProperty("role","muted");}
  const QMap<QString,QString> labels={
    {"plan_baseline","1  规划"},{"plan_avoidance","1  规划"},
    {"preview_baseline","2  预览"},{"preview_avoidance","2  预览"},
    {"execute_baseline","3  执行"},{"execute_avoidance","3  执行"},
    {"plan","规划并预览"},{"execute","执行已预览轨迹"},
    {"replay","重播预览"},{"stop","停止任务"},{"manual","返回单点控制"},{"recover","恢复控制"}};
  for(auto it=labels.begin();it!=labels.end();++it) {
    auto *button=new QPushButton(it.value(),this);button->setObjectName(it.key());
    button->setMinimumHeight(52);button->setMinimumWidth(0);buttons_[it.key()]=button;
    button->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
    button->setProperty("role",it.key().startsWith("execute")?"primary":"secondary");
    connect(button,&QPushButton::clicked,this,[this,name=it.key()]{emit command(name);});
  }
  buttons_["execute_baseline"]->setToolTip("实机执行已预览的无避障轨迹；执行前检查起点与场景");
  buttons_["execute_avoidance"]->setToolTip("实机执行已预览的有避障轨迹；执行前检查起点与场景");
  buttons_["execute"]->setToolTip("执行当前已预览的轨迹；执行前检查起点与场景");
  buttons_["stop"]->setToolTip("取消本程序任务，不是硬件急停");
  buttons_["manual"]->setToolTip("退出多点预览，使用官方单点控制；停止后需先恢复控制");
  buttons_["recover"]->setToolTip("检查停稳反馈并恢复控制门；不恢复旧轨迹、不自动使能");
  buttons_["stop"]->setProperty("role","danger");
  buttons_["recover"]->setProperty("role","primary");
  persistent_=new QWidget(this);persistent_->setObjectName("taskPersistentActions");persistent_->setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Fixed);
  auto *persistentLayout=new QVBoxLayout(persistent_);persistentLayout->setContentsMargins(0,0,0,0);persistentLayout->setSpacing(4);
  persistentLayout->addWidget(buttons_["stop"]);persistentLayout->addWidget(buttons_["recover"]);
  for(const QString name:{QString("stop"),QString("recover")})buttons_[name]->setProperty("persistent",true);
  arrange(false);
}
void MotionControls::arrange(bool comparison) {
  while(auto *item=grid_->takeAt(0)) delete item;
  for(auto it=buttons_.begin();it!=buttons_.end();++it)
    if(it.key()!="stop" && it.key()!="recover") it.value()->hide();
  for(auto *label:{baseline_,avoidance_})label->hide();
  comparison_=comparison;
  for(int row=0;row<7;++row)grid_->setRowStretch(row,0);
  auto add=[this](const QString &name,int row,int column,int span=1) {
    grid_->addWidget(buttons_[name],row,column,1,span);buttons_[name]->show();
  };
  if(comparison) {
    grid_->addWidget(baseline_,0,0);grid_->addWidget(avoidance_,0,1);baseline_->show();avoidance_->show();
    add("plan_baseline",1,0);add("plan_avoidance",1,1);
    add("preview_baseline",2,0);add("preview_avoidance",2,1);
    add("execute_baseline",3,0);add("execute_avoidance",3,1);
  } else {add("plan",0,0,2);add("execute",1,0,2);}
  const int row=comparison?4:2;
  add("replay",row,0);add("manual",row,1);
  buttons_["stop"]->show();buttons_["recover"]->hide();
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
  const auto emphasize=[this](const QString &name,bool primary) {
    auto *button=buttons_[name];const QString role=primary?"primary":"secondary";
    if(button->property("role").toString()==role)return;
    button->setProperty("role",role);button->style()->unpolish(button);button->style()->polish(button);
  };
  emphasize("plan",!gate && !state["can_execute"].toBool());emphasize("execute",!gate && state["can_execute"].toBool());
  for(const QString mode:{QString("baseline"),QString("avoidance")}) {
    const bool planned=state["has_"+mode].toBool(), executable=state["can_execute_"+mode].toBool();
    emphasize("plan_"+mode,!gate && !planned);emphasize("preview_"+mode,!gate && planned && !executable);emphasize("execute_"+mode,!gate && executable);
  }
}
}
