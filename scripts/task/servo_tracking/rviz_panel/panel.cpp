#include "panel.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QStringList>
#include <QSignalBlocker>
#include <QPainter>
#include <QEvent>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_group.hpp>
#include <cmath>
#include <unistd.h>

namespace piper_servo_tracking {
namespace {
// Status and numeric text must never resize RViz's dock. Paint a single line
// within the width chosen by the user; QLabel retains the complete text.
class StableLabel : public QLabel {
public:
  explicit StableLabel(const QString &text,QWidget *parent=nullptr):QLabel(text,parent){
    setTextFormat(Qt::PlainText);setWordWrap(false);setMinimumWidth(0);
    setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
    setFixedHeight(fontMetrics().height()+4);
  }
  QSize sizeHint()const override{return QSize(0,fontMetrics().height()+4);}
  QSize minimumSizeHint()const override{return sizeHint();}
protected:
  void changeEvent(QEvent *event)override{
    QLabel::changeEvent(event);
    if(event->type()==QEvent::FontChange || event->type()==QEvent::StyleChange)
      setFixedHeight(fontMetrics().height()+4);
  }
  void paintEvent(QPaintEvent *)override{
    QPainter painter(this);painter.setFont(font());
    painter.setPen(palette().color(isEnabled() ? QPalette::Active : QPalette::Disabled,foregroundRole()));
    painter.drawText(contentsRect(),alignment()|Qt::TextSingleLine,
      fontMetrics().elidedText(text(),Qt::ElideRight,contentsRect().width()));
  }
};

QString compactState(const QString &text){
  QString result=text.section('\n',0,0).simplified();
  // Keep the cause category visible, with full diagnostics available on hover.
  const int separator=result.indexOf(QChar(0xff1a));
  if(separator>0)result=result.left(separator);
  return result.size()>26 ? result.left(25)+QChar(0x2026) : result;
}
}
Panel::Panel(QWidget *parent):rviz_common::Panel(parent) {
  auto *layout=new QVBoxLayout(this);
  layout->setContentsMargins(10,10,10,10);layout->setSpacing(8);
  mode_=new StableLabel("六轴伺服");mode_->setStyleSheet("font-size:18px; font-weight:600");
  state_=new StableLabel("连接中…");
  layout->addWidget(mode_);layout->addWidget(state_);
  auto *legend=new QHBoxLayout;
  auto *target_legend=new QLabel("黄色：目标");
  target_legend->setStyleSheet("color:#a37b00");
  legend->addWidget(target_legend);legend->addWidget(new QLabel("原色：实际"));
  legend->addStretch();layout->addLayout(legend);
  auto *control_row=new QHBoxLayout;
  control_row->addWidget(new QLabel("控制"));
  control_=new QComboBox;control_->addItems({"伺服","单点"});
  control_->setToolTip("切换模式会先停止当前动作。伺服：连续跟踪黄色目标，奇异起点会自动经 MoveIt 规划到配置的准备姿态。单点：从实测关节规划到黄色目标，点击执行");
  control_row->addWidget(control_,1);layout->addLayout(control_row);
  auto *row=new QHBoxLayout;
  start_=new QPushButton("开启跟踪");stop_=new QPushButton("停止");
  plan_=new QPushButton("规划");execute_=new QPushButton("执行");
  plan_->setToolTip("从当前实测关节规划到黄色目标；只规划，不移动机械臂");
  execute_->setToolTip("执行已检查的单点计划；修改目标后需要重新规划");
  start_->setToolTip("保留已设置的目标，检查可达后开始连续跟踪；只有“重置目标”会对齐实际末端");
  for(auto *button:{start_,plan_,execute_,stop_}){
    button->setFixedHeight(36);button->setMinimumWidth(52);
    button->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
  }
  stop_->setStyleSheet("QPushButton {background:#b73b3b; color:white; font-weight:600;}");
  // The stack reserves the same geometry for both modes, including hidden
  // buttons. Switching modes cannot change the dock's minimum width/height.
  actions_=new QStackedWidget;
  auto *servo_actions=new QWidget;auto *servo_layout=new QHBoxLayout(servo_actions);
  servo_layout->setContentsMargins(0,0,0,0);servo_layout->setSpacing(0);servo_layout->addWidget(start_);
  auto *point_actions=new QWidget;auto *point_layout=new QHBoxLayout(point_actions);
  point_layout->setContentsMargins(0,0,0,0);point_layout->setSpacing(8);
  point_layout->addWidget(plan_,1);point_layout->addWidget(execute_,1);
  actions_->addWidget(servo_actions);actions_->addWidget(point_actions);actions_->setFixedHeight(36);
  row->addWidget(actions_,2);row->addWidget(stop_,1);layout->addLayout(row);
  auto *reset_row=new QHBoxLayout;
  reset_=new QPushButton("重置目标");home_=new QPushButton("回零");
  for(auto *button:{reset_,home_}){
    button->setMinimumHeight(32);button->setMinimumWidth(52);
    button->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
    reset_row->addWidget(button,1);
  }
  layout->addLayout(reset_row);
  reset_->setToolTip("停止跟踪，将目标对齐实际末端；不驱动机械臂");
  home_->setToolTip("停止跟踪，经关节规划回到六轴 0°；不重新标零、不自动使能");
  stop_->setToolTip("软件停止；不代替物理急停");
  speed_label_=new StableLabel("速度 50%");layout->addWidget(speed_label_);
  slider_=new QSlider(Qt::Horizontal);slider_->setRange(10,100);slider_->setValue(50);
  layout->addWidget(slider_);
  speed_limits_=new StableLabel("上限 — mm/s · — °/s");layout->addWidget(speed_limits_);
  speed_limits_->setToolTip("TCP 线速度与角速度上限，非实时速度；还受关节软件上限限制");
  position_error_=new StableLabel("位置误差 — mm");layout->addWidget(position_error_);
  position_error_->setToolTip("目标虚影 TCP 与实际 TCP 的直线距离；远目标按限速逐步跟踪");
  orientation_error_=new StableLabel("姿态误差 — °");layout->addWidget(orientation_error_);
  orientation_error_->setToolTip("目标虚影 TCP 与实际 TCP 的三维最短旋转角，不是关节角；只平移目标时，姿态误差可为 0°");
  reach_=new StableLabel("目标：未检查");layout->addWidget(reach_);
  posture_=new StableLabel("姿态：等待反馈");layout->addWidget(posture_);
  show_=new QCheckBox("显示可达范围（近似）");show_->setChecked(true);layout->addWidget(show_);
  layout->addStretch();
  connect(start_,&QPushButton::clicked,this,[this]{send("start");});
  connect(plan_,&QPushButton::clicked,this,[this]{send("plan");});
  connect(execute_,&QPushButton::clicked,this,[this]{send("execute");});
  connect(stop_,&QPushButton::clicked,this,[this]{send("stop");});
  connect(reset_,&QPushButton::clicked,this,[this]{send("reset");});
  connect(home_,&QPushButton::clicked,this,[this]{send("home");});
  connect(control_,QOverload<int>::of(&QComboBox::activated),this,[this](int index){sendMode(index==1);});
  connect(slider_,&QSlider::valueChanged,this,[this](int value){
    speed_label_->setText(QString("速度 %1%").arg(value));
    updateSpeedLimits(value/100.);
    if(speed_){std_msgs::msg::Float64 msg;msg.data=value/100.;speed_->publish(msg);}
  });
  connect(show_,&QCheckBox::toggled,this,[this](bool value){
    if(workspace_){std_msgs::msg::Bool msg;msg.data=value;workspace_->publish(msg);}
  });
  connect(&timer_,&QTimer::timeout,this,[this]{
    if(!node_) return;
    executor_.spin_some(std::chrono::milliseconds(3));
    std_msgs::msg::Float64 msg;msg.data=node_->now().seconds();heartbeat_->publish(msg);
    const bool fresh=received_.isValid() && received_.elapsed()<800;
    updateControls(fresh);
    if(!fresh){
      setState("连接中断 / 等待任务");
      position_error_->setText("位置误差 — mm");
      orientation_error_->setText("姿态误差 — °");
      linear_speed_base_=angular_speed_base_=-1.;
      speed_limits_->setToolTip("TCP 线速度与角速度上限，非实时速度；关节软件上限等待状态反馈");
      updateSpeedLimits(slider_->value()/100.);
    }
  });
  updateControls(false);
}
Panel::~Panel(){timer_.stop();}
void Panel::onInitialize(){
  node_=std::make_shared<rclcpp::Node>("servo_panel_"+std::to_string(getpid()),
    rclcpp::NodeOptions().use_global_arguments(false));
  executor_.add_node(node_);
  for(const auto &name:{"start","stop","reset","home","plan","execute"})clients_[name]=node_->create_client<std_srvs::srv::Trigger>(std::string("/piper_servo/")+name);
  mode_client_=node_->create_client<std_srvs::srv::SetBool>("/piper_servo/control_mode");
  status_=node_->create_subscription<std_msgs::msg::String>("/piper_servo/status",rclcpp::QoS(1).transient_local().reliable(),
    [this](std_msgs::msg::String::ConstSharedPtr msg){receive(msg->data);});
  heartbeat_=node_->create_publisher<std_msgs::msg::Float64>("/piper_servo/panel/heartbeat",1);
  speed_=node_->create_publisher<std_msgs::msg::Float64>("/piper_servo/speed_scale",1);
  workspace_=node_->create_publisher<std_msgs::msg::Bool>("/piper_servo/workspace_visible",1);
  timer_.start(100);
}
void Panel::send(const std::string &name){
  if(!clients_.count(name) || !clients_[name]->service_is_ready()){setState("任务未连接");return;}
  clients_[name]->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
      const auto response=future.get();
      if(!response->success)setState(QString::fromStdString(response->message));
    });
}
void Panel::sendMode(bool point_mode){
  // Status is authoritative even when a request is rejected or delayed.
  {const QSignalBlocker blocker(control_);control_->setCurrentIndex(point_mode_ ? 1 : 0);}
  if(!mode_client_ || !mode_client_->service_is_ready()){setState("任务未连接");return;}
  auto request=std::make_shared<std_srvs::srv::SetBool::Request>();request->data=point_mode;
  switching_mode_=true;updateControls(true);
  mode_client_->async_send_request(request,
    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future){
      switching_mode_=false;
      const auto response=future.get();
      if(!response->success)setState(QString::fromStdString(response->message));
      updateControls(received_.isValid() && received_.elapsed()<800);
    });
}
void Panel::updateControls(bool fresh){
  actions_->setCurrentIndex(point_mode_ ? 1 : 0);
  const bool available=fresh && !active_ && !busy_ && !switching_mode_;
  start_->setEnabled(available && !point_mode_);
  plan_->setEnabled(available && point_mode_);
  execute_->setEnabled(available && point_mode_ && plan_ready_);
  control_->setEnabled(fresh && !switching_mode_);
  reset_->setEnabled(fresh);stop_->setEnabled(fresh);
  home_->setEnabled(fresh && !busy_ && !switching_mode_);
}
void Panel::setState(const QString &summary,const QString &detail){
  const auto compact=compactState(summary);
  if(state_->text()!=compact)state_->setText(compact);
  state_->setToolTip(detail.isEmpty() ? summary : detail);
}
void Panel::updateSpeedLimits(double scale){
  const bool fresh=received_.isValid() && received_.elapsed()<800;
  const auto format_limit=[scale,fresh](double base,double conversion,int precision){
    if(!fresh || !std::isfinite(base) || base<0. || !std::isfinite(scale) || scale<=0.)return QString("—");
    return QString::number(base*scale*conversion,'f',precision);
  };
  speed_limits_->setText(QString("上限 %1 mm/s · %2°/s")
    .arg(format_limit(linear_speed_base_,1000.,0))
    .arg(format_limit(angular_speed_base_,180./std::acos(-1.),1)));
}
void Panel::receive(const std::string &json){
  const auto object=QJsonDocument::fromJson(QByteArray::fromStdString(json)).object();
  if(object.isEmpty())return;
  received_.restart();active_=object["active"].toBool();busy_=object["busy"].toBool();
  point_mode_=object["control_mode"].toString()=="point";plan_ready_=object["plan_ready"].toBool();
  {const QSignalBlocker blocker(control_);control_->setCurrentIndex(point_mode_ ? 1 : 0);}
  updateControls(true);
  // A constant button label avoids changing size hints when tracking starts.
  mode_->setText((object["mode"].toString()=="real" ? "实机 · " : "纯仿真 · ")+QString(point_mode_ ? "单点运动" : "六轴伺服"));
  execute_->setToolTip("执行已检查的单点计划；修改目标或速度后需要重新规划");
  const auto state_text=object["state"].toString();
  const auto feedback_error=object["feedback_error"].toString();
  QStringList bound_axes;
  for(const auto value:object["feedback_bounds"].toArray()){
    auto joint=value.toObject()["joint"].toString();
    if(joint.startsWith("joint"))joint="J"+joint.mid(5);
    if(!joint.isEmpty())bound_axes.append(joint);
  }
  setState(!object["feedback_accepted"].toBool() && !bound_axes.isEmpty()
    ? "反馈超限 "+bound_axes.join(" / ") : state_text,
    state_text+(feedback_error.isEmpty() ? QString() : "\n"+feedback_error));
  const auto reach_text=object["reachability"].toString();
  reach_->setText("目标："+(reach_text.contains("关节角超出模型限位") ? QString("反馈超限") : reach_text));
  reach_->setToolTip(reach_text);
  const auto format_error=[](const QJsonValue &value,double threshold,int decimals){
    if(!value.isDouble() || !std::isfinite(value.toDouble()) || value.toDouble()<0.)
      return QString("—");
    const double error=value.toDouble();
    if(error>0. && error<threshold)
      return QString("<%1").arg(threshold,0,'f',decimals);
    return QString::number(error,'f',decimals);
  };
  position_error_->setText(QString("位置误差 %1 mm").arg(format_error(object["position_error_mm"],.1,1)));
  orientation_error_->setText(QString("姿态误差 %1 °").arg(format_error(object["orientation_error_deg"],.01,2)));
  const auto level=object["singularity_level"].toString();
  posture_->setVisible(level!="disabled");
  posture_->setText(level=="halt" ? "姿态：奇异区域" : level=="warning" ? "姿态：接近奇异" : level=="normal" ? "姿态：正常" : "姿态：等待反馈");
  posture_->setStyleSheet(level=="halt" ? "color:#bd3030" : level=="warning" ? "color:#b47800" : "");
  posture_->setToolTip(object["singularity_condition"].isDouble() ? QString("雅可比条件数 %1").arg(object["singularity_condition"].toDouble(),0,'f',1) : "雅可比条件数不可用或趋于无穷");
  const auto status_scale=object["speed_scale"];
  const auto base_limit=[&status_scale](const QJsonValue &value){
    if(!status_scale.isDouble() || !std::isfinite(status_scale.toDouble()) || status_scale.toDouble()<=0. ||
       !value.isDouble() || !std::isfinite(value.toDouble()) || value.toDouble()<0.)return -1.;
    return value.toDouble()/status_scale.toDouble();
  };
  linear_speed_base_=base_limit(object["linear_speed_limit_mps"]);
  angular_speed_base_=base_limit(object["angular_speed_limit_radps"]);
  const auto joint_limits=object["joint_velocity_limits_radps"].toArray();
  QStringList joint_text;
  if(joint_limits.size()==6)for(const auto value:joint_limits){
    if(!value.isDouble() || !std::isfinite(value.toDouble()) || value.toDouble()<0.){joint_text.clear();break;}
    joint_text.append(QString::number(value.toDouble(),'f',2));
  }
  speed_limits_->setToolTip("TCP 线速度与角速度上限，非实时速度；还受关节软件上限限制"+
    (joint_text.size()==6 ? QString("\n关节软件上限 J1–J6：%1 rad/s").arg(joint_text.join(" / ")) : QString()));
  if(!slider_->isSliderDown()){
    const QSignalBlocker blocker(slider_);
    slider_->setValue(qRound(object["speed_scale"].toDouble(.5)*100));
    speed_label_->setText(QString("速度 %1%").arg(slider_->value()));
  }
  updateSpeedLimits(slider_->value()/100.);
  const QSignalBlocker blocker(show_);show_->setChecked(object["workspace_visible"].toBool(true));
}
// Same compatibility fix as the project's multi-point display, without depending on that task.
class TimeCompatibleGroup: public rviz_common::DisplayGroup {
public:
  void update(float wall_dt,float ros_dt)override{
#ifdef PIPER_RVIZ_NANOSECOND_DELTA
    wall_dt*=1e-9f;ros_dt*=1e-9f;
#endif
    rviz_common::DisplayGroup::update(wall_dt,ros_dt);
  }
};
}
PLUGINLIB_EXPORT_CLASS(piper_servo_tracking::Panel,rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(piper_servo_tracking::TimeCompatibleGroup,rviz_common::Display)
