#include "task_panel.hpp"
#include "default_layout.hpp"
#include "scene_marker_display.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QFileDialog>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTabWidget>
#include <atomic>
#include <unistd.h>

namespace piper_multi_point_rviz {
namespace {
void textIfChanged(QLabel *label,const QString &text) {
  if(label->text()!=text) label->setText(text);
}
}
TaskPanel::TaskPanel(QWidget *parent) : rviz_common::Panel(parent) {
  auto *rootLayout=new QVBoxLayout(this);rootLayout->setContentsMargins(0,0,0,0);
  auto *tabs=new QTabWidget(this);rootLayout->addWidget(tabs);
  auto *taskPage=new QWidget;tabs->addTab(taskPage,"运动");
  scene_editor_=new SceneEditor;tabs->addTab(scene_editor_,"场景");
  connect(scene_editor_,&SceneEditor::command,this,[this](const QString &name,const QString &payload){send(name.toStdString(),payload.toStdString());});
  auto *layout=new QVBoxLayout(taskPage);layout->setContentsMargins(8,8,8,8);layout->setSpacing(8);
  auto *files=new QHBoxLayout();
  path_=new QLineEdit(this);path_->setPlaceholderText("任务 YAML 文件");path_->setMinimumWidth(100);
  browse_=new QPushButton("选择…",this);load_=new QPushButton("加载",this);
  files->addWidget(path_,1);files->addWidget(browse_);files->addWidget(load_);layout->addLayout(files);
  motion_=new MotionControls(this);layout->addWidget(motion_);
  connect(motion_,&MotionControls::command,this,[this](const QString &name){send(name.toStdString());});
  state_=new QLabel("未连接",this);state_->setObjectName("taskState");
  state_->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
  layout->addWidget(state_);layout->addStretch();
  connect(browse_,&QPushButton::clicked,this,[this]{
    auto file=QFileDialog::getOpenFileName(this,"选择任务配置",path_->text(),"YAML (*.yaml *.yml)");
    if(!file.isEmpty()) path_->setText(file);
  });
  connect(load_,&QPushButton::clicked,this,[this]{send("load",path_->text().toStdString());});
  connect(path_,&QLineEdit::textChanged,this,[this]{refresh();});
  connect(&timer_,&QTimer::timeout,this,[this]{if(node_) executor_.spin_some(std::chrono::milliseconds(5)); refresh();});
  clock_.start();refresh();
}
TaskPanel::~TaskPanel() {
  timer_.stop();
  restoreOfficialDisplay();
}
void TaskPanel::onInitialize() {
  static std::atomic<unsigned> count{0};
  node_=std::make_shared<rclcpp::Node>("piper_task_panel_"+std::to_string(getpid())+"_"+std::to_string(++count));
  executor_.add_node(node_);
  client_=node_->create_client<srv::TaskCommand>("/piper_multi_point/command");
  status_=node_->create_subscription<std_msgs::msg::String>("/piper_multi_point/status",rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg){receive(msg->data);});
  ensurePreviewDisplay();
  timer_.start(100);
}
void TaskPanel::ensurePreviewDisplay() {
  auto *context=getDisplayContext();if(!context || preview_) return;
  auto *root=displayRoot();
  rviz_common::Display *editor=nullptr;
  for(int i=0;i<root->numDisplays();++i){
    auto *d=root->getDisplayAt(i);
    if(d->getName()=="Scene editor" && d->getClassId()=="rviz_default_plugins/InteractiveMarkers")editor=d;
  }
  if(!editor){
    editor=root->createDisplay("rviz_default_plugins/InteractiveMarkers");
    if(editor){editor->setName("Scene editor");root->addDisplay(editor);editor->initialize(context);}
  }
  // Also repair saved RViz layouts created before the namespace correction.
  if(editor){configureSceneMarkerDisplay(editor);editor->setEnabled(true);}
  for(int i=0;i<root->numDisplays();++i) {
    auto *d=root->getDisplayAt(i);
    if(d->getClassId()=="moveit_rviz_plugin/Trajectory" && d->getName()=="Multi-point preview") {preview_=d;break;}
  }
  if(!preview_) {
    preview_=root->createDisplay("moveit_rviz_plugin/Trajectory");
    if(!preview_) return;
    preview_->setName("Multi-point preview");root->addDisplay(preview_);
    preview_->initialize(context);
    preview_->subProp("Trajectory Topic")->setValue("/multi_point/display_planned_path");
    preview_->subProp("Loop Animation")->setValue(true);
    preview_->subProp("State Display Time")->setValue("1x");
    preview_->setEnabled(true);
  }
}
void TaskPanel::restoreOfficialDisplay() {
  for(const auto &saved:saved_properties_) if(saved.property) saved.property->setValue(saved.value);
  saved_properties_.clear();
}
void TaskPanel::showTaskPreview(bool visible) {
  ensurePreviewDisplay();
  if(preview_) {
    // Do not reset/reload the robot model on status updates or mode changes.
    // The subscriber must stay alive so a fast plan is not lost during switching.
    preview_->subProp("Show Robot Visual")->setValue(visible);
    preview_->subProp("Show Robot Collision")->setValue(false);
    preview_->subProp("Show Trail")->setValue(false);
    preview_->subProp("Interrupt Display")->setValue(true);
    preview_->subProp("Loop Animation")->setValue(visible);
    preview_->subProp("Robot Alpha")->setValue(0.35);
    preview_->subProp("Color Enabled")->setValue(false);
  }
  if(!visible) restoreOfficialDisplay();
  auto *context=getDisplayContext();if(!context) return;
  auto hide=[this](rviz_common::properties::Property *p) {
    saved_properties_.push_back({p,p->getValue()});p->setValue(false);
  };
  auto *root=displayRoot();
  for(int i=0;i<root->numDisplays();++i) {
    auto *d=root->getDisplayAt(i);
    if(d->getClassId()!="moveit_rviz_plugin/MotionPlanning") continue;
    auto *path=d->subProp("Planned Path");
    path->subProp("Color Enabled")->setValue(false);
    path->subProp("Robot Alpha")->setValue(0.35);
    path->subProp("Loop Animation")->setValue(false);
    if(visible) {
      hide(path->subProp("Show Robot Visual"));
      hide(path->subProp("Show Robot Collision"));
      hide(path->subProp("Show Trail"));
    }
    auto *scene=d->subProp("Scene Robot");
    // The solid model always follows the official monitored real joint state.
    scene->subProp("Show Robot Visual")->setValue(true);
    scene->subProp("Show Robot Collision")->setValue(false);
    scene->subProp("Robot Alpha")->setValue(1.0);
    auto *request=d->subProp("Planning Request");
    request->subProp("Start State Alpha")->setValue(0.45);
    request->subProp("Goal State Alpha")->setValue(0.45);
    if(visible) {
      hide(request->subProp("Query Start State"));
      hide(request->subProp("Query Goal State"));
    }
  }
}
void TaskPanel::send(const std::string &command,const std::string &path) {
  if(!client_ || !client_->service_is_ready()) {notice_="未连接";refresh();return;}
  if(waiting_ && command!="stop") return;
  auto request=std::make_shared<srv::TaskCommand::Request>();request->command=command;request->path=path;
  waiting_=true;request_started_=clock_.elapsed();notice_.clear();
  client_->async_send_request(request,[this](rclcpp::Client<srv::TaskCommand>::SharedFuture future){
    waiting_=false;
    try {auto reply=future.get();if(!reply->accepted) {notice_="请求未接受 · 查看终端";state_->setToolTip(QString::fromStdString(reply->message));}}
    catch(const std::exception &e){notice_="请求失败 · 查看终端";RCLCPP_ERROR(node_->get_logger(),"%s",e.what());}
  });refresh();
}
void TaskPanel::receive(const std::string &json) {
  auto doc=QJsonDocument::fromJson(QByteArray::fromStdString(json));if(!doc.isObject()) return;
  auto value=doc.object();last_status_=clock_.elapsed();
  scene_status_=value;
  state_->setToolTip(value["message"].toString());
  recovery_required_=value["recovery_required"].toBool();
  busy_=value["busy"].toBool();has_config_=value["has_config"].toBool();can_execute_=value["can_execute"].toBool();
  const auto state=value["state"].toString();
  if(state!=backend_state_) notice_.clear();backend_state_=state;
  requested_preview_visible_=value["preview_visible"].toBool();
  const auto config=value["config_path"].toString();
  if(config!=loaded_path_) {
    loaded_path_=config;
    if(!path_->hasFocus() && !config.isEmpty()) path_->setText(config);
    path_->setToolTip(config);
  }
}
rviz_common::DisplayGroup *TaskPanel::displayRoot() {
  auto *context=getDisplayContext();if(!context) return nullptr;
  auto *root=context->getRootDisplayGroup();
  for(int i=0;i<root->numDisplays();++i) {
    auto *d=root->getDisplayAt(i);
    if(d->getClassId()=="piper_multi_point_rviz/TimeCompatibleGroup")
      return dynamic_cast<rviz_common::DisplayGroup*>(d);
  }
  return root;
}
void TaskPanel::ensureCompatibleDisplays() {
#ifdef PIPER_RVIZ_NANOSECOND_DELTA
  auto *context=getDisplayContext();if(!context) return;
  auto *root=context->getRootDisplayGroup();
  auto *group=displayRoot();
  if(group==root) {
    group=dynamic_cast<rviz_common::DisplayGroup*>(root->createDisplay("piper_multi_point_rviz/TimeCompatibleGroup"));
    if(!group) return;
    group->setName("Displays (time compatible)");root->addDisplay(group);
    group->initialize(context);group->setEnabled(true);
    RCLCPP_INFO(node_->get_logger(),"RViz 14.1.23 frame-time units corrected; existing displays retained");
  }
  // Reparent without reset, enable toggles or reconstruction: interactive goals,
  // planner settings and subscribers stay alive. Later added displays join too.
  for(int i=root->numDisplays()-1;i>=0;--i) {
    auto *d=root->getDisplayAt(i);if(d==group) continue;
    root->takeDisplay(d);group->addDisplay(d);
  }
#endif
}
void TaskPanel::refresh() {
  ensureCompatibleDisplays();
  if(default_layout_pending_ && clock_.elapsed()>1000) {
    auto *context=getDisplayContext();
    if(context && context->getWindowManager()) {
      auto *window=qobject_cast<QMainWindow*>(context->getWindowManager()->getParentWindow());
      if(applyDefaultLayout(window,this)) default_layout_pending_=false;
    }
  }
  const bool online=client_ && client_->service_is_ready() && clock_.elapsed()-last_status_<2000;
  if(waiting_ && clock_.elapsed()-request_started_>5000){waiting_=false;notice_="请求超时 · 查看终端";RCLCPP_ERROR(node_->get_logger(),"Task request acknowledgement timed out; do not repeat execution blindly");}
  const bool visible=online && requested_preview_visible_;
  if(!visibility_known_ || visible!=preview_visible_) {
    showTaskPreview(visible);preview_visible_=visible;visibility_known_=true;
  }
  const bool idle=online && !busy_ && !waiting_;
  scene_editor_->updateState(scene_status_,idle);
  browse_->setEnabled(!busy_);path_->setEnabled(!busy_);load_->setEnabled(idle && !path_->text().isEmpty());
  motion_->updateState(scene_status_,online,idle,path_->text()==loaded_path_,preview_visible_);
  const QMap<QString,QString> labels={{"ERROR","任务失败 · 查看终端"},{"RECOVERING","恢复控制中…"},{"IDLE","就绪"},{"MANUAL","单点操作"},{"LOADED","已加载"},{"PLANNING","规划中…"},{"PLANNED","规划完成 · 请选择预览"},{"READY","待确认执行"},{"EXECUTING","执行中…"}};
  QString status=online ? labels.value(backend_state_,"就绪") : "未连接";
  if(online && (backend_state_=="START_INVALID" || backend_state_=="FEEDBACK_UNAVAILABLE"))status=scene_status_["summary"].toString("反馈检查失败");
  if(online && !notice_.isEmpty()) status=notice_;
  if(online && recovery_required_ && !busy_) status="已停止 · 请恢复控制";
  textIfChanged(state_,status);
}
void TaskPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);config.mapSetValue("TaskConfig",path_->text());
  config.mapSetValue("Default Left Layout",false); // Preserve operator-saved layouts.
}
void TaskPanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);QString path;if(config.mapGetString("TaskConfig",&path)) path_->setText(path);
  config.mapGetBool("Default Left Layout",&default_layout_pending_);
}
}
PLUGINLIB_EXPORT_CLASS(piper_multi_point_rviz::TaskPanel,rviz_common::Panel)
