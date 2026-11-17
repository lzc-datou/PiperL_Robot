#pragma once
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <piper_multi_point_rviz/srv/task_command.hpp>
#include <QElapsedTimer>
#include <QPointer>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <QTimer>
#include <QLineEdit>
#include <QLabel>
#include <QPushButton>
#include "scene_editor.hpp"
#include "motion_controls.hpp"

namespace piper_multi_point_rviz {
class TaskPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit TaskPanel(QWidget *parent = nullptr);
  ~TaskPanel() override;
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config &config) override;
private:
  void send(const std::string &command, const std::string &path = "");
  void refresh();
  void receive(const std::string &json);
  void ensurePreviewDisplay();
  void ensureCompatibleDisplays();
  bool default_layout_pending_=false;
  rviz_common::DisplayGroup *displayRoot();
  void showTaskPreview(bool visible);
  QPointer<rviz_common::Display> preview_;
  struct SavedProperty {QPointer<rviz_common::properties::Property> property; QVariant value;};
  std::vector<SavedProperty> saved_properties_;
  void restoreOfficialDisplay();
  bool recovery_required_=false;
  bool visibility_known_=false,preview_visible_=false,requested_preview_visible_=false;
  QString backend_state_,notice_;
  QLineEdit *path_;
  QString loaded_path_;
  QLabel *state_;
  QPushButton *browse_, *load_;
  MotionControls *motion_;
  SceneEditor *scene_editor_;
  QJsonObject scene_status_;
  QTimer timer_;
  QElapsedTimer clock_;
  qint64 last_status_ = -10000;
  bool busy_ = false, has_config_ = false, can_execute_ = false, waiting_ = false;
  qint64 request_started_ = 0;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<srv::TaskCommand>::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_;
};
}
