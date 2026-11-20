#pragma once
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QCheckBox>
#include <QComboBox>
#include <QStackedWidget>
#include <QTimer>
#include <QElapsedTimer>
#include <QPlainTextEdit>

namespace piper_servo_tracking {
class Panel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit Panel(QWidget *parent=nullptr);
  QSize sizeHint() const override { return QSize(440,640); }
  QSize minimumSizeHint() const override { return QSize(320,360); }
  void onInitialize() override;
  ~Panel() override;
private:
  void send(const std::string &name);
  void sendMode(bool point_mode);
  void updateControls(bool fresh);
  void updateSpeedLimits(double scale);
  void setState(const QString &summary,const QString &detail=QString());
  void receive(const std::string &json);
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heartbeat_, speed_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr workspace_;
  std::map<std::string,rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> clients_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr mode_client_;
  QLabel *mode_, *state_, *position_error_, *orientation_error_, *reach_, *posture_, *speed_label_, *speed_limits_;
  QLabel *flow_hint_;
  QPlainTextEdit *state_detail_;
  QPushButton *start_, *stop_, *reset_, *home_, *plan_, *execute_;
  QComboBox *control_;
  QStackedWidget *actions_;
  QSlider *slider_;
  QCheckBox *show_;
  QTimer timer_;
  QElapsedTimer received_;
  bool visual_mode_=false;
  bool active_=false, busy_=false, point_mode_=false, plan_ready_=false, switching_mode_=false;
  double linear_speed_base_=-1., angular_speed_base_=-1.;
};
}
