// RViz 14.1.23 passes nanoseconds through the seconds-based Display API.
// Keep the existing upstream display instances and normalize their frame delta.
#include <rviz_common/display_group.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include "../../common/rviz_window.hpp"
#include <pluginlib/class_list_macros.hpp>
namespace piper_multi_point_rviz {
class TimeCompatibleGroup : public rviz_common::DisplayGroup {
public:
  void load(const rviz_common::Config &config) override {
    rviz_common::DisplayGroup::load(config);
    bool fit=false;config.mapGetBool("Fit Window On Startup",&fit);
    if(auto *manager=context_->getWindowManager())
      piper_ui::prepareRvizWindow(qobject_cast<QMainWindow *>(manager->getParentWindow()),fit);
  }
  void save(rviz_common::Config config) const override {
    rviz_common::DisplayGroup::save(config);
    config.mapSetValue("Fit Window On Startup",false);
  }
  void update(float wall_dt,float ros_dt) override {
#ifdef PIPER_RVIZ_NANOSECOND_DELTA
    wall_dt *= 1e-9f;
    ros_dt *= 1e-9f;
#endif
    rviz_common::DisplayGroup::update(wall_dt,ros_dt);
  }
};
}
PLUGINLIB_EXPORT_CLASS(piper_multi_point_rviz::TimeCompatibleGroup,rviz_common::Display)
