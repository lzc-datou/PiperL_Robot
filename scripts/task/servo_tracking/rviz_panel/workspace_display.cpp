#include <memory>
#include <utility>
#include <vector>
#include <OgreMovableObject.h>
#include <OgreSceneNode.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/interaction/selection_handler.hpp>
#include <rviz_default_plugins/displays/marker_array/marker_array_display.hpp>

namespace piper_servo_tracking {
// A translucent envelope must not intercept selection rays to the TCP inside it.
// Hide only this display's geometry during RViz's offscreen picking passes.
// Normal rendering, disabled displays and individual marker visibility are preserved.
class WorkspacePickFilter : public rviz_common::interaction::SelectionHandler {
public:
  WorkspacePickFilter(rviz_common::DisplayContext *context, Ogre::SceneNode *root)
  : SelectionHandler(context), root_(root) {}

  void preRenderPass(uint32_t) override {
    if (depth_++ == 0) hide(root_);
  }
  void postRenderPass(uint32_t) override {
    if (depth_ && --depth_ == 0) {
      for (const auto &entry : visibility_) entry.first->setVisible(entry.second);
      visibility_.clear();
    }
  }
private:
  void hide(Ogre::SceneNode *node) {
    for (auto *object : node->getAttachedObjects()) {
      visibility_.emplace_back(object, object->getVisible());
      object->setVisible(false);
    }
    for (auto *child : node->getChildren()) {
      if (auto *scene = dynamic_cast<Ogre::SceneNode *>(child)) hide(scene);
    }
  }
  Ogre::SceneNode *root_;
  unsigned depth_ = 0;
  std::vector<std::pair<Ogre::MovableObject *, bool>> visibility_;
};

class WorkspaceDisplay : public rviz_default_plugins::displays::MarkerArrayDisplay {
public:
  void onInitialize() override {
    MarkerArrayDisplay::onInitialize();
    pick_filter_ = rviz_common::interaction::createSelectionHandler<WorkspacePickFilter>(
      context_, scene_node_);
  }
private:
  std::shared_ptr<WorkspacePickFilter> pick_filter_;
};
}
PLUGINLIB_EXPORT_CLASS(piper_servo_tracking::WorkspaceDisplay, rviz_common::Display)
