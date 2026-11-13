#include <moveit/controller_manager/controller_manager.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace soarm4_moveit_config
{
// Planning only: this plugin exposes no command interface or controller handles.
class PreviewOnlyControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:
  void initialize(const rclcpp::Node::SharedPtr& node) override
  {
    RCLCPP_INFO(node->get_logger(), "Preview-only controller manager: hardware execution unavailable.");
  }
  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string&) override { return {}; }
  void getControllersList(std::vector<std::string>& names) override { names.clear(); }
  void getActiveControllers(std::vector<std::string>& names) override { names.clear(); }
  void getControllerJoints(const std::string&, std::vector<std::string>& joints) override { joints.clear(); }
  ControllerState getControllerState(const std::string&) override { return {}; }
  bool switchControllers(const std::vector<std::string>&, const std::vector<std::string>&) override { return false; }
};
}
PLUGINLIB_EXPORT_CLASS(soarm4_moveit_config::PreviewOnlyControllerManager, moveit_controller_manager::MoveItControllerManager)
