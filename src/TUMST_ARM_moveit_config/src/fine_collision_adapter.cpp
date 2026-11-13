#include <moveit/planning_interface/planning_response_adapter.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <cmath>
#include <mutex>

namespace soarm4_moveit_config
{
class FineCollisionAdapter : public planning_interface::PlanningResponseAdapter
{
public:
  void initialize(const rclcpp::Node::SharedPtr& node, const std::string&) override
  {
    node_=rclcpp::Node::make_shared("soarm4_fine_review_client", node->get_namespace());
    client_=node_->create_client<moveit_msgs::srv::GetStateValidity>("fine_check_state");
  }
  std::string getDescription() const override { return "SO-ARM4 detailed joint coupling and limit review"; }
  void adapt(const planning_scene::PlanningSceneConstPtr& scene, const planning_interface::MotionPlanRequest& request,
             planning_interface::MotionPlanResponse& response) const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!response.trajectory || !response.trajectory->getWayPointCount() ||
        !client_->wait_for_service(std::chrono::seconds(5)))
    { reject(response, "Fine review service unavailable or empty path"); return; }
    const auto* group=response.trajectory->getGroup();
    std::size_t samples=0;
    for (std::size_t i=0; i<response.trajectory->getWayPointCount(); ++i)
    {
      const auto& a=response.trajectory->getWayPoint(i ? i-1 : 0);
      const auto& b=response.trajectory->getWayPoint(i);
      double maximum=0.;
      for (const auto& name : group->getVariableNames())
        maximum=std::max(maximum,std::abs(b.getVariablePosition(name)-a.getVariablePosition(name)));
      const int steps=std::max(1,static_cast<int>(std::ceil(maximum/(0.5*M_PI/180.))));
      for (int k=(i ? 1 : 0); k<=steps; ++k)
      {
        moveit::core::RobotState state(a); a.interpolate(b,static_cast<double>(k)/steps,state);
        if (!scene->isStateValid(state,request.group_name))
        { reject(response,"Interpolated path collides with planning scene");return; }
        auto req=std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
        req->group_name=request.group_name;moveit::core::robotStateToRobotStateMsg(state,req->robot_state);
        auto future=client_->async_send_request(req);
        if (rclcpp::spin_until_future_complete(node_,future,std::chrono::seconds(3))!=rclcpp::FutureReturnCode::SUCCESS)
        { reject(response,"Fine review timeout"); return; }
        auto result=future.get();
        if (!result->valid)
        {
          std::string message="Detailed joint-contact/limit review rejected path";
          if (!result->contacts.empty()) message+=": "+result->contacts[0].contact_body_1+" / "+result->contacts[0].contact_body_2;
          reject(response,message);return;
        }
        ++samples;
      }
    }
    RCLCPP_INFO(node_->get_logger(),"RViz/service plan passed detailed review: %zu samples",samples);
  }
private:
  void reject(planning_interface::MotionPlanResponse& response,const std::string& why) const
  { response.error_code=moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;RCLCPP_ERROR(node_->get_logger(),"%s",why.c_str()); }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr client_;
  mutable std::mutex mutex_;
};
}
PLUGINLIB_EXPORT_CLASS(soarm4_moveit_config::FineCollisionAdapter,planning_interface::PlanningResponseAdapter)
