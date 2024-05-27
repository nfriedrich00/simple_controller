#ifndef SIMPLE_CONTROLLER_HPP // include guard
#define SIMPLE_CONTROLLER_HPP

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace simple_controller
{

  int test;

class SimpleController : public nav2_core::Controller
{
public:
    SimpleController() = default;
    ~SimpleController() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void activate() override;

    void deactivate() override;

    void cleanup() override;

    void setPlan(const nav_msgs::msg::Path & path) override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;

    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D * costmap_;
    rclcpp::Logger logger_ {rclcpp::get_logger("SimpleController")};
    rclcpp::Clock::SharedPtr clock_;
    double max_linear_vel_, linear_vel_;
    double max_angular_vel_;
    double lookahead_dist_;
    nav_msgs::msg::Path global_plan_;
};

}
#endif /* SIMPLE_CONTROLLER_HPP */