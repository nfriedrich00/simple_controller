#include "../include/simple_controller/simple_controller.hpp"
#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/costmap_filters/filter_values.hpp>

using nav2_util::geometry_utils::min_by;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;

namespace simple_controller
{

void SimpleController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
    auto node = parent.lock();
    node_ = parent;
    if (!node)
    {
    throw nav2_core::ControllerException("Unable to lock node!");
    }

    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    double control_frequency = 20.0;

// ...
// todo: add parameter handler
// ...

    max_linear_vel_ = 1.0;
    linear_vel_ = max_linear_vel_;
    max_angular_vel_ = 1.0;
    lookahead_dist_ = 2*max_linear_vel_;
}

void SimpleController::cleanup()
{
    RCLCPP_INFO(
        logger_,
        "Cleaning up controller: %s of type"
        " simple_controller::SimpleController",
        plugin_name_.c_str());
}

void SimpleController::activate()
{
    RCLCPP_INFO(
        logger_,
        "Activating controller: %s of type"
        " simple_controller::SimpleController",
        plugin_name_.c_str());
    // Add callback for dynamic parameters
    auto node = node_.lock();//??
}

void SimpleController::deactivate()
{
    RCLCPP_INFO(
        logger_,
        "Deactivating controller: %s of type"
        " simple_controller::SimpleController",
        plugin_name_.c_str());
}

geometry_msgs::msg::TwistStamped SimpleController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) // I don't know, why goal_checker is commented out.
{
    // find nearest point: divide path in two parts and ignore everything behind the robot

    auto nearest_pose_it =
    nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), --global_plan_.poses.end(),
    [&pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(pose, ps);
    });

    // find goal pose for this iteration: begin is nearest pose -> result will be in front of robot
    auto goal_pose_it = std::find_if(
    nearest_pose_it, --global_plan_.poses.end(), [&](const auto & ps) {
      return euclidean_distance(pose, ps) >= lookahead_dist_;
    });


    geometry_msgs::msg::PoseStamped goal_pose = *goal_pose_it;

    // find rotation direction:
    // reference point in front of robot
    // is this point on the left or right side of the path?

    // robot orientation is yaw
    tf2::Quaternion q_robot(
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z,
    pose.pose.orientation.w);

    tf2::Matrix3x3 m(q_robot);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    geometry_msgs::msg::Point reference_point;
    reference_point.x = pose.pose.position.x + cos(yaw);
    reference_point.y = pose.pose.position.y + sin(yaw);

    bool isLeft = (reference_point.x - pose.pose.position.x)*(goal_pose.pose.position.y - pose.pose.position.y) - (reference_point.y - pose.pose.position.y)*(goal_pose.pose.position.x - pose.pose.position.x) < 0;


    // output commands
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();
    cmd_vel.twist.linear.x = linear_vel_;
    #define PI 3.14159265
    if (isLeft) // turn right. positive angular vel (z) is counterclockwise
    {
        cmd_vel.twist.angular.z = -max_angular_vel_;
    }
    else        // turn left
    {
        cmd_vel.twist.angular.z = max_angular_vel_;
    }

  return cmd_vel;
}


void SimpleController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void SimpleController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    linear_vel_ = max_linear_vel_;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      linear_vel_ = max_linear_vel_ * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      linear_vel_ = speed_limit;
    }
  }
}

}  // namespace nav2_regulated_pure_pursuit_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  simple_controller::SimpleController,
  nav2_core::Controller)
