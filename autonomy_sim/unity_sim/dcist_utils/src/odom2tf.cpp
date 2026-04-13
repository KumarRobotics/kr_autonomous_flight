#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.hpp>

#include <memory>
#include <optional>
#include <string>

namespace dcist {

std::optional<geometry_msgs::msg::TransformStamped> LookupTransform(
    const tf2_ros::Buffer& buffer, const std::string& target_frame,
    const std::string& source_frame, const rclcpp::Logger& logger) {
  try {
    return buffer.lookupTransform(target_frame, source_frame,
                                  tf2::TimePointZero);
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(logger, "%s", ex.what());
    return {};
  }
}

class Odom2Tf : public rclcpp::Node {
 public:
  Odom2Tf()
      : Node("odom2tf"),
        buffer_(this->get_clock()),
        listener_(buffer_),
        broadcaster_(this),
        static_broadcaster_(this) {
    world_frame_ = this->declare_parameter<std::string>("world_frame", "world");
    robot_ = this->declare_parameter<std::string>("robot", "");
    ground_truth_robot_frame_ = this->declare_parameter<std::string>(
        "ground_truth_robot_frame", "none");

    RCLCPP_INFO(this->get_logger(), "world_frame: %s", world_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "robot: %s", robot_.c_str());

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom_in", 100,
        std::bind(&Odom2Tf::OdomCb, this, std::placeholders::_1));
    pub_odom_ =
        this->create_publisher<nav_msgs::msg::Odometry>("odom_out", 100);
  }

  bool SendTransformWorldMap(const nav_msgs::msg::Odometry& odom_msg) {
    const auto true_state_frame = ground_truth_robot_frame_;
    const auto tf_world_robot = LookupTransform(
        buffer_, world_frame_, true_state_frame, this->get_logger());
    if (!tf_world_robot) {
      RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "[odom2tf:] Ground truth robot frame specified as: "
              << ground_truth_robot_frame_
              << ", but it is not found, map to world tf is not published, "
                 "fix this!! \n");
      return false;
    }

    // NOTE: we assume at this point T_map_odom is identity
    Eigen::Isometry3d T_map_robot;  // same as T_odom_robot
    tf2::fromMsg(odom_msg.pose.pose, T_map_robot);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "T_map_robot from odom: \n" << T_map_robot.matrix());

    const auto T_world_robot = tf2::transformToEigen(*tf_world_robot);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "T_world_robot from tf: \n" << T_world_robot.matrix());

    // Compute the desired transform T_world_map
    const auto T_world_map = T_world_robot * T_map_robot.inverse();

    // Send static tf
    auto tf_world_map = tf2::eigenToTransform(T_world_map);
    tf_world_map.header.frame_id = world_frame_;
    tf_world_map.header.stamp = odom_msg.header.stamp;
    tf_world_map.child_frame_id = robot_ + "/map";

    RCLCPP_WARN(this->get_logger(), "Send static transform from %s to %s",
                tf_world_map.child_frame_id.c_str(),
                tf_world_map.header.frame_id.c_str());
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "T_world_map: \n" << T_world_map.matrix());

    static_broadcaster_.sendTransform(tf_world_map);
    return true;
  }

  void OdomCb(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
    if (!static_tf_sent_) {
      static_tf_sent_ = SendTransformWorldMap(*odom_msg);
    }

    // relay odom and publish tf
    auto odom_msg_out = *odom_msg;
    odom_msg_out.header.frame_id = robot_ + "/odom";
    odom_msg_out.child_frame_id = robot_;
    pub_odom_->publish(odom_msg_out);

    geometry_msgs::msg::TransformStamped tf_odom_robot;
    tf_odom_robot.header = odom_msg_out.header;
    tf_odom_robot.child_frame_id = robot_;
    tf_odom_robot.transform.translation.x = odom_msg_out.pose.pose.position.x;
    tf_odom_robot.transform.translation.y = odom_msg_out.pose.pose.position.y;
    tf_odom_robot.transform.translation.z = odom_msg_out.pose.pose.position.z;
    tf_odom_robot.transform.rotation = odom_msg_out.pose.pose.orientation;
    broadcaster_.sendTransform(tf_odom_robot);
  }

 private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  tf2_ros::TransformBroadcaster broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;

  bool static_tf_sent_{false};
  std::string world_frame_;
  std::string robot_;
  std::string ground_truth_robot_frame_;
};

}  // namespace dcist

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dcist::Odom2Tf>());
  rclcpp::shutdown();
  return 0;
}
