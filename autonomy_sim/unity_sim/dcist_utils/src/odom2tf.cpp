#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <optional>

namespace dcist {

std::optional<geometry_msgs::TransformStamped> LookupTransform(
    const tf2_ros::Buffer& buffer, const std::string& target_frame,
    const std::string& source_frame) {
  try {
    return buffer.lookupTransform(target_frame, source_frame, ros::Time(0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(1, "%s", ex.what());
    return {};
  }
}

struct Odom2Tf {
  Odom2Tf(const ros::NodeHandle& pnh)
      : pnh_(pnh),
        sub_odom_(pnh_.subscribe("odom_in", 100, &Odom2Tf::OdomCb, this)),
        pub_odom_(pnh_.advertise<nav_msgs::Odometry>("odom_out", 100)),
        listener_(buffer_),
        world_frame_(pnh_.param<std::string>("world_frame", "world")),
        robot_(pnh_.param<std::string>("robot", "")),
        ground_truth_robot_frame_(
            pnh_.param<std::string>("ground_truth_robot_frame", "none")) {
    ROS_INFO("world_frame: %s", world_frame_.c_str());
    ROS_INFO("robot: %s", robot_.c_str());
  }

  bool SendTransformWorldMap(const nav_msgs::Odometry& odom_msg) {
    // Lookup tf from true_state to world, which should be provided by simulator
    // const auto true_state_frame = robot_ + "/TrueState";
    const auto true_state_frame = ground_truth_robot_frame_;
    const auto tf_world_robot =
        LookupTransform(buffer_, world_frame_, true_state_frame);
    if (!tf_world_robot) {
      ROS_ERROR_STREAM(
          "[odom2tf:] Ground truth robot frame specified as: "
          << ground_truth_robot_frame_
          << ", but it is not found, map to world tf is not published, fix this!! \n");
      return false;
    }

    ROS_INFO("Odom: %s", sub_odom_.getTopic().c_str());

    // NOTE: we assume at this point T_map_odom is identity
    // Compute transfrom from robot to odom
    Eigen::Isometry3d T_map_robot;  // same as T_odom_robot
    tf2::fromMsg(odom_msg.pose.pose, T_map_robot);
    ROS_INFO_STREAM("T_map_robot from odom: \n" << T_map_robot.matrix());

    const auto T_world_robot = tf2::transformToEigen(*tf_world_robot);
    ROS_INFO_STREAM("T_world_robot from tf: \n" << T_world_robot.matrix());

    // Compute the desired transform T_world_map
    const auto T_world_map = T_world_robot * T_map_robot.inverse();

    // Send static tf
    auto tf_world_map = tf2::eigenToTransform(T_world_map);
    tf_world_map.header.frame_id = world_frame_;
    tf_world_map.header.stamp = odom_msg.header.stamp;
    tf_world_map.child_frame_id = robot_ + "/map";

    ROS_WARN("Send static transform from %s to %s",
             tf_world_map.child_frame_id.c_str(),
             tf_world_map.header.frame_id.c_str());
    ROS_INFO_STREAM("T_world_map: \n" << T_world_map.matrix());

    static_broadcaster_.sendTransform(tf_world_map);
    return true;
  }

  void OdomCb(const nav_msgs::Odometry& odom_msg) {
    if (!static_tf_sent_) {
      static_tf_sent_ = SendTransformWorldMap(odom_msg);
    }

    // relay odom and publish tf
    auto odom_msg_out = odom_msg;
    odom_msg_out.header.frame_id = robot_ + "/odom";
    odom_msg_out.child_frame_id = robot_;
    pub_odom_.publish(odom_msg_out);

    geometry_msgs::TransformStamped tf_odom_robot;
    tf_odom_robot.header = odom_msg_out.header;
    tf_odom_robot.child_frame_id = robot_;
    tf_odom_robot.transform.translation.x = odom_msg_out.pose.pose.position.x;
    tf_odom_robot.transform.translation.y = odom_msg_out.pose.pose.position.y;
    tf_odom_robot.transform.translation.z = odom_msg_out.pose.pose.position.z;
    tf_odom_robot.transform.rotation = odom_msg_out.pose.pose.orientation;
    broadcaster_.sendTransform(tf_odom_robot);
  }

  ros::NodeHandle pnh_;
  ros::Subscriber sub_odom_;
  ros::Publisher pub_odom_;

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
  ros::init(argc, argv, "odom2tf");
  dcist::Odom2Tf node(ros::NodeHandle("~"));
  ros::spin();
  return 0;
}