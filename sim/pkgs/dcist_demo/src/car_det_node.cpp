#include <arl_unity_ros/ImageDetections.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_high_level_msgs/StateTransition.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <mutex>
#include <optional>

namespace dcist_utils {

class TFListener {
 public:
  TFListener() : listener_(buffer_) {}

  std::optional<geometry_msgs::TransformStamped> LookupTransform(
      const std::string& target, const std::string& source,
      const ros::Time& time) {
    geometry_msgs::TransformStamped tf_stamped;
    try {
      return buffer_.lookupTransform(target, source, time, ros::Duration(0.4));
    } catch (tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1, "Fail to find transform from [%s] to [%s]",
                        source.c_str(), target.c_str());
      return {};
    }
  }

 private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

class CarDetNode {
 public:
  explicit CarDetNode(const ros::NodeHandle& pnh);

  void DetectionCb(const arl_unity_ros::ImageDetections& det_msg);
  void ConnectCb();

 private:
  ros::NodeHandle pnh_;
  ros::Subscriber detection_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher path_pub_;
  ros::Publisher state_trans_pub_;
  // std::mutex connect_mtx_;
  int class_id_;
  int object_id_;
  TFListener tf_listener_;
};

CarDetNode::CarDetNode(const ros::NodeHandle& pnh)
    : pnh_(pnh),
      class_id_(pnh_.param<int>("class_id", -1)),
      object_id_(pnh_.param<int>("object_id", -1)) {
  // auto connect_cb = boost::bind(&CarDetNode::ConnectCb, this);
  // std::lock_guard<std::mutex> lock(connect_mtx_);
  detection_sub_ =
      pnh_.subscribe("detection", 1, &CarDetNode::DetectionCb, this);
  pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
  path_pub_ = pnh_.advertise<nav_msgs::Path>("path", 1);
  state_trans_pub_ = pnh_.advertise<mav_high_level_msgs::StateTransition>(
      "state_trigger", 1);

  ROS_INFO("Looking for class %d object %d", class_id_, object_id_);
}

void CarDetNode::DetectionCb(const arl_unity_ros::ImageDetections& det_msg) {
  static bool published = false;
  for (const auto& det : det_msg.detections) {
    if (static_cast<int>(det.class_id) != class_id_) continue;
    if (static_cast<int>(det.object_id) != object_id_) continue;
    if (det.class_name != "Vehicle") continue;

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = det_msg.header;
    pose_msg.pose.position.x = det.x_pos;
    pose_msg.pose.position.y = det.y_pos;
    pose_msg.pose.position.z = det.z_pos;
    pose_msg.pose.orientation.w = 1.0;
    pose_pub_.publish(pose_msg);

    if (!published) {
      const auto tf_map_cam = tf_listener_.LookupTransform(
          "quad1/map", det_msg.header.frame_id, det_msg.header.stamp);

      if (!tf_map_cam) {
        ROS_ERROR("TF problem!");
      }

      // transform det pose from local to map
      const auto T_m_c = tf2::transformToEigen(*tf_map_cam);
      const Eigen::Vector3d p_c(det.x_pos, det.y_pos, det.z_pos);
      const Eigen::Vector3d p_m = T_m_c * p_c;

      geometry_msgs::PoseStamped pose_msg_c;
      pose_msg_c.pose.position.x = p_m.x();
      pose_msg_c.pose.position.y = p_m.y();
      pose_msg_c.pose.position.z = p_m.z() + 4.0;
      pose_msg_c.pose.orientation.w = 1.0;

      nav_msgs::Path path_msg;
      path_msg.header = det_msg.header;
      path_msg.header.frame_id = "/map";
      path_msg.poses.push_back(pose_msg_c);
      path_pub_.publish(path_msg);

      mav_high_level_msgs::StateTransition st_msg;
      st_msg.transition.data = "waypoints";
      state_trans_pub_.publish(st_msg);
      ROS_INFO("!!! Detect car");

      published = true;
    }
  }
}

// void CarDetNode::ConnectCb() {
//   std::lock_guard<std::mutex> lock(connect_mtx_);
//   if (pose_pub_.getNumSubscribers() == 0 &&
//       path_pub_.getNumSubscribers() == 0) {
//     ROS_INFO("Shutting down %s", detection_sub_.getTopic().c_str());
//     detection_sub_.shutdown();
//   } else if (!detection_sub_) {
//     detection_sub_ =
//         pnh_.subscribe("detection", 1, &CarDetNode::DetectionCb, this);
//     ROS_INFO("Resubscribing %s", detection_sub_.getTopic().c_str());
//   }
// }

}  // namespace dcist_utils

int main(int argc, char** argv) {
  ros::init(argc, argv, "car_det_node");
  ros::NodeHandle pnh("~");
  dcist_utils::CarDetNode node(pnh);
  ros::spin();
}
