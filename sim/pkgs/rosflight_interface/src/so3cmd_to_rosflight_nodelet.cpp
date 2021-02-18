#include <geometry_msgs/PoseStamped.h>
#include <kr_mav_msgs/SO3Command.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <rosflight_msgs/Command.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Geometry>

class SO3CmdToRosflight : public nodelet::Nodelet {
 public:
  void onInit(void);

 private:
  void so3_cmd_callback(const kr_mav_msgs::SO3Command::ConstPtr &msg);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
  void imu_callback(const sensor_msgs::Imu::ConstPtr &pose);

  bool odom_set_, imu_set_, so3_cmd_set_;
  Eigen::Quaterniond odom_q_, imu_q_;
  int num_props_;
  double max_prop_force_;

  ros::Publisher command_pub_;

  ros::Subscriber so3_cmd_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;

  double so3_cmd_timeout_;
  ros::Time last_so3_cmd_time_;
  kr_mav_msgs::SO3Command last_so3_cmd_;
};

void SO3CmdToRosflight::odom_callback(
    const nav_msgs::Odometry::ConstPtr &odom) {
  if (!odom_set_) odom_set_ = true;

  odom_q_ = Eigen::Quaterniond(
      odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
}

void SO3CmdToRosflight::imu_callback(const sensor_msgs::Imu::ConstPtr &pose) {
  if (!imu_set_) imu_set_ = true;

  imu_q_ = Eigen::Quaterniond(pose->orientation.w, pose->orientation.x,
                              pose->orientation.y, pose->orientation.z);

  if (so3_cmd_set_ &&
      ((ros::Time::now() - last_so3_cmd_time_).toSec() >= so3_cmd_timeout_)) {
    ROS_INFO(
        "[rosflight_nodelet:] so3_cmd timeout. %f seconds since last command",
        (ros::Time::now() - last_so3_cmd_time_).toSec());
    const auto last_so3_cmd_ptr =
        boost::make_shared<kr_mav_msgs::SO3Command>(last_so3_cmd_);

    so3_cmd_callback(last_so3_cmd_ptr);
  }
}

void SO3CmdToRosflight::so3_cmd_callback(
    const kr_mav_msgs::SO3Command::ConstPtr &msg) {
  if (!so3_cmd_set_) so3_cmd_set_ = true;

  // grab desired forces and rotation from so3
  const Eigen::Vector3d f_des(msg->force.x, msg->force.y, msg->force.z);

  const Eigen::Quaterniond q_des(msg->orientation.w, msg->orientation.x,
                                 msg->orientation.y, msg->orientation.z);

  tf::Quaternion q_des_tf =
      tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z,
                     msg->orientation.w);

  // check psi for stability
  const Eigen::Matrix3d R_des(q_des);
  const Eigen::Matrix3d R_cur(odom_q_);

  const float Psi =
      0.5f * (3.0f - (R_des(0, 0) * R_cur(0, 0) + R_des(1, 0) * R_cur(1, 0) +
                      R_des(2, 0) * R_cur(2, 0) + R_des(0, 1) * R_cur(0, 1) +
                      R_des(1, 1) * R_cur(1, 1) + R_des(2, 1) * R_cur(2, 1) +
                      R_des(0, 2) * R_cur(0, 2) + R_des(1, 2) * R_cur(1, 2) +
                      R_des(2, 2) * R_cur(2, 2)));

  if (Psi > 1.0f)  // Position control stability guaranteed only when Psi < 1
  {
    ROS_WARN_THROTTLE(1, "Psi %2.2f > 1.0, orientation error is too large!",
                      Psi);
  }

  double force =
      f_des(0) * R_cur(0, 2) + f_des(1) * R_cur(1, 2) + f_des(2) * R_cur(2, 2);

  double throttle = force / (num_props_ * max_prop_force_);

  // clamp from 0.0 to 1.0
  throttle = std::min(1.0, throttle);
  throttle = std::max(0.0, throttle);

  if (!msg->aux.enable_motors) throttle = 0;

  // publish messages
  auto setpoint_msg = boost::make_shared<rosflight_msgs::Command>();
  setpoint_msg->header = msg->header;
  setpoint_msg->mode =
      rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  setpoint_msg->ignore = rosflight_msgs::Command::IGNORE_NONE;

  double roll_sp, pitch_sp, yaw_sp;
  tf::Matrix3x3(q_des_tf).getRPY(roll_sp, pitch_sp, yaw_sp);

  // Require conversion to FRD frame for rosflight
  setpoint_msg->x = roll_sp;
  setpoint_msg->y = -pitch_sp;
  setpoint_msg->z = -msg->angular_velocity.z;  // yaw rate

  // No field in rosflight_msgs/Command for rate setpoints at same time as
  // attitude? setpoint_msg->mode =
  // rosflight_msgs::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
  // setpoint_msg->x = msg->angular_velocity.x;
  // setpoint_msg->y = msg->angular_velocity.y;
  // setpoint_msg->z = msg->angular_velocity.z;

  setpoint_msg->F = throttle;

  command_pub_.publish(setpoint_msg);

  // save last so3_cmd
  last_so3_cmd_ = *msg;
  last_so3_cmd_time_ = ros::Time::now();
}

void SO3CmdToRosflight::onInit(void) {
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  // get thrust scaling parameters
  if (priv_nh.getParam("num_props", num_props_))
    ROS_INFO("Got number of props: %d", num_props_);
  else
    ROS_ERROR("Must set num_props param");

  if (priv_nh.getParam("max_prop_force", max_prop_force_))
    ROS_INFO("Using max_prop_force=%g ", max_prop_force_);
  else
    ROS_ERROR("Must set max_prop_force param for thrust scaling");

  ROS_ASSERT_MSG(max_prop_force_ > 0,
                 "max_prop_force must be positive. max_prop_force = %g",
                 max_prop_force_);

  // get param for so3 command timeout duration
  priv_nh.param("so3_cmd_timeout", so3_cmd_timeout_, 0.25);

  odom_set_ = false;
  imu_set_ = false;
  so3_cmd_set_ = false;

  command_pub_ = priv_nh.advertise<rosflight_msgs::Command>("command", 10);

  so3_cmd_sub_ =
      priv_nh.subscribe("so3_cmd", 10, &SO3CmdToRosflight::so3_cmd_callback,
                        this, ros::TransportHints().tcpNoDelay());

  odom_sub_ = priv_nh.subscribe("odom", 10, &SO3CmdToRosflight::odom_callback,
                                this, ros::TransportHints().tcpNoDelay());

  imu_sub_ = priv_nh.subscribe("imu", 10, &SO3CmdToRosflight::imu_callback,
                               this, ros::TransportHints().tcpNoDelay());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SO3CmdToRosflight, nodelet::Nodelet);
