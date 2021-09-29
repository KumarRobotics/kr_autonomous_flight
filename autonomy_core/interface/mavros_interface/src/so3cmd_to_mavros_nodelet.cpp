#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <kr_mav_msgs/SO3Command.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Geometry>

enum ThrustModels { TM_KARTIK, TM_MIKE };

class SO3CmdToMavros : public nodelet::Nodelet {
 public:
  void onInit(void);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  void so3_cmd_callback(const kr_mav_msgs::SO3Command::ConstPtr &msg);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
  void imu_callback(const sensor_msgs::Imu::ConstPtr &pose);
  void battery_callback(const sensor_msgs::BatteryState::ConstPtr &msg);

  double thrust_model_kartik(double thrust);
  double thrust_model_mike(double thrust);

  bool odom_set_, imu_set_, so3_cmd_set_;
  Eigen::Quaterniond odom_q_, imu_q_;
  double num_props_;
  double thrust_vs_rpm_cof_a_, thrust_vs_rpm_cof_b_, thrust_vs_rpm_cof_c_;
  double rpm_vs_throttle_coeff_a_, rpm_vs_throttle_coeff_b_;

  double bat_cof_, throttle_cof_, const_cof_;  // for mike model

  ros::Publisher attitude_raw_pub_;
  ros::Publisher odom_pose_pub_;  // For sending PoseStamped to firmware

  ros::Subscriber so3_cmd_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber battery_sub_;

  bool check_psi_;
  double battery_voltage_;
  double so3_cmd_timeout_;
  ros::Time last_so3_cmd_time_;
  kr_mav_msgs::SO3Command last_so3_cmd_;

  ThrustModels thrust_model_{TM_KARTIK};
};
void SO3CmdToMavros::battery_callback(
    const sensor_msgs::BatteryState::ConstPtr &msg) {
  battery_voltage_ = msg->voltage;
}

void SO3CmdToMavros::odom_callback(const nav_msgs::Odometry::ConstPtr &odom) {
  if (!odom_set_) odom_set_ = true;

  odom_q_ = Eigen::Quaterniond(
      odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  // Publish PoseStamped for mavros vision_pose plugin
  auto odom_pose_msg = boost::make_shared<geometry_msgs::PoseStamped>();
  odom_pose_msg->header = odom->header;
  odom_pose_msg->pose = odom->pose.pose;
  odom_pose_pub_.publish(odom_pose_msg);
}

void SO3CmdToMavros::imu_callback(const sensor_msgs::Imu::ConstPtr &pose) {
  if (!imu_set_) imu_set_ = true;

  imu_q_ = Eigen::Quaterniond(pose->orientation.w, pose->orientation.x,
                              pose->orientation.y, pose->orientation.z);

  if (so3_cmd_set_ &&
      ((ros::Time::now() - last_so3_cmd_time_).toSec() >= so3_cmd_timeout_)) {
    ROS_INFO("so3_cmd timeout. %f seconds since last command",
             (ros::Time::now() - last_so3_cmd_time_).toSec());
    const auto last_so3_cmd_ptr =
        boost::make_shared<kr_mav_msgs::SO3Command>(last_so3_cmd_);

    so3_cmd_callback(last_so3_cmd_ptr);
  }
}

static std::pair<double, double> solve_quadratic(double a, double b, double c) {
  const double term1 = -b, term2 = std::sqrt(b * b - 4 * a * c);
  return std::make_pair((term1 + term2) / (2 * a), (term1 - term2) / (2 * a));
}
double SO3CmdToMavros::thrust_model_kartik(double thrust) {
  double avg_thrust = std::max(0.0, thrust) / num_props_;

  // Scale thrust to individual rotor velocities (RPM)
  auto rpm_solutions =
      solve_quadratic(thrust_vs_rpm_cof_a_, thrust_vs_rpm_cof_b_,
                      thrust_vs_rpm_cof_c_ - avg_thrust);
  const double omega_avg = std::max(rpm_solutions.first, rpm_solutions.second);

  // Scaling from rotor velocity (RPM) to att_throttle for pixhawk
  double throttle =
      (omega_avg - rpm_vs_throttle_coeff_b_) / rpm_vs_throttle_coeff_a_;
  return throttle;
}
double SO3CmdToMavros::thrust_model_mike(double thrust) {
  // thrust = const_cof_ + bat_cof_*battery_voltage_ + throttle_cof_*throttle
  double throttle =
      (thrust - const_cof_ - bat_cof_ * battery_voltage_) / throttle_cof_;
  return throttle;
}

void SO3CmdToMavros::so3_cmd_callback(
    const kr_mav_msgs::SO3Command::ConstPtr &msg) {
  if (!so3_cmd_set_) so3_cmd_set_ = true;

  // grab desired forces and rotation from so3
  const Eigen::Vector3d f_des(msg->force.x, msg->force.y, msg->force.z);

  const Eigen::Quaterniond q_des(msg->orientation.w, msg->orientation.x,
                                 msg->orientation.y, msg->orientation.z);

  // convert to tf::Quaternion
  tf::Quaternion imu_tf =
      tf::Quaternion(imu_q_.x(), imu_q_.y(), imu_q_.z(), imu_q_.w());
  tf::Quaternion odom_tf =
      tf::Quaternion(odom_q_.x(), odom_q_.y(), odom_q_.z(), odom_q_.w());

  // extract RPY's
  double imu_roll, imu_pitch, imu_yaw;
  double odom_roll, odom_pitch, odom_yaw;
  tf::Matrix3x3(imu_tf).getRPY(imu_roll, imu_pitch, imu_yaw);
  tf::Matrix3x3(odom_tf).getRPY(odom_roll, odom_pitch, odom_yaw);

  // create only yaw tf:Quaternions
  tf::Quaternion imu_tf_yaw;
  tf::Quaternion odom_tf_yaw;
  imu_tf_yaw.setRPY(0.0, 0.0, imu_yaw);
  odom_tf_yaw.setRPY(0.0, 0.0, odom_yaw);
  const tf::Quaternion tf_imu_odom_yaw = imu_tf_yaw * odom_tf_yaw.inverse();

  // transform!
  const Eigen::Quaterniond q_des_transformed =
      Eigen::Quaterniond(tf_imu_odom_yaw.w(), tf_imu_odom_yaw.x(),
                         tf_imu_odom_yaw.y(), tf_imu_odom_yaw.z()) *
      q_des;

  // check psi for stability
  const Eigen::Matrix3d R_des(q_des);
  const Eigen::Matrix3d R_cur(odom_q_);

  const double Psi =
      0.5f * (3.0f - (R_des(0, 0) * R_cur(0, 0) + R_des(1, 0) * R_cur(1, 0) +
                      R_des(2, 0) * R_cur(2, 0) + R_des(0, 1) * R_cur(0, 1) +
                      R_des(1, 1) * R_cur(1, 1) + R_des(2, 1) * R_cur(2, 1) +
                      R_des(0, 2) * R_cur(0, 2) + R_des(1, 2) * R_cur(1, 2) +
                      R_des(2, 2) * R_cur(2, 2)));

  double thrust = 0.0;
  if (!check_psi_ ||
      Psi < 1.0f)  // Position control stability guaranteed only when Psi < 1
  {
    thrust = f_des(0) * R_cur(0, 2) + f_des(1) * R_cur(1, 2) +
             f_des(2) * R_cur(2, 2);
  } else {
    ROS_WARN_THROTTLE(1, "psi > 1.0, thrust set to 0.0 in mavros_interface.");
  }

  double throttle = 0.0;
  if (thrust_model_ == TM_KARTIK)
    throttle = thrust_model_kartik(thrust);
  else if (thrust_model_ == TM_MIKE)
    throttle = thrust_model_mike(thrust);
  else
    ROS_ERROR_THROTTLE(1, "Unimplemented Thrust Model!!");

  // clamp from 0.0 to 1.0
  throttle = std::min(1.0, throttle);
  throttle = std::max(0.0, throttle);

  if (!msg->aux.enable_motors) throttle = 0;

  // publish messages
  auto setpoint_msg = boost::make_shared<mavros_msgs::AttitudeTarget>();
  setpoint_msg->header = msg->header;
  setpoint_msg->type_mask = 0;
  setpoint_msg->orientation.w = q_des_transformed.w();
  setpoint_msg->orientation.x = q_des_transformed.x();
  setpoint_msg->orientation.y = q_des_transformed.y();
  setpoint_msg->orientation.z = q_des_transformed.z();
  setpoint_msg->body_rate.x = msg->angular_velocity.x;
  setpoint_msg->body_rate.y = msg->angular_velocity.y;
  setpoint_msg->body_rate.z = msg->angular_velocity.z;
  setpoint_msg->thrust = throttle;

  attitude_raw_pub_.publish(setpoint_msg);

  // save last so3_cmd
  last_so3_cmd_ = *msg;
  last_so3_cmd_time_ = ros::Time::now();
}

void SO3CmdToMavros::onInit(void) {
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  if (!priv_nh.getParam("check_psi", check_psi_)) check_psi_ = true;
  if (!check_psi_)
    ROS_WARN(
        "Turning off Psi check. You should not need this unless doing "
        "really aggressive manuvers.  Talk to Mike.");

  bool use_kartik_model;
  priv_nh.param("use_kartik_thrust_model", use_kartik_model, true);
  if (use_kartik_model) {
    thrust_model_ = TM_KARTIK;
    if (priv_nh.getParam("num_props", num_props_))
      ROS_INFO("Got number of props: %f", num_props_);
    else
      ROS_ERROR("Must set num_props param");

    // get thrust scaling parameters
    if (priv_nh.getParam("thrust_vs_rpm_coeff_a", thrust_vs_rpm_cof_a_) &&
        priv_nh.getParam("thrust_vs_rpm_coeff_b", thrust_vs_rpm_cof_b_) &&
        priv_nh.getParam("thrust_vs_rpm_coeff_c", thrust_vs_rpm_cof_c_)) {
      ROS_ASSERT_MSG(
          thrust_vs_rpm_cof_a_ > 0,
          "thrust_vs_rpm_cof_a must be positive. thrust_vs_rpm_cof_a = %g",
          thrust_vs_rpm_cof_a_);

      ROS_INFO(
          "Using Thrust = %g*RPM^2 + %g*RPM + %g to scale thrust to prop "
          "speed.",
          thrust_vs_rpm_cof_a_, thrust_vs_rpm_cof_b_, thrust_vs_rpm_cof_c_);
    } else {
      ROS_ERROR(
          "Must set coefficients for thrust vs rpm (scaling from rotor "
          "velocity (RPM) to thrust produced)");
    }

    if (priv_nh.getParam("rpm_vs_throttle_coeff_a", rpm_vs_throttle_coeff_a_) &&
        priv_nh.getParam("rpm_vs_throttle_coeff_b", rpm_vs_throttle_coeff_b_))

      ROS_INFO(
          "Using RPM = %g*throttle + %g to scale prop speed to att_throttle.",
          rpm_vs_throttle_coeff_a_, rpm_vs_throttle_coeff_b_);
    else
      ROS_ERROR(
          "Must set coefficients for thrust scaling (scaling from rotor "
          "velocity (RPM) to att_throttle for pixhawk)");
  } else {
    thrust_model_ = TM_MIKE;
    if (priv_nh.getParam("thrust_vs_throttle", throttle_cof_) &&
        priv_nh.getParam("thrust_vs_battery", bat_cof_) &&
        priv_nh.getParam("thrust_constant", const_cof_)) {
      ROS_INFO_STREAM("Using thrust_vs_throttle: "
                      << throttle_cof_ << ", thrust_vs_battery: " << bat_cof_
                      << " , thrust_constant: " << const_cof_);
    } else
      ROS_ERROR("Must set parameters for thrust");
  }

  // get param for so3 command timeout duration
  priv_nh.param("so3_cmd_timeout", so3_cmd_timeout_, 0.25);

  odom_set_ = false;
  imu_set_ = false;
  so3_cmd_set_ = false;

  attitude_raw_pub_ =
      priv_nh.advertise<mavros_msgs::AttitudeTarget>("attitude_raw", 10);

  odom_pose_pub_ =
      priv_nh.advertise<geometry_msgs::PoseStamped>("odom_pose", 10);

  so3_cmd_sub_ =
      priv_nh.subscribe("so3_cmd", 10, &SO3CmdToMavros::so3_cmd_callback, this,
                        ros::TransportHints().tcpNoDelay());

  odom_sub_ = priv_nh.subscribe("odom", 10, &SO3CmdToMavros::odom_callback,
                                this, ros::TransportHints().tcpNoDelay());

  imu_sub_ = priv_nh.subscribe("imu", 10, &SO3CmdToMavros::imu_callback, this,
                               ros::TransportHints().tcpNoDelay());

  battery_sub_ =
      priv_nh.subscribe("battery", 10, &SO3CmdToMavros::battery_callback, this,
                        ros::TransportHints().tcpNoDelay());
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(SO3CmdToMavros, nodelet::Nodelet);
