// Copyright 2016 KumarRobotics - Kartik Mohta
#include <angles/angles.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/utils.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <utility>

#include "fla_ukf/fla_ukf.h"

namespace fla_ukf {

class FLAUKFNodelet : public rclcpp::Node {
 public:
  explicit FLAUKFNodelet(const rclcpp::NodeOptions &options);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  // Need this since we have FLAUKF which
                                    // needs aligned pointer
 private:
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void cam_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void lidar_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void laser_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void height_callback(const sensor_msgs::msg::Range::ConstSharedPtr msg);
  void gps_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void mag_callback(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg);
  void vio_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void laser_toggle_callback(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void yaw_callback(
      const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr msg);

  FLAUKF fla_ukf_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      sub_cam_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_laser_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_height_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      sub_lidar_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_gps_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr sub_mag_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_vio_odom_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_height_toggle_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr
      sub_yaw_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_ukf_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_mag_yaw_;
  std::string world_frame_id_, cam_frame_id_, robot_frame_id_, vision_frame_id_,
      lidar_frame_id_;
  unsigned int cam_delay_;
  std::queue<sensor_msgs::msg::Imu> imu_queue_;
  static constexpr int imu_calib_limit_ = 100;
  int imu_calib_count_ = 0;
  Eigen::Vector3d acc_gravity_ = Eigen::Vector3d::Zero();
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  Eigen::Isometry3d T_cam_body_, T_body_cam_, T_world_vision_, T_lidar_body_;
  bool static_transforms_initialized_ = false;
  bool takeoff_detected_ = false;
  bool height_measurement_received_ = false;
  double takeoff_detection_threshold_;
  bool ignore_laser_ = false;
  bool ignore_height_ = false;
  std::optional<Eigen::Isometry3d> T_world_lidarMap_;
  bool enable_lidar_, enable_laser_, enable_height_, enable_gps_, enable_mag_,
      enable_vio_odom_, enable_yaw_;
  double declination_;
  static constexpr size_t mag_buffer_size_ = 100;
  std::array<Eigen::Vector3d, mag_buffer_size_> mag_buffer_;
  size_t mag_buffer_count_ = 0;
  rclcpp::Time last_vio_timestamp_{0, 0, RCL_ROS_TIME};
  bool publish_tf_ = false;
};

/**
 * Copied from
 * https://github.com/KumarRobotics/kr_utils/blob/master/kr_math/include/kr_math/SO3.hpp
 *
 * @brief Get the roll, pitch, yaw angles from a quaternion.
 * @param q Quaternion.
 * @return 3x1 Vector with elements [roll,pitch,yaw] about [x,y,z] axes.
 *
 * @note Assumes quaternion represents rotation of the form:
 * (world) = Rz * Ry * Rx (body).
 *
 */
template <typename T>
FLAUKF::Vec<3> quatToEulerZYX(const Eigen::Quaternion<T> &q) {
  T q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();

  T sth = 2 * (q0 * q2 - q1 * q3);
  if (sth > 1) {
    sth = 1;
  } else if (sth < -1) {
    sth = -1;
  }

  const T theta = std::asin(sth);
  const T cth = std::sqrt(static_cast<T>(1) - sth * sth);

  T phi, psi;
  if (cth < std::numeric_limits<T>::epsilon() * 10) {
    phi = std::atan2(2 * (q1 * q2 - q0 * q3),
                     q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3);
    psi = 0;
  } else {
    phi = std::atan2(2 * (q0 * q1 + q2 * q3),
                     q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    psi = std::atan2(2 * (q1 * q2 + q0 * q3),
                     q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
  }

  FLAUKF::Vec<3> rpy;
  rpy[0] = phi;    //  x, [-pi,pi]
  rpy[1] = theta;  //  y, [-pi/2,pi/2]
  rpy[2] = psi;    //  z, [-pi,pi]
  return rpy;
}

void FLAUKFNodelet::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  if (!static_transforms_initialized_) {
    geometry_msgs::msg::TransformStamped transform_cam_body, transform_world_vision,
        transform_lidar_body;
    try {
      transform_cam_body = tf_buffer_->lookupTransform(
          cam_frame_id_, robot_frame_id_, tf2::TimePointZero);  // Used for front IMU
      if (enable_vio_odom_) {
        transform_world_vision = tf_buffer_->lookupTransform(
            world_frame_id_, vision_frame_id_, tf2::TimePointZero);
      }
      // TODO:unhack this
      // TODO:remap the odometry instead of hard coding
      transform_lidar_body = tf_buffer_->lookupTransform(
          lidar_frame_id_, robot_frame_id_, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }
    T_cam_body_ = tf2::transformToEigen(transform_cam_body.transform);
    T_body_cam_ = T_cam_body_.inverse();
    T_world_vision_ = tf2::transformToEigen(transform_world_vision.transform);
    T_lidar_body_ = tf2::transformToEigen(transform_lidar_body.transform);
    static_transforms_initialized_ = true;
  }

  imu_queue_.push(*msg);
  if (imu_queue_.size() <= cam_delay_) return;

  const rclcpp::Time msg_stamp(msg->header.stamp, RCL_ROS_TIME);
  if ((msg_stamp - last_vio_timestamp_).seconds() >= 0.5 && enable_vio_odom_) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 50,
        "========= WARNING! No recent odometry (LIDAR or VIO) "
        "messages received. If using VIO, check if camera and IMU are published correctly; "
        "If using LIDAR odometry, wait a while (~10s), if warning continues, check if LIDAR"
        "packets are published correctly =========");
  }

  const sensor_msgs::msg::Imu imu_msg = imu_queue_.front();
  imu_queue_.pop();

  // Assemble input
  FLAUKF::InputVec u;
  u(0) = imu_msg.linear_acceleration.x;
  u(1) = imu_msg.linear_acceleration.y;
  u(2) = imu_msg.linear_acceleration.z;
  u(3) = imu_msg.angular_velocity.x;
  u(4) = imu_msg.angular_velocity.y;
  u(5) = imu_msg.angular_velocity.z;
  u.segment<3>(0) = T_body_cam_.rotation() * u.segment<3>(0);
  u.segment<3>(3) = T_body_cam_.rotation() * u.segment<3>(3);

  if (imu_calib_count_ < imu_calib_limit_) {  // Gravity calibration
    imu_calib_count_++;
    acc_gravity_ += u.segment<3>(0);
  } else if (imu_calib_count_ == imu_calib_limit_) {  // Save gravity norm
    imu_calib_count_++;
    acc_gravity_ /= imu_calib_limit_;
    double g = acc_gravity_.norm();

    RCLCPP_INFO(this->get_logger(), "Setting gravity to %f", g);
    fla_ukf_.SetGravity(g);
  } else if (fla_ukf_.ProcessUpdate(
                 u, rclcpp::Time(imu_msg.header.stamp, RCL_ROS_TIME))) {
    auto odom_ukf_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_ukf_msg->header.stamp = imu_msg.header.stamp;
    odom_ukf_msg->header.frame_id = world_frame_id_;
    odom_ukf_msg->child_frame_id = robot_frame_id_;
    auto x = fla_ukf_.GetState();
#if ENABLE_VIO_YAW_OFFSET
    RCLCPP_INFO_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 100,
        "VIO Yaw offset: " << x(15) * 180 / M_PI << " deg");
#endif
    odom_ukf_msg->pose.pose.position.x = x(0);
    odom_ukf_msg->pose.pose.position.y = x(1);
    odom_ukf_msg->pose.pose.position.z = x(2);
    Eigen::Quaterniond q = Eigen::AngleAxisd(x(8), Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(x(7), Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(x(6), Eigen::Vector3d::UnitX());
    odom_ukf_msg->pose.pose.orientation.x = q.x();
    odom_ukf_msg->pose.pose.orientation.y = q.y();
    odom_ukf_msg->pose.pose.orientation.z = q.z();
    odom_ukf_msg->pose.pose.orientation.w = q.w();
    odom_ukf_msg->twist.twist.linear.x = x(3);
    odom_ukf_msg->twist.twist.linear.y = x(4);
    odom_ukf_msg->twist.twist.linear.z = x(5);
    odom_ukf_msg->twist.twist.angular.x = u(3);
    odom_ukf_msg->twist.twist.angular.y = u(4);
    odom_ukf_msg->twist.twist.angular.z = u(5);
    auto P = fla_ukf_.GetStateCovariance();
    for (int j = 0; j < 6; j++)
      for (int i = 0; i < 6; i++)
        odom_ukf_msg->pose.covariance[i + j * 6] =
            P((i < 3) ? i : i + 3, (j < 3) ? j : j + 3);

    if (publish_tf_) {
      // Publish TF
      geometry_msgs::msg::TransformStamped transform_msg;
      transform_msg.header = odom_ukf_msg->header;
      transform_msg.child_frame_id = odom_ukf_msg->child_frame_id;
      transform_msg.transform.translation.x =
          odom_ukf_msg->pose.pose.position.x;
      transform_msg.transform.translation.y =
          odom_ukf_msg->pose.pose.position.y;
      transform_msg.transform.translation.z =
          odom_ukf_msg->pose.pose.position.z;
      transform_msg.transform.rotation = odom_ukf_msg->pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform_msg);
    }

    // Publish Msg
    pub_ukf_odom_->publish(std::move(odom_ukf_msg));
  }
}

void FLAUKFNodelet::lidar_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
  if (!height_measurement_received_) {
    RCLCPP_WARN_ONCE(this->get_logger(), "No height measurement received!");
  }

  // Reset height_measurement_received_ to check for height measurements between
  // each camera pose
  height_measurement_received_ = false;

  if (!T_world_lidarMap_) {
    auto x = fla_ukf_.GetState();
    const Eigen::Vector3d translation{x(0), x(1), x(2)};
    const Eigen::Quaterniond rotation =
        Eigen::AngleAxisd(x(8), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(x(7), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(x(6), Eigen::Vector3d::UnitX());
    Eigen::Isometry3d init_T = Eigen::Isometry3d::Identity();
    init_T.linear() = rotation.toRotationMatrix();
    init_T.translation() = translation;
    T_world_lidarMap_ = init_T;
  }

  Eigen::Isometry3d T_lidarMap_lidar;
  tf2::fromMsg(msg->pose.pose, T_lidarMap_lidar);

  const Eigen::Isometry3d T_world_body =
      T_world_lidarMap_.value() * T_lidarMap_lidar * T_lidar_body_;

  const Eigen::Quaterniond q_world_body(T_world_body.rotation());
  const Eigen::Vector3d t_world_body(T_world_body.translation());

  Eigen::Vector3d rpy = quatToEulerZYX(q_world_body);

  // Assemble measurement
  FLAUKF::MeasCamVec z;
  z(0) = t_world_body(0);
  z(1) = t_world_body(1);
  z(2) = t_world_body(2);
  z(3) = rpy(0);
  z(4) = rpy(1);
  z(5) = rpy(2);
  // Assemble measurement covariance
  FLAUKF::MeasCamCov Rnlidar(FLAUKF::MeasCamCov::Zero());

  // Force inflating the covariance matrix (position x5 and orientation x25)
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      if (i == j) {
        if (i < 3)
          Rnlidar(i, j) = 10;
        else
          Rnlidar(i, j) = 100;
      }
    }
  }

  // Measurement update
  if (fla_ukf_.MeasurementUpdateSE3(
          z, Rnlidar, rclcpp::Time(msg->header.stamp, RCL_ROS_TIME)))
    ignore_laser_ = true;
}

void FLAUKFNodelet::cam_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
  if (!height_measurement_received_) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "No height measurement received!");
  }

  // Reset height_measurement_received_ to check for height measurements between
  // each camera pose
  height_measurement_received_ = false;

  Eigen::Isometry3d T_vision_cam;
  tf2::fromMsg(msg->pose.pose, T_vision_cam);

  const Eigen::Isometry3d T_world_body =
      T_world_vision_ * T_vision_cam * T_cam_body_;

  const Eigen::Quaterniond q_world_body(T_world_body.rotation());
  const Eigen::Vector3d t_world_body(T_world_body.translation());

  const Eigen::Vector3d rpy = quatToEulerZYX(q_world_body);

  // Assemble measurement
  FLAUKF::MeasCamVec z;
  z(0) = t_world_body(0);
  z(1) = t_world_body(1);
  z(2) = t_world_body(2);
  z(3) = rpy(0);
  z(4) = rpy(1);
  z(5) = rpy(2);
  // Assemble measurement covariance
  FLAUKF::MeasCamCov RnCam(FLAUKF::MeasCamCov::Zero());
  RnCam(0, 0) = 0.01;
  RnCam(1, 1) = 0.01;
  RnCam(2, 2) = 0.25;
  RnCam(3, 3) = 0.07;
  RnCam(4, 4) = 0.07;
  RnCam(5, 5) = 0.07;

  // Measurement update
  fla_ukf_.MeasurementUpdateSE3(
      z, RnCam, rclcpp::Time(msg->header.stamp, RCL_ROS_TIME));
}

void FLAUKFNodelet::laser_toggle_callback(
    const std_msgs::msg::Bool::ConstSharedPtr msg) {
  ignore_height_ = !msg->data;
  RCLCPP_INFO_STREAM(this->get_logger(), "Ignore laser: " << ignore_height_);
}

void FLAUKFNodelet::laser_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr /*msg*/) {
  // Placeholder: laser odometry path was commented out in the ROS1 version.
}

void FLAUKFNodelet::height_callback(
    const sensor_msgs::msg::Range::ConstSharedPtr msg) {
  if (ignore_height_) return;
  height_measurement_received_ = true;

  float range = msg->range;
  float range_std_dev = 0.05;

  if (!takeoff_detected_ && range > takeoff_detection_threshold_) {
    takeoff_detected_ = true;
    RCLCPP_WARN_STREAM(this->get_logger(), "Takeoff detected! range: " << range);
  }

  if (!takeoff_detected_) {
    range = 0.0;
    range_std_dev = 0.2;
  }

  // Ignore very high and low range measurement
  if (msg->range < std::min(0.9 * msg->max_range, 20.0) &&
      msg->range > std::max(1.1 * msg->min_range, 0.2)) {
    FLAUKF::MeasHeightVec z;
    z(0) = range;

    // Assemble measurement covariance
    FLAUKF::MeasHeightCov RnHeight{FLAUKF::MeasHeightCov::Zero()};
    RnHeight(0, 0) = range_std_dev * range_std_dev;
    fla_ukf_.MeasurementUpdateHeight(
        z, RnHeight, rclcpp::Time(msg->header.stamp, RCL_ROS_TIME));
  }
}

void FLAUKFNodelet::vio_odom_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  last_vio_timestamp_ = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);

  Eigen::Isometry3d T_vision_cam;
  tf2::fromMsg(msg->pose.pose, T_vision_cam);

  Eigen::Vector3d v_vision_cam;
  tf2::fromMsg(msg->twist.twist.linear, v_vision_cam);

  // TODO: unhack this
  // TODO: actually T_vision_cam is LIDAR odometry
  auto T_world_odom = T_world_vision_;
  auto T_odom_lidar = T_vision_cam;
  const Eigen::Isometry3d T_world_body =
      T_world_odom * T_odom_lidar * T_lidar_body_;  // T_cam_body_;

  const Eigen::Quaterniond q_world_body(T_world_body.rotation());
  const Eigen::Vector3d t_world_body(T_world_body.translation());
  const Eigen::Vector3d v_world_body =
      T_world_vision_.linear() * v_vision_cam;  // Ignoring the (omega \times r)
                                                // since r_cam_body is small

  const Eigen::Vector3d rpy = quatToEulerZYX(q_world_body);

#if VIO_NO_POSITION
  // NOTE(Kartik): Only using orientation and velocity
#else
  // Assemble measurement
  FLAUKF::MeasVioVec z;
  z(0) = t_world_body(0);
  z(1) = t_world_body(1);
  z(2) = t_world_body(2);
  z(3) = v_world_body(0);
  z(4) = v_world_body(1);
  z(5) = v_world_body(2);
  z(6) = rpy(0);
  z(7) = rpy(1);
  z(8) = rpy(2);

  // TODO: unhack this, tuning the covariance for LIDAR odometry integration
  // Assemble measurement covariance
  FLAUKF::MeasVioCov RnVio(FLAUKF::MeasVioCov::Zero());
  double pos_covariance = 0.5;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i != j) {
        RnVio(i, j) = 0;
        continue;
      }
      const double cov_multiplier = 0;
      const double cov_inflater = pos_covariance;
      RnVio(i, j) =
          cov_multiplier * msg->pose.covariance[i * 6 + j] + cov_inflater;
    }
  }
  // Orientation covariance
  double orientation_cov_deg = 10;
  double orientation_cov = orientation_cov_deg * 3.14 / 180;
  for (int i = 6; i < 9; i++) {
    for (int j = 6; j < 9; j++) {
      if (i == j) {
        RnVio(i, j) = orientation_cov;
      } else {
        RnVio(i, j) = 0;
      }
    }
  }

  // Linear velocity covariance
  for (int i = 3; i < 6; i++) {
    for (int j = 3; j < 6; j++) {
      if (i == j) {
        RnVio(i, j) = 50;
      } else {
        RnVio(i, j) = 0;
      }
    }
  }
#endif

  // Measurement update
  if (fla_ukf_.MeasurementUpdateVio(
          z, RnVio, rclcpp::Time(msg->header.stamp, RCL_ROS_TIME)))
    ignore_laser_ = true;
}

void FLAUKFNodelet::gps_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  FLAUKF::MeasGpsVec z;
  z(0) = msg->pose.pose.position.x;
  z(1) = msg->pose.pose.position.y;
  z(2) = msg->pose.pose.position.z;
  z(3) = msg->twist.twist.linear.x;
  z(4) = msg->twist.twist.linear.y;
  z(5) = msg->twist.twist.linear.z;

  // Assemble measurement covariance
  FLAUKF::MeasGpsCov RnGps{FLAUKF::MeasGpsCov::Zero()};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      RnGps(i, j) = msg->pose.covariance[i * 6 + j];
      RnGps(i + 3, j + 3) = msg->twist.covariance[i * 6 + j];
    }
  }
  fla_ukf_.MeasurementUpdateGps(
      z, RnGps, rclcpp::Time(msg->header.stamp, RCL_ROS_TIME));
}

void FLAUKFNodelet::mag_callback(
    const sensor_msgs::msg::MagneticField::ConstSharedPtr msg) {
  if (!static_transforms_initialized_) return;

  auto x = fla_ukf_.GetState();
  const Eigen::Quaterniond q_roll_pitch =
      Eigen::AngleAxisd(x(7), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(x(6), Eigen::Vector3d::UnitX());

  // Using vectornav mag
  const Eigen::Vector3d mag_vec_i = Eigen::Vector3d(
      msg->magnetic_field.x, msg->magnetic_field.y, msg->magnetic_field.z);
  const Eigen::Vector3d mag_vec_b = T_body_cam_.rotation() * mag_vec_i;

  const Eigen::Vector3d mag_vec = q_roll_pitch * mag_vec_b;

  FLAUKF::MeasYawVec z;
  // ENU convention so yaw = 0 is eastwards
  const double mag_yaw = M_PI_2 - std::atan2(mag_vec(1), mag_vec(0));
  z(0) = angles::normalize_angle(mag_yaw - declination_);

  if (mag_buffer_count_ < mag_buffer_size_) {
    mag_buffer_[mag_buffer_count_] = mag_vec_i;
    mag_buffer_count_++;
  } else {
    Eigen::Vector3d avg_mag_vec_i(0, 0, 0);
    for (size_t i = 0; i < mag_buffer_size_; i++) {
      avg_mag_vec_i += mag_buffer_[i];
    }
    avg_mag_vec_i /= mag_buffer_size_;
    mag_buffer_count_ = 0;

    const Eigen::Vector3d avg_mag_vec_b =
        T_body_cam_.rotation() * avg_mag_vec_i;
    const Eigen::Vector3d avg_mag_vec = q_roll_pitch * avg_mag_vec_b;

    const double avg_mag_yaw = std::atan2(avg_mag_vec(1), avg_mag_vec(0));

    const Eigen::Quaterniond R_map_robot{
        Eigen::AngleAxisd(x(8), Eigen::Vector3d::UnitZ())};
    const Eigen::Quaterniond R_robot_mag{
        Eigen::AngleAxisd(avg_mag_yaw, Eigen::Vector3d::UnitZ())};
    const Eigen::Quaterniond R_map_mag = R_map_robot * R_robot_mag;
    const Eigen::Quaterniond R_map_mag_enu =
        Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()) * R_map_mag;

    auto mag_yaw_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
    mag_yaw_msg->header = msg->header;
    mag_yaw_msg->header.frame_id = world_frame_id_;
    mag_yaw_msg->pose.position.x = 0;
    mag_yaw_msg->pose.position.y = 0;
    mag_yaw_msg->pose.position.z = 0;
    mag_yaw_msg->pose.orientation.x = R_map_mag_enu.x();
    mag_yaw_msg->pose.orientation.y = R_map_mag_enu.y();
    mag_yaw_msg->pose.orientation.z = R_map_mag_enu.z();
    mag_yaw_msg->pose.orientation.w = R_map_mag_enu.w();
    pub_mag_yaw_->publish(std::move(mag_yaw_msg));
  }

  FLAUKF::MeasYawCov RnYaw{FLAUKF::MeasYawCov::Zero()};
  RnYaw(0, 0) = angles::from_degrees(10) * angles::from_degrees(10);

  if (enable_mag_)
    fla_ukf_.MeasurementUpdateYaw(
        z, RnYaw, rclcpp::Time(msg->header.stamp, RCL_ROS_TIME));
}

void FLAUKFNodelet::yaw_callback(
    const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr msg) {
  // Get the transform through tf.
  geometry_msgs::msg::TransformStamped stamped_transform_msg;
  try {
    stamped_transform_msg = tf_buffer_->lookupTransform(
        msg->header.frame_id, robot_frame_id_,
        rclcpp::Time(msg->header.stamp, RCL_ROS_TIME),
        rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "yaw_callback tf lookup failed: %s", ex.what());
    return;
  }

  // Convert the tf to eigen.
  Eigen::Quaterniond q_sensor_body;
  tf2::fromMsg(stamped_transform_msg.transform.rotation, q_sensor_body);

  Eigen::Quaterniond q_world_sensor;
  tf2::fromMsg(msg->quaternion, q_world_sensor);
  const Eigen::Quaterniond q_world_body = q_world_sensor * q_sensor_body;
  const Eigen::Vector3d rpy =
      q_world_body.toRotationMatrix().eulerAngles(2, 1, 0).reverse();

  FLAUKF::MeasYawVec z;

  z(0) = rpy(2);

  FLAUKF::MeasYawCov RnYaw;
  RnYaw(0, 0) = angles::from_degrees(1) * angles::from_degrees(1);

  RCLCPP_INFO_STREAM(this->get_logger(), "yaw_callback:, yaw: "
                                             << 180 / M_PI * z.transpose()
                                             << " deg");
  fla_ukf_.MeasurementUpdateYaw(
      z, RnYaw, rclcpp::Time(msg->header.stamp, RCL_ROS_TIME));
}

FLAUKFNodelet::FLAUKFNodelet(const rclcpp::NodeOptions &options)
    : rclcpp::Node("fla_ukf", options) {
  enable_laser_ = this->declare_parameter("enable_laser", true);
  enable_lidar_ = this->declare_parameter("enable_lidar", true);
  enable_gps_ = this->declare_parameter("enable_gps", true);
  enable_height_ = this->declare_parameter("enable_height", true);
  enable_mag_ = this->declare_parameter("enable_mag", true);
  enable_vio_odom_ = this->declare_parameter("enable_vio_odom", true);
  enable_yaw_ = this->declare_parameter("enable_yaw", true);
  declination_ =
      this->declare_parameter("magnetic_declination", angles::from_degrees(-12.12));

  world_frame_id_ =
      this->declare_parameter<std::string>("world_frame_id", "world");
  vision_frame_id_ =
      this->declare_parameter<std::string>("vision_frame_id", "vision");
  lidar_frame_id_ =
      this->declare_parameter<std::string>("lidar_frame_id", "lidar");
  robot_frame_id_ =
      this->declare_parameter<std::string>("robot_frame_id", "base_link");
  cam_frame_id_ = this->declare_parameter<std::string>("cam_frame_id", "cam");

  takeoff_detection_threshold_ =
      this->declare_parameter("takeoff_detection_threshold", 0.2);

  // UKF Parameters
  const double alpha = this->declare_parameter("alpha", 0.1);
  const double beta = this->declare_parameter("beta", 2.0);
  const double kappa = this->declare_parameter("kappa", 0.0);

  // Noise standard devs
  double std_acc[3], std_gyro[3], std_acc_bias[3], std_gyro_bias[3];
  std_acc[0] = this->declare_parameter("noise_std.process.acc.x", 0.1);
  std_acc[1] = this->declare_parameter("noise_std.process.acc.y", 0.1);
  std_acc[2] = this->declare_parameter("noise_std.process.acc.z", 0.1);
  std_gyro[0] = this->declare_parameter("noise_std.process.gyro.x", 0.1);
  std_gyro[1] = this->declare_parameter("noise_std.process.gyro.y", 0.1);
  std_gyro[2] = this->declare_parameter("noise_std.process.gyro.z", 0.1);
  std_acc_bias[0] =
      this->declare_parameter("noise_std.process.acc_bias.x", 0.001);
  std_acc_bias[1] =
      this->declare_parameter("noise_std.process.acc_bias.y", 0.001);
  std_acc_bias[2] =
      this->declare_parameter("noise_std.process.acc_bias.z", 0.001);
  std_gyro_bias[0] =
      this->declare_parameter("noise_std.process.gyro_bias.x", 0.001);
  std_gyro_bias[1] =
      this->declare_parameter("noise_std.process.gyro_bias.y", 0.001);
  std_gyro_bias[2] =
      this->declare_parameter("noise_std.process.gyro_bias.z", 0.001);
  const double vio_yaw_drift =
      this->declare_parameter("noise_std.vio_yaw_drift", 0.01);

  const int cam_delay = this->declare_parameter("cam_delay", 0);
  assert(cam_delay >= 0);
  cam_delay_ = static_cast<unsigned int>(cam_delay);

  const FLAUKF::Scalar_t floor_change_threshold =
      this->declare_parameter("floor_change_threshold", 0.2);
  fla_ukf_.SetFloorChangeThreshold(floor_change_threshold);

  const FLAUKF::Scalar_t floor_merge_threshold =
      this->declare_parameter("floor_merge_threshold", 0.1);
  fla_ukf_.SetFloorMergeThreshold(floor_merge_threshold);

  const FLAUKF::Scalar_t initial_floor_height =
      this->declare_parameter("initial_floor_height", 0.0);
  fla_ukf_.SetCurrentFloorHeight(initial_floor_height);

  publish_tf_ = this->declare_parameter("publish_tf", true);

  // Fixed process noise
  FLAUKF::ProcNoiseCov Rv;
  Rv.setZero();
  Rv(0, 0) = std_acc[0] * std_acc[0];
  Rv(1, 1) = std_acc[1] * std_acc[1];
  Rv(2, 2) = std_acc[2] * std_acc[2];
  Rv(3, 3) = std_gyro[0] * std_gyro[0];
  Rv(4, 4) = std_gyro[1] * std_gyro[1];
  Rv(5, 5) = std_gyro[2] * std_gyro[2];
  Rv(6, 6) = std_acc_bias[0] * std_acc_bias[0];
  Rv(7, 7) = std_acc_bias[1] * std_acc_bias[1];
  Rv(8, 8) = std_acc_bias[2] * std_acc_bias[2];
  Rv(9, 9) = std_gyro_bias[0] * std_gyro_bias[0];
  Rv(10, 10) = std_gyro_bias[1] * std_gyro_bias[1];
  Rv(11, 11) = std_gyro_bias[2] * std_gyro_bias[2];
  Rv(12, 12) = vio_yaw_drift * vio_yaw_drift;

  // Initialize UKF
  fla_ukf_.SetParameters(alpha, beta, kappa);
  fla_ukf_.SetImuCovariance(Rv);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pub_ukf_odom_ =
      this->create_publisher<nav_msgs::msg::Odometry>("odom_out", 10);
  pub_mag_yaw_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("mag_yaw", 10);

  // imu
  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10,
      std::bind(&FLAUKFNodelet::imu_callback, this, std::placeholders::_1));
  if (enable_lidar_)
    sub_lidar_ =
        this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "pose_lidar", 10,
            std::bind(&FLAUKFNodelet::lidar_callback, this,
                      std::placeholders::_1));
  if (enable_height_) {
    sub_height_ = this->create_subscription<sensor_msgs::msg::Range>(
        "height", 10,
        std::bind(&FLAUKFNodelet::height_callback, this,
                  std::placeholders::_1));
    sub_height_toggle_ = this->create_subscription<std_msgs::msg::Bool>(
        "use_downward_lidar", 10,
        std::bind(&FLAUKFNodelet::laser_toggle_callback, this,
                  std::placeholders::_1));
  }

  if (enable_gps_)
    sub_gps_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "gps_odom", 10,
        std::bind(&FLAUKFNodelet::gps_callback, this, std::placeholders::_1));
  // Always enable mag subscriber so that we publish the mag yaw
  sub_mag_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      "mag", 10,
      std::bind(&FLAUKFNodelet::mag_callback, this, std::placeholders::_1));
  if (enable_vio_odom_)
    sub_vio_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "vio_odom", 10,
        std::bind(&FLAUKFNodelet::vio_odom_callback, this,
                  std::placeholders::_1));
  if (enable_yaw_) {
    sub_yaw_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
        "yaw", 10,
        std::bind(&FLAUKFNodelet::yaw_callback, this, std::placeholders::_1));
  }
}

}  // namespace fla_ukf

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(fla_ukf::FLAUKFNodelet)
