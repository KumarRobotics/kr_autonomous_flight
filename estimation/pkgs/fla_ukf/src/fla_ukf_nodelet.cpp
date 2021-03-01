// Copyright 2016 KumarRobotics - Kartik Mohta
#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <limits>
#include <memory>
#include <queue>
#include <string>

#include "fla_ukf/fla_ukf.h"

class FLAUKFNodelet : public nodelet::Nodelet {
 public:
  FLAUKFNodelet();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;  // Need this since we have FLAUKF which
                                    // needs aligned pointer
 private:
  void onInit(void) override;
  void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
  void cam_callback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void velodyne_callback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void laser_callback(const nav_msgs::Odometry::ConstPtr &msg);
  void height_callback(const sensor_msgs::Range::ConstPtr &msg);
  void gps_callback(const nav_msgs::Odometry::ConstPtr &msg);
  void mag_callback(const sensor_msgs::MagneticField::ConstPtr &msg);
  void vio_odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
  void laser_toggle_callback(const std_msgs::Bool::ConstPtr &msg);
  void yaw_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg);

  FLAUKF fla_ukf_;
  ros::Subscriber sub_imu_, sub_cam_, sub_laser_, sub_height_, sub_velodyne_,
      sub_gps_, sub_mag_, sub_vio_odom_, sub_height_toggle_, sub_yaw_;
  ros::Publisher pub_ukf_odom_, pub_mag_yaw_;
  std::string world_frame_id_, cam_frame_id_, robot_frame_id_, vision_frame_id_,
      velodyne_frame_id_;
  unsigned int cam_delay_;
  std::queue<sensor_msgs::Imu> imu_queue_;
  static constexpr int imu_calib_limit_ = 100;
  int imu_calib_count_ = 0;
  Eigen::Vector3d acc_gravity_ = Eigen::Vector3d::Zero();
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  Eigen::Isometry3d T_cam_body_, T_body_cam_, T_world_vision_, T_velodyne_body_;
  bool static_transforms_initialized_ = false;
  bool takeoff_detected_ = false;
  bool height_measurement_received_ = false;
  double takeoff_detection_threshold_;
  bool ignore_laser_ = false;
  bool ignore_height_ = false;
  boost::optional<Eigen::Isometry3d> T_world_velodyneMap_;
  bool enable_cam_, enable_velodyne_, enable_laser_, enable_height_,
      enable_gps_, enable_mag_, enable_vio_odom_, enable_yaw_;
  double declination_;
  static constexpr size_t mag_buffer_size_ = 100;
  std::array<Eigen::Vector3d, mag_buffer_size_> mag_buffer_;
  size_t mag_buffer_count_ = 0;
  ros::Time last_vio_timestamp_;
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

FLAUKFNodelet::FLAUKFNodelet() {}
void FLAUKFNodelet::imu_callback(const sensor_msgs::Imu::ConstPtr &msg) {
  if (!static_transforms_initialized_) {
    geometry_msgs::TransformStamped transform_cam_body, transform_world_vision,
        transform_velodyne_body;
    try {
      transform_cam_body = tf_buffer_.lookupTransform(
          cam_frame_id_, robot_frame_id_, ros::Time(0));  // Used for front IMU
      if (enable_cam_ || enable_vio_odom_) {
        transform_world_vision = tf_buffer_.lookupTransform(
            world_frame_id_, vision_frame_id_, ros::Time(0));
      }
      if (enable_velodyne_) {
        transform_velodyne_body = tf_buffer_.lookupTransform(
            velodyne_frame_id_, robot_frame_id_, ros::Time(0));
      }
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
    }
    tf::transformMsgToEigen(transform_cam_body.transform, T_cam_body_);
    T_body_cam_ = T_cam_body_.inverse();
    tf::transformMsgToEigen(transform_world_vision.transform, T_world_vision_);
    tf::transformMsgToEigen(transform_velodyne_body.transform,
                            T_velodyne_body_);
    static_transforms_initialized_ = true;
  }

  imu_queue_.push(*msg);
  if (imu_queue_.size() <= cam_delay_) return;

  if (msg->header.stamp - last_vio_timestamp_ >= ros::Duration(0.5)) {
    ROS_WARN_THROTTLE(0.05,
                      "========= DANGER!!! DANGER!!! No recent VIO "
                      "messages received =========");
  }

  const sensor_msgs::Imu imu_msg = imu_queue_.front();
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

    ROS_INFO("Setting gravity to %f", g);
    fla_ukf_.SetGravity(g);
  } else if (fla_ukf_.ProcessUpdate(u, imu_msg.header.stamp)) {
    auto odom_ukf_msg = boost::make_shared<nav_msgs::Odometry>();
    odom_ukf_msg->header.stamp = imu_msg.header.stamp;
    odom_ukf_msg->header.frame_id = world_frame_id_;
    odom_ukf_msg->child_frame_id = robot_frame_id_;
    auto x = fla_ukf_.GetState();
#if ENABLE_VIO_YAW_OFFSET
    ROS_INFO_STREAM_THROTTLE(
        0.1, "VIO Yaw offset: " << x(15) * 180 / M_PI << " deg");
#endif
    // ROS_INFO_STREAM("t_world_body (filter): " <<
    // x.segment<3>(0).transpose());
    // ROS_INFO_STREAM("rpy_world_body (filter): "
    //                 << 180 / M_PI * x.segment<3>(6).transpose());
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
    // Publish Msg
    pub_ukf_odom_.publish(odom_ukf_msg);

    if (publish_tf_) {
      // Publish TF
      geometry_msgs::TransformStamped transform_msg;
      transform_msg.header = odom_ukf_msg->header;
      transform_msg.child_frame_id = odom_ukf_msg->child_frame_id;
      transform_msg.transform.translation.x =
          odom_ukf_msg->pose.pose.position.x;
      transform_msg.transform.translation.y =
          odom_ukf_msg->pose.pose.position.y;
      transform_msg.transform.translation.z =
          odom_ukf_msg->pose.pose.position.z;
      transform_msg.transform.rotation = odom_ukf_msg->pose.pose.orientation;
      tf_broadcaster_.sendTransform(transform_msg);
    }
  }
}

void FLAUKFNodelet::velodyne_callback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  if (!height_measurement_received_)
    NODELET_ERROR_THROTTLE(1, "No height measurement received!");

  // Reset height_measurement_received_ to check for height measurements between
  // each camera pose
  height_measurement_received_ = false;

  if (!T_world_velodyneMap_) {
    auto x = fla_ukf_.GetState();
    const Eigen::Vector3d translation{x(0), x(1), x(2)};
    const Eigen::Quaterniond rotation =
        Eigen::AngleAxisd(x(8), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(x(7), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(x(6), Eigen::Vector3d::UnitX());
    T_world_velodyneMap_->linear() = rotation.toRotationMatrix();
    T_world_velodyneMap_->translation() = translation;
  }

  Eigen::Isometry3d T_velodyneMap_velodyne;
  tf::poseMsgToEigen(msg->pose.pose, T_velodyneMap_velodyne);

  const Eigen::Isometry3d T_world_body =
      T_world_velodyneMap_.get() * T_velodyneMap_velodyne * T_velodyne_body_;

  // ROS_INFO_STREAM("T_velodyne_body:\n" << T_velodyne_body_.matrix());
  // ROS_INFO_STREAM("T_velodyneMap_velodyne:\n" <<
  // T_velodyneMap_velodyne.matrix());
  // ROS_INFO_STREAM("T_world_body:\n" << T_world_body.matrix());

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
  FLAUKF::MeasCamCov RnVelodyne(FLAUKF::MeasCamCov::Zero());
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      const double cov_multiplier = ((i < 3) ? 5 : 25) * ((j < 3) ? 5 : 25);
      RnVelodyne(i, j) = cov_multiplier * msg->pose.covariance[j + i * 6];
    }
  }
  std::cout << "RnVelodyne: " << RnVelodyne.diagonal().transpose() << std::endl;

  // Measurement update
  if (fla_ukf_.MeasurementUpdateCam(z, RnVelodyne, msg->header.stamp))
    ignore_laser_ = true;
}

void FLAUKFNodelet::cam_callback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  if (!height_measurement_received_)
    NODELET_ERROR_THROTTLE(1, "No height measurement received!");

  // Reset height_measurement_received_ to check for height measurements between
  // each camera pose
  height_measurement_received_ = false;

  Eigen::Isometry3d T_vision_cam;
  tf::poseMsgToEigen(msg->pose.pose, T_vision_cam);

  const Eigen::Isometry3d T_world_body =
      T_world_vision_ * T_vision_cam * T_cam_body_;

  // ROS_INFO_STREAM("T_cam_body:\n" << T_cam_body_.matrix());
  // ROS_INFO_STREAM("T_vision_cam:\n" << T_vision_cam.matrix());
  // ROS_INFO_STREAM("T_world_body:\n" << T_world_body.matrix());

  const Eigen::Quaterniond q_world_body(T_world_body.rotation());
  const Eigen::Vector3d t_world_body(T_world_body.translation());

#if 0
  double q[] = {q_world_body.w(), q_world_body.x(), q_world_body.y(),
                q_world_body.z()};
  double yaw = std::atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                          1 - 2 * (q[2] * q[2] + q[3] * q[3]));
  double pitch = std::asin(2 * (q[0] * q[2] - q[3] * q[1]));
  double roll = std::atan2(2 * (q[0] * q[1] + q[2] * q[3]),
                           1 - 2 * (q[1] * q[1] + q[2] * q[2]));
#endif
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
#if 0
  // NOTE(Kartik): SVO has junk covariance right now
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      RnCam(i, j) = msg->pose.covariance[j + i * 6];
#endif

  // Measurement update
  fla_ukf_.MeasurementUpdateCam(z, RnCam, msg->header.stamp);
}
void FLAUKFNodelet::laser_toggle_callback(const std_msgs::Bool::ConstPtr &msg) {
  ignore_height_ = !msg->data;
  ROS_INFO_STREAM("Ignore laser: " << ignore_height_);
}

void FLAUKFNodelet::laser_callback(const nav_msgs::Odometry::ConstPtr &msg) {
  // NODELET_INFO("laser_callback");
  if (ignore_laser_) return;

  FLAUKF::MeasLaserVec z;
  double yaw = tf2::getYaw(msg->pose.pose.orientation);
  z(0) = msg->pose.pose.position.x;
  z(1) = msg->pose.pose.position.y;
  z(2) = yaw;

  // Assemble measurement covariance
  FLAUKF::MeasLaserCov RnLaser{FLAUKF::MeasLaserCov::Zero()};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      int row_idx = (i < 2) ? i : i + 1;  // Skip 3rd row
      int col_idx = (j < 2) ? j : j + 1;  // Skip 3rd col
      RnLaser(i, j) = msg->pose.covariance[row_idx + col_idx * 6];
    }
  }
  RnLaser.block<2, 2>(0, 0) *= 50 * 50;  // Inflate covariance for XY
  fla_ukf_.MeasurementUpdateLaser(z, RnLaser, msg->header.stamp);
}

void FLAUKFNodelet::height_callback(const sensor_msgs::Range::ConstPtr &msg) {
  if (ignore_height_) return;
  // NODELET_INFO("height_callback, range: %f", msg->range);
  height_measurement_received_ = true;

  float range = msg->range;
  float range_std_dev = 0.05;

  if (!takeoff_detected_ && range > takeoff_detection_threshold_) {
    takeoff_detected_ = true;
    ROS_WARN_STREAM("Takeoff detected! range: " << range);
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
    // ROS_INFO_STREAM("Height meas: " << z(0));

    // Assemble measurement covariance
    FLAUKF::MeasHeightCov RnHeight{FLAUKF::MeasHeightCov::Zero()};
    RnHeight(0, 0) = range_std_dev * range_std_dev;
    fla_ukf_.MeasurementUpdateHeight(z, RnHeight, msg->header.stamp);
  }
}

void FLAUKFNodelet::vio_odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {
  last_vio_timestamp_ = msg->header.stamp;

  Eigen::Isometry3d T_vision_cam;
  tf::poseMsgToEigen(msg->pose.pose, T_vision_cam);

  Eigen::Vector3d v_vision_cam;
  tf::vectorMsgToEigen(msg->twist.twist.linear, v_vision_cam);

  const Eigen::Isometry3d T_world_body =
      T_world_vision_ * T_vision_cam * T_cam_body_;

  const Eigen::Quaterniond q_world_body(T_world_body.rotation());
  const Eigen::Vector3d t_world_body(T_world_body.translation());
  const Eigen::Vector3d v_world_body =
      T_world_vision_.linear() * v_vision_cam;  // Ignoring the (omega \times r)
                                                // since r_cam_body is small

  const Eigen::Vector3d rpy = quatToEulerZYX(q_world_body);

  // ROS_INFO_STREAM("t_world_body (vio): " << t_world_body.transpose());
  // ROS_INFO_STREAM("rpy_world_body (vio): " << 180/M_PI * rpy.transpose());

#if VIO_NO_POSITION
  // NOTE(Kartik): Only using orientation and velocity

  // Assemble measurement
  FLAUKF::MeasVioVec z;
  z(0) = v_world_body(0);
  z(1) = v_world_body(1);
  z(2) = v_world_body(2);
  z(3) = rpy(0);
  z(4) = rpy(1);
  z(5) = rpy(2);

  // Assemble measurement covariance
  FLAUKF::MeasVioCov RnVio(FLAUKF::MeasVioCov::Zero());
  // Pose covariance
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      const int row_idx = i + 3;
      const int col_idx = j + 3;

      // Remove cross covariance with Z and increase Z covariance
      // const double cov_multiplier = (i == 2 || j == 2) ? 0 : 1;
      // const double cov_inflater  = (i == 2 && j == 2) ? 100 : 0;

      RnVio(row_idx, col_idx) = msg->pose.covariance[i * 6 + j];
    }
  }
  // Linear velocity covariance
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      RnVio(i, j) = msg->twist.covariance[i * 6 + j];
    }
  }
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

  // Assemble measurement covariance
  FLAUKF::MeasVioCov RnVio(FLAUKF::MeasVioCov::Zero());
  // Pose covariance
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      const int row_idx = (i < 3) ? i : i + 3;
      const int col_idx = (j < 3) ? j : j + 3;

      // Remove cross covariance with Z and increase Z covariance
      const double cov_multiplier = (i == 2 || j == 2) ? 0 : 1;
      const double cov_inflater = (i == 2 && j == 2) ? 100 : 0;

      RnVio(row_idx, col_idx) =
          cov_multiplier * msg->pose.covariance[i * 6 + j] + cov_inflater;
    }
  }
  // Linear velocity covariance
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      RnVio(3 + i, 3 + j) = msg->twist.covariance[i * 6 + j];
    }
  }

  for (int k = 0; k < 2; ++k) {  // Only need to check for position X/Y
    // Keep the VIO covariance within a reasonable band so that we don't trust
    // it too much or too less
    double const vio_pos_min_std = 1.0;
    double const vio_pos_max_std = 3.0;

    double scale_factor = 1;
    if (RnVio(k, k) == 0)
      RnVio(k, k) = vio_pos_min_std * vio_pos_min_std;
    else if (RnVio(k, k) > vio_pos_max_std * vio_pos_max_std)
      scale_factor = vio_pos_max_std / std::sqrt(RnVio(k, k));
    else if (RnVio(k, k) < vio_pos_min_std * vio_pos_min_std)
      scale_factor = vio_pos_min_std / std::sqrt(RnVio(k, k));

    if (scale_factor != 1) {
      for (int i = 0; i < RnVio.rows(); ++i) {
        RnVio(i, k) *= scale_factor;
        RnVio(k, i) *= scale_factor;
      }
    }
  }
#endif

  // Measurement update
  if (fla_ukf_.MeasurementUpdateVio(z, RnVio, msg->header.stamp))
    ignore_laser_ = true;
}

void FLAUKFNodelet::gps_callback(const nav_msgs::Odometry::ConstPtr &msg) {
  // NODELET_INFO("gps_callback");

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
  fla_ukf_.MeasurementUpdateGps(z, RnGps, msg->header.stamp);
}

void FLAUKFNodelet::mag_callback(
    const sensor_msgs::MagneticField::ConstPtr &msg) {
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
  // NODELET_INFO_STREAM_THROTTLE(1, "Yaw from mag: " <<
  // angles::to_degrees(z(0)));

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
    // ROS_INFO_STREAM("Average mag yaw: " << avg_mag_yaw);

    const Eigen::Quaterniond R_map_robot{
        Eigen::AngleAxisd(x(8), Eigen::Vector3d::UnitZ())};
    const Eigen::Quaterniond R_robot_mag{
        Eigen::AngleAxisd(avg_mag_yaw, Eigen::Vector3d::UnitZ())};
    const Eigen::Quaterniond R_map_mag = R_map_robot * R_robot_mag;
    const Eigen::Quaterniond R_map_mag_enu =
        Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()) * R_map_mag;

    auto mag_yaw_msg = boost::make_shared<geometry_msgs::PoseStamped>();
    mag_yaw_msg->header = msg->header;
    mag_yaw_msg->header.frame_id = world_frame_id_;
    mag_yaw_msg->pose.position.x = 0;
    mag_yaw_msg->pose.position.y = 0;
    mag_yaw_msg->pose.position.z = 0;
    mag_yaw_msg->pose.orientation.x = R_map_mag_enu.x();
    mag_yaw_msg->pose.orientation.y = R_map_mag_enu.y();
    mag_yaw_msg->pose.orientation.z = R_map_mag_enu.z();
    mag_yaw_msg->pose.orientation.w = R_map_mag_enu.w();
    pub_mag_yaw_.publish(mag_yaw_msg);
  }

  FLAUKF::MeasYawCov RnYaw{FLAUKF::MeasYawCov::Zero()};
  RnYaw(0, 0) = angles::from_degrees(10) * angles::from_degrees(10);

  if (enable_mag_) fla_ukf_.MeasurementUpdateYaw(z, RnYaw, msg->header.stamp);
}

void FLAUKFNodelet::yaw_callback(
    const geometry_msgs::QuaternionStamped::ConstPtr &msg) {
  // Get the transform through tf.
  geometry_msgs::TransformStamped stamped_transform_msg =
      tf_buffer_.lookupTransform(msg->header.frame_id, robot_frame_id_,
                                 msg->header.stamp, ros::Duration(0.1));

  // Convert the tf to eigen.
  Eigen::Quaterniond q_sensor_body;
  tf2::convert(stamped_transform_msg.transform.rotation, q_sensor_body);

  Eigen::Quaterniond q_world_sensor;
  tf2::convert(msg->quaternion, q_world_sensor);
  const Eigen::Quaterniond q_world_body = q_world_sensor * q_sensor_body;
  const Eigen::Vector3d rpy =
      q_world_body.toRotationMatrix().eulerAngles(2, 1, 0).reverse();

  FLAUKF::MeasYawVec z;

  z(0) = rpy(2);

  FLAUKF::MeasYawCov RnYaw;
  RnYaw(0, 0) = angles::from_degrees(1) * angles::from_degrees(1);

  ROS_INFO_STREAM("yaw_callback:, yaw: " << 180 / M_PI * z.transpose()
                                         << " deg");
  fla_ukf_.MeasurementUpdateYaw(z, RnYaw, msg->header.stamp);
}

void FLAUKFNodelet::onInit(void) {
  ros::NodeHandle n(getPrivateNodeHandle());

  n.param("enable_laser", enable_laser_, true);
  n.param("enable_cam", enable_cam_, true);
  n.param("enable_velodyne", enable_velodyne_, true);
  n.param("enable_gps", enable_gps_, true);
  n.param("enable_height", enable_height_, true);
  n.param("enable_mag", enable_mag_, true);
  n.param("enable_vio_odom", enable_vio_odom_, true);
  n.param("enable_yaw", enable_yaw_, true);
  n.param("magnetic_declination", declination_, angles::from_degrees(-12.12));

  n.param("world_frame_id", world_frame_id_, std::string("world"));
  n.param("vision_frame_id", vision_frame_id_, std::string("vision"));
  n.param("velodyne_frame_id", velodyne_frame_id_, std::string("velodyne"));
  n.param("robot_frame_id", robot_frame_id_, std::string("base_link"));
  n.param("cam_frame_id", cam_frame_id_, std::string("cam"));

  n.param("takeoff_detection_threshold", takeoff_detection_threshold_, 0.2);

  // UKF Parameters
  double alpha, beta, kappa;
  n.param("alpha", alpha, 0.1);
  n.param("beta", beta, 2.0);
  n.param("kappa", kappa, 0.0);

  // Noise standard devs
  double std_acc[3], std_gyro[3], std_acc_bias[3], std_gyro_bias[3],
      vio_yaw_drift;
  n.param("noise_std/process/acc/x", std_acc[0], 0.1);
  n.param("noise_std/process/acc/y", std_acc[1], 0.1);
  n.param("noise_std/process/acc/z", std_acc[2], 0.1);
  n.param("noise_std/process/gyro/x", std_gyro[0], 0.1);
  n.param("noise_std/process/gyro/y", std_gyro[1], 0.1);
  n.param("noise_std/process/gyro/z", std_gyro[2], 0.1);
  n.param("noise_std/process/acc_bias/x", std_acc_bias[0], 0.001);
  n.param("noise_std/process/acc_bias/y", std_acc_bias[1], 0.001);
  n.param("noise_std/process/acc_bias/z", std_acc_bias[2], 0.001);
  n.param("noise_std/process/gyro_bias/x", std_gyro_bias[0], 0.001);
  n.param("noise_std/process/gyro_bias/y", std_gyro_bias[1], 0.001);
  n.param("noise_std/process/gyro_bias/z", std_gyro_bias[2], 0.001);
  n.param("noise_std/vio_yaw_drift", vio_yaw_drift, 0.01);

  int cam_delay;
  n.param("cam_delay", cam_delay, 0);
  assert(cam_delay >= 0);
  cam_delay_ = cam_delay;

  FLAUKF::Scalar_t floor_change_threshold;
  n.param("floor_change_threshold", floor_change_threshold, 0.2);
  fla_ukf_.SetFloorChangeThreshold(floor_change_threshold);

  FLAUKF::Scalar_t floor_merge_threshold;
  n.param("floor_merge_threshold", floor_merge_threshold, 0.1);
  fla_ukf_.SetFloorMergeThreshold(floor_merge_threshold);

  FLAUKF::Scalar_t initial_floor_height;
  n.param("initial_floor_height", initial_floor_height, 0.0);
  fla_ukf_.SetCurrentFloorHeight(initial_floor_height);

  n.param("publish_tf", publish_tf_, true);

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

  tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

  pub_ukf_odom_ = n.advertise<nav_msgs::Odometry>("odom_out", 10);
  pub_mag_yaw_ = n.advertise<geometry_msgs::PoseStamped>("mag_yaw", 10);

  sub_imu_ = n.subscribe("imu", 10, &FLAUKFNodelet::imu_callback, this,
                         ros::TransportHints().tcpNoDelay());
  if (enable_cam_)
    sub_cam_ = n.subscribe("pose_cam", 10, &FLAUKFNodelet::cam_callback, this,
                           ros::TransportHints().tcpNoDelay());
  if (enable_velodyne_)
    sub_velodyne_ =
        n.subscribe("pose_velodyne", 10, &FLAUKFNodelet::velodyne_callback,
                    this, ros::TransportHints().tcpNoDelay());
  if (enable_laser_)
    sub_laser_ = n.subscribe("laser_odom", 10, &FLAUKFNodelet::laser_callback,
                             this, ros::TransportHints().tcpNoDelay());
  if (enable_height_) {
    sub_height_ = n.subscribe("height", 10, &FLAUKFNodelet::height_callback,
                              this, ros::TransportHints().tcpNoDelay());
    sub_height_toggle_ = n.subscribe("use_downward_lidar", 10,
                                     &FLAUKFNodelet::laser_toggle_callback,
                                     this, ros::TransportHints().tcpNoDelay());
  }

  if (enable_gps_)
    sub_gps_ = n.subscribe("gps_odom", 10, &FLAUKFNodelet::gps_callback, this,
                           ros::TransportHints().tcpNoDelay());
  // Always enable mag subscriber so that we publish the mag yaw
  sub_mag_ = n.subscribe("mag", 10, &FLAUKFNodelet::mag_callback, this,
                         ros::TransportHints().tcpNoDelay());
  if (enable_vio_odom_)
    sub_vio_odom_ =
        n.subscribe("vio_odom", 10, &FLAUKFNodelet::vio_odom_callback, this,
                    ros::TransportHints().tcpNoDelay());
  if (enable_yaw_) {
    sub_yaw_ = n.subscribe("yaw", 10, &FLAUKFNodelet::yaw_callback, this,
                           ros::TransportHints().tcpNoDelay());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(FLAUKFNodelet, nodelet::Nodelet)
