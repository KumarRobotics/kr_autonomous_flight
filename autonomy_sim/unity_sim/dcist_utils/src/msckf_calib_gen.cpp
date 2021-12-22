#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>
#include <optional>

namespace dcist {

/// @brief Emit key value pair of std::vector
template <typename T>
void EmitStdVector(YAML::Emitter* out,
                   const std::string& key,
                   const std::vector<T>& values) {
  *out << YAML::Key << key;
  *out << YAML::Value << YAML::Flow;
  *out << YAML::BeginSeq;
  for (const auto& v : values) *out << v;
  *out << YAML::EndSeq;
}

/// @brief Emit key value pair of eigen matrix, row major traversal
template <typename Derived>
void EmitMatrixRowwise(YAML::Emitter* out,
                       const std::string& key,
                       const Eigen::MatrixBase<Derived>& mat) {
  *out << YAML::Key << key;
  *out << YAML::Value << YAML::Flow;
  *out << YAML::BeginSeq;
  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      *out << mat(i, j);
    }
  }
  *out << YAML::EndSeq;
}

void EmitCamera(YAML::Emitter* out,
                const sensor_msgs::CameraInfo& cinfo,
                const Eigen::Matrix4d& T_cam_imu,
                const std::optional<Eigen::Matrix4d> T_cn_cnm1 = std::nullopt) {
  *out << YAML::Value;
  *out << YAML::BeginMap;

  // transform
  EmitMatrixRowwise(out, "T_cam_imu", T_cam_imu);

  *out << YAML::Key << "camera_model" << YAML::Value << "pinhole";
  EmitStdVector<double>(
      out, "intrinsics", {cinfo.K[0], cinfo.K[4], cinfo.K[2], cinfo.K[5]});
  *out << YAML::Key << "distortion_model" << YAML::Value << "radtan";
  EmitStdVector(out, "distortion_coeffs", std::vector<double>(4, 0.0));
  EmitStdVector<int>(
      out,
      "resolution",
      {static_cast<int>(cinfo.width), static_cast<int>(cinfo.height)});

  *out << YAML::Key << "rostopic" << YAML::Value
       << "/" + cinfo.header.frame_id + "/image_raw";

  if (T_cn_cnm1) {
    EmitMatrixRowwise(out, "T_cn_cnm1", *T_cn_cnm1);
  }

  *out << YAML::EndMap;
}

geometry_msgs::TransformStamped LookupTransform(
    const tf2_ros::Buffer& buffer,
    const std::string& target_frame,
    const std::string& source_frame) {
  return buffer.lookupTransform(target_frame, source_frame, ros::Time(0));
}

/**
 * @brief This class subscribe to camera info topics of a pair of stereo cameras
 * and tf and output a calib.yaml file which can be used with msckf_vio
 */
class CalibGenerator {
 public:
  CalibGenerator(const ros::NodeHandle& pnh)
      : pnh_(pnh),
        sub_cinfo0_(pnh_.subscribe(
            "cam0/camera_info", 1, &CalibGenerator::Cinfo0Cb, this)),
        sub_cinfo1_(pnh_.subscribe(
            "cam1/camera_info", 1, &CalibGenerator::Cinfo1Cb, this)),
        sub_imu_(pnh_.subscribe("imu", 1, &CalibGenerator::ImuCb, this)),
        listener_(buffer_),
        output_file_(pnh_.param<std::string>("output", "/tmp/calib.yaml")) {}

  void Cinfo0Cb(const sensor_msgs::CameraInfo& cinfo_msg) {
    ROS_INFO("Got cinfo0");
    cinfo0_ = cinfo_msg;
    sub_cinfo0_.shutdown();
  }

  void Cinfo1Cb(const sensor_msgs::CameraInfo& cinfo_msg) {
    ROS_INFO("Got cinfo1");
    cinfo1_ = cinfo_msg;
    sub_cinfo1_.shutdown();
  }

  void ImuCb(const sensor_msgs::Imu& imu_msg) {
    if (cinfo0_.header.frame_id.empty() || cinfo1_.header.frame_id.empty()) {
      ROS_INFO_THROTTLE(1,
                        "Waiting for camera info, maybe you need to add a rqt "
                        "to visualize image topic so that it gets published!");
      return;
    }

    tf_cam0_imu_ = LookupTransform(
        buffer_, cinfo0_.header.frame_id, imu_msg.header.frame_id);
    ROS_INFO_STREAM("T_cam0_imu: \n"
                    << tf2::transformToEigen(tf_cam0_imu_.transform).matrix());

    tf_cam1_imu_ = LookupTransform(
        buffer_, cinfo1_.header.frame_id, imu_msg.header.frame_id);
    ROS_INFO_STREAM("T_cam1_imu: \n"
                    << tf2::transformToEigen(tf_cam1_imu_.transform).matrix());

    WriteYaml(output_file_);
    ROS_WARN("Write calib file to: %s", output_file_.c_str());
    ros::shutdown();
  }

  void WriteYaml(const std::string& filename) const {
    std::ofstream of(filename);
    YAML::Emitter out(of);

    out.SetIndent(4);

    out << YAML::BeginMap;
    out << YAML::Key << "cam0";
    const auto T_cam0_imu = tf2::transformToEigen(tf_cam0_imu_.transform);
    EmitCamera(&out, cinfo0_, T_cam0_imu.matrix());

    out << YAML::Key << "cam1";
    // compute T_cam1_cam0
    const auto T_cam1_imu = tf2::transformToEigen(tf_cam1_imu_.transform);
    const auto T_cam1_cam0 = T_cam1_imu * T_cam0_imu.inverse();
    EmitCamera(&out, cinfo1_, T_cam1_imu.matrix(), {T_cam1_cam0.matrix()});
    out << YAML::EndMap;
  }

  ros::NodeHandle pnh_;
  ros::Subscriber sub_cinfo0_, sub_cinfo1_;
  ros::Subscriber sub_imu_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  std::string output_file_;
  std::string robot_;
  sensor_msgs::CameraInfo cinfo0_, cinfo1_;
  geometry_msgs::TransformStamped tf_cam0_imu_, tf_cam1_imu_;
};

}  // namespace dcist

using namespace dcist;

int main(int argc, char** argv) {
  ros::init(argc, argv, "msckf_calib_gen");

  ros::NodeHandle pnh("~");
  CalibGenerator gen(pnh);
  ros::spin();

  return 0;
}