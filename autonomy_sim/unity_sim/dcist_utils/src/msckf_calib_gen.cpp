#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.hpp>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

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
                const sensor_msgs::msg::CameraInfo& cinfo,
                const Eigen::Matrix4d& T_cam_imu,
                const std::optional<Eigen::Matrix4d> T_cn_cnm1 = std::nullopt) {
  *out << YAML::Value;
  *out << YAML::BeginMap;

  // transform
  EmitMatrixRowwise(out, "T_cam_imu", T_cam_imu);

  *out << YAML::Key << "camera_model" << YAML::Value << "pinhole";
  EmitStdVector<double>(
      out, "intrinsics", {cinfo.k[0], cinfo.k[4], cinfo.k[2], cinfo.k[5]});
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

geometry_msgs::msg::TransformStamped LookupTransform(
    const tf2_ros::Buffer& buffer,
    const std::string& target_frame,
    const std::string& source_frame) {
  return buffer.lookupTransform(target_frame, source_frame,
                                tf2::TimePointZero);
}

/**
 * @brief This class subscribes to camera info topics of a pair of stereo
 * cameras and tf and outputs a calib.yaml file which can be used with
 * msckf_vio.
 */
class CalibGenerator : public rclcpp::Node {
 public:
  CalibGenerator()
      : Node("msckf_calib_gen"),
        buffer_(this->get_clock()),
        listener_(buffer_) {
    output_file_ = this->declare_parameter<std::string>(
        "output", "/tmp/calib.yaml");

    sub_cinfo0_ =
        this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "cam0/camera_info", 1,
            std::bind(&CalibGenerator::Cinfo0Cb, this,
                      std::placeholders::_1));
    sub_cinfo1_ =
        this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "cam1/camera_info", 1,
            std::bind(&CalibGenerator::Cinfo1Cb, this,
                      std::placeholders::_1));
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 1,
        std::bind(&CalibGenerator::ImuCb, this, std::placeholders::_1));
  }

  void Cinfo0Cb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr cinfo_msg) {
    RCLCPP_INFO(this->get_logger(), "Got cinfo0");
    cinfo0_ = *cinfo_msg;
    sub_cinfo0_.reset();
  }

  void Cinfo1Cb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr cinfo_msg) {
    RCLCPP_INFO(this->get_logger(), "Got cinfo1");
    cinfo1_ = *cinfo_msg;
    sub_cinfo1_.reset();
  }

  void ImuCb(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) {
    if (cinfo0_.header.frame_id.empty() || cinfo1_.header.frame_id.empty()) {
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Waiting for camera info, maybe you need to add a rqt to visualize "
          "image topic so that it gets published!");
      return;
    }

    tf_cam0_imu_ = LookupTransform(
        buffer_, cinfo0_.header.frame_id, imu_msg->header.frame_id);
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "T_cam0_imu: \n"
            << tf2::transformToEigen(tf_cam0_imu_.transform).matrix());

    tf_cam1_imu_ = LookupTransform(
        buffer_, cinfo1_.header.frame_id, imu_msg->header.frame_id);
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "T_cam1_imu: \n"
            << tf2::transformToEigen(tf_cam1_imu_.transform).matrix());

    WriteYaml(output_file_);
    RCLCPP_WARN(this->get_logger(), "Write calib file to: %s",
                output_file_.c_str());
    rclcpp::shutdown();
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
    const auto T_cam1_imu = tf2::transformToEigen(tf_cam1_imu_.transform);
    const auto T_cam1_cam0 = T_cam1_imu * T_cam0_imu.inverse();
    EmitCamera(&out, cinfo1_, T_cam1_imu.matrix(), {T_cam1_cam0.matrix()});
    out << YAML::EndMap;
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cinfo0_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cinfo1_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  std::string output_file_;
  std::string robot_;
  sensor_msgs::msg::CameraInfo cinfo0_, cinfo1_;
  geometry_msgs::msg::TransformStamped tf_cam0_imu_, tf_cam1_imu_;
};

}  // namespace dcist

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dcist::CalibGenerator>());
  rclcpp::shutdown();
  return 0;
}
