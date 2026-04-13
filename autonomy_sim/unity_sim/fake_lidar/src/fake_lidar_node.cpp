#include <cv_bridge/cv_bridge.hpp>
#include <image_geometry/pinhole_camera_model.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.hpp>

#include <boost/timer/timer.hpp>
#include <cmath>
#include <iostream>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <string>

#include "utils.h"

// Timer stuff
using boost::timer::cpu_timer;
using boost::timer::cpu_times;
cpu_timer timer;

namespace fake_lidar {

class DepthCamera {
 public:
  explicit DepthCamera(double lb, double ub, double cc, size_t idx)
      : lower_bound(lb), upper_bound(ub), cam_center(cc), cam_id(idx){};

  double lower_bound;
  double upper_bound;
  double cam_center;
  size_t cam_id;
};

template <typename PointT>
pcl::PointCloud<PointT> TransformCloud(
    const pcl::PointCloud<PointT>& cloud,
    const geometry_msgs::msg::TransformStamped& tf) {
  pcl::PointCloud<PointT> out;
  const auto transform = tf2::transformToEigen(tf);
  pcl::transformPointCloud(cloud, out, transform.cast<float>());
  return out;
}

// Parts of this code were adapted from
// https://github.com/versatran01/cloud2range
class FakeLidarNode : public rclcpp::Node {
 public:
  using PointT = pcl::PointXYZI;
  using CloudT = pcl::PointCloud<PointT>;
  using Image = sensor_msgs::msg::Image;
  using Cinfo = sensor_msgs::msg::CameraInfo;

  // Use ApproximateTime policy to avoid issues caused by msgs not published at
  // exact same time
  using Policy = message_filters::sync_policies::ApproximateTime<
      Image, Image, Image, Image, Image, Image, Image, Image>;
  using Synchonizer = message_filters::Synchronizer<Policy>;

  using PolicyCinfo =
      message_filters::sync_policies::ApproximateTime<Cinfo, Cinfo, Cinfo,
                                                      Cinfo>;
  using SynchonizerCinfo = message_filters::Synchronizer<PolicyCinfo>;

  FakeLidarNode() : Node("fake_lidar") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Read params
    n_beams_ = this->declare_parameter<int>("n_beams", 0);
    if (n_beams_ <= 0) {
      RCLCPP_FATAL(this->get_logger(), "n_beams must be > 0");
      throw std::runtime_error("n_beams <= 0");
    }
    rpm_ = this->declare_parameter<int>("rpm", 0);
    if (rpm_ <= 0) {
      RCLCPP_FATAL(this->get_logger(), "rpm must be > 0");
      throw std::runtime_error("rpm <= 0");
    }
    sample_freq_ = this->declare_parameter<int>("sample_freq", 0);
    if (sample_freq_ <= 0) {
      RCLCPP_FATAL(this->get_logger(), "sample_freq must be > 0");
      throw std::runtime_error("sample_freq <= 0");
    }

    min_alt_angle_ = this->declare_parameter<double>("min_angle", 0.0);
    max_alt_angle_ = this->declare_parameter<double>("max_angle", 0.0);
    if (!(min_alt_angle_ < max_alt_angle_)) {
      RCLCPP_FATAL(this->get_logger(), "min_angle must be < max_angle");
      throw std::runtime_error("min_angle >= max_angle");
    }

    min_range_ = this->declare_parameter<double>("min_range", 0.0);
    max_range_ = this->declare_parameter<double>("max_range", 0.0);
    if (!(min_range_ < max_range_ && min_range_ >= 0.0)) {
      RCLCPP_FATAL(this->get_logger(), "range bounds invalid");
      throw std::runtime_error("range bounds invalid");
    }

    save_as_images_ = this->declare_parameter<bool>("save_as_images", false);
    save_path_ =
        this->declare_parameter<std::string>("save_path_prefix", std::string(""));
    save_image_interval_ =
        this->declare_parameter<int>("save_image_interval", 1);

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "[fake_lidar_node:] save range images: "
                           << save_as_images_);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "[fake_lidar_node:] range images save_path_prefix: "
                           << save_path_);

    cinfo_got_ = false;

    const auto model =
        this->declare_parameter<std::string>("model", std::string(""));
    RCLCPP_INFO(this->get_logger(),
                "[fake_lidar_node:] lidar model: %s",
                model.c_str());

    RCLCPP_INFO(
        this->get_logger(),
        " [fake_lidar_node:] n_beams: %d, rpm: %d, angle(deg): [%0.2f, %0.2f], "
        "range: [%0.2f, %0.2f]",
        n_beams_, rpm_, Deg_Rad(min_alt_angle_), Deg_Rad(max_alt_angle_),
        min_range_, max_range_);

    n_cols_ = sample_freq_ * 60 / rpm_;
    RCLCPP_INFO(this->get_logger(),
                "[fake_lidar_node:] range image shape (%d, %d)", n_beams_,
                n_cols_);

    // Compute azimuth and altitude step sizes
    d_azimuth_ = Rad_Deg(360.0 / n_cols_);
    d_altitude_ = (max_alt_angle_ - min_alt_angle_) / (n_beams_ - 1);
    RCLCPP_INFO(this->get_logger(),
                "[fake_lidar_node:] angular resolution(deg) horizontal: %0.2f, "
                "vertical: %0.2f",
                Deg_Rad(d_azimuth_), Deg_Rad(d_altitude_));

    // this will be used to create the full pc with a mask that includes all
    // points
    full_pc_mask_ = cv::Mat(n_beams_, n_cols_, CV_8UC1);  // gray
    full_pc_mask_.setTo(cv::Scalar(255));

    // Depth Cameras
    depth_cameras_ = {
        DepthCamera(7 * M_PI / 4, 2 * M_PI + M_PI / 4, 0, 1),
        DepthCamera(M_PI / 4, 3 * M_PI / 4, M_PI / 2, 2),
        DepthCamera(3 * M_PI / 4, 5 * M_PI / 4, M_PI, 3),
        DepthCamera(5 * M_PI / 4, 7 * M_PI / 4, 3 * M_PI / 2, 4),
    };

    // Fill in cinfo
    cinfo_.height = n_beams_;
    cinfo_.width = n_cols_;
    cinfo_.distortion_model = model;
    cinfo_.k[0] = min_alt_angle_;
    cinfo_.k[1] = max_alt_angle_;
    cinfo_.k[2] = min_range_;
    cinfo_.k[3] = max_range_;
    cinfo_.k[4] = d_azimuth_;
    cinfo_.k[5] = d_altitude_;
  }

  // Setup() is invoked once the node is wrapped in a shared_ptr because
  // image_transport camera publishers need shared_from_this().
  void Setup() {
    for (int i = 0; i < kNumCameras; ++i) {
      image_subs_[i].subscribe(this, "image" + std::to_string(i + 1));
      RCLCPP_DEBUG_STREAM(this->get_logger(), image_subs_[i].getTopic());
      cinfo_subs_[i].subscribe(this, "cinfo" + std::to_string(i + 1));
      RCLCPP_DEBUG_STREAM(this->get_logger(), cinfo_subs_[i].getTopic());
      semantic_subs_[i].subscribe(this, "semantic" + std::to_string(i + 1));
      RCLCPP_DEBUG_STREAM(this->get_logger(), semantic_subs_[i].getTopic());
    }

    sync_ = std::make_unique<Synchonizer>(
        Policy(5), image_subs_[0], semantic_subs_[0], image_subs_[1],
        semantic_subs_[1], image_subs_[2], semantic_subs_[2], image_subs_[3],
        semantic_subs_[3]);
    sync_->registerCallback(std::bind(
        &FakeLidarNode::AllCamerasCb, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
        std::placeholders::_5, std::placeholders::_6, std::placeholders::_7,
        std::placeholders::_8));

    sync_cinfo_ = std::make_unique<SynchonizerCinfo>(
        PolicyCinfo(5), cinfo_subs_[0], cinfo_subs_[1], cinfo_subs_[2],
        cinfo_subs_[3]);
    sync_cinfo_->registerCallback(std::bind(
        &FakeLidarNode::CinfoCb, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

    pub_range_image_ = image_transport::create_camera_publisher(
        this, "range/image");
    pub_range_image_mask_ = image_transport::create_camera_publisher(
        this, "range/mask_image");

    full_pc_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 1);
    tree_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "trees_cloud", 1);
  }

  geometry_msgs::msg::TransformStamped LookupTransform(
      const std::string& target, const std::string& source) {
    try {
      return tf_buffer_->lookupTransform(target, source, tf2::TimePointZero,
                                         tf2::durationFromSec(1e-3));
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      throw ex;
    }
  }

  void CinfoCb(const Cinfo::ConstSharedPtr& cinfo1_msg,
               const Cinfo::ConstSharedPtr& cinfo2_msg,
               const Cinfo::ConstSharedPtr& cinfo3_msg,
               const Cinfo::ConstSharedPtr& cinfo4_msg) {
    if (!cinfo_got_) {
      camera_models_ = {image_geometry::PinholeCameraModel(),
                        image_geometry::PinholeCameraModel(),
                        image_geometry::PinholeCameraModel(),
                        image_geometry::PinholeCameraModel()};
      camera_models_[0].fromCameraInfo(*cinfo1_msg);
      camera_models_[1].fromCameraInfo(*cinfo2_msg);
      camera_models_[2].fromCameraInfo(*cinfo3_msg);
      camera_models_[3].fromCameraInfo(*cinfo4_msg);
      cinfo_got_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "[fake_lidar_node:] Got camera info!");
    }
  }

  void AllCamerasCb(const Image::ConstSharedPtr& image1_msg,
                    const Image::ConstSharedPtr& semantic1_msg,
                    const Image::ConstSharedPtr& image2_msg,
                    const Image::ConstSharedPtr& semantic2_msg,
                    const Image::ConstSharedPtr& image3_msg,
                    const Image::ConstSharedPtr& semantic3_msg,
                    const Image::ConstSharedPtr& image4_msg,
                    const Image::ConstSharedPtr& semantic4_msg) {
    timer.start();

    if (!cinfo_got_) {
      RCLCPP_INFO(this->get_logger(),
                  "[fake_lidar_node:] Camera info not obtained yet...");
      return;
    }

    RCLCPP_INFO_ONCE(this->get_logger(),
                     "[fake_lidar_node:] Entering images callback");

    std::vector<cv::Mat> images = {
        DecodeDepthImage(image1_msg), DecodeDepthImage(image2_msg),
        DecodeDepthImage(image3_msg), DecodeDepthImage(image4_msg)};

    std::vector<cv::Mat> semantic_images = {
        DecodeSemanticImage(semantic1_msg),
        DecodeSemanticImage(semantic2_msg),
        DecodeSemanticImage(semantic3_msg),
        DecodeSemanticImage(semantic4_msg)};

    cv::Mat range_image = cv::Mat::zeros(n_beams_, n_cols_, CV_32FC1);
    cv::Mat range_image_mask = cv::Mat::zeros(n_beams_, n_cols_, CV_8UC1);
    cv::Mat range_image_to_save = cv::Mat::zeros(n_beams_, n_cols_, CV_16UC1);

    const double radius = 200;

    for (auto cam : depth_cameras_) {
      for (double azimuth = cam.lower_bound; azimuth < cam.upper_bound;
           azimuth += d_azimuth_) {
        double az_mod = std::fmod(azimuth, 2 * M_PI);
        double cam_azimuth = azimuth - cam.cam_center;
        const int col = static_cast<int>(az_mod / d_azimuth_) % n_cols_;

        for (double alt = min_alt_angle_; alt <= max_alt_angle_;
             alt += d_altitude_) {
          PointCV pt;
          EuclideanSpherical(radius, alt, cam_azimuth, pt);
          auto uv = camera_models_[cam.cam_id - 1].project3dToPixel(pt);
          int u = int(uv.x);
          int v = int(uv.y);
          double current_depth =
              static_cast<double>(images[cam.cam_id - 1].at<ushort>(v, u)) /
              1000.0;
          double range = current_depth / (cos(alt) * cos(cam_azimuth));

          const int row = (alt - min_alt_angle_) / d_altitude_ + 0.5;
          int row_idx = n_beams_ - row - 1;
          range_image.at<float>(row_idx, col) = (float)range;
          if (save_as_images_) {
            range_image_to_save.at<ushort>(row_idx, col) =
                (ushort)(range * 1000);
          }
          cv::Vec3b rgbPixel =
              semantic_images[cam.cam_id - 1].at<cv::Vec3b>(v, u);
          if (rgbPixel.val[1] == 234) {
            range_image_mask.at<unsigned char>(row_idx, col) = 255;
          } else if (rgbPixel.val[1] == 218 || rgbPixel.val[1] == 114) {
            range_image_mask.at<unsigned char>(row_idx, col) = 0;
          } else {
            range_image_mask.at<unsigned char>(row_idx, col) = 100;
          }
        }
      }
    }

    cinfo_.header = image1_msg->header;
    cv_bridge::CvImage cv_image(image1_msg->header, "32FC1", range_image);
    cv_bridge::CvImage cv_mask_image(image1_msg->header, "mono8",
                                     range_image_mask);

    auto pc = makeCloudPCL(range_image, full_pc_mask_, max_range_);
    pcl_conversions::toPCL(image1_msg->header.stamp, pc->header.stamp);

    auto tree_pc = makeCloudPCL(range_image, range_image_mask, max_range_);
    tree_pc->header.stamp = pc->header.stamp;
    auto pc_robot_frame =
        transformPC(pc, image1_msg->header.frame_id, "quadrotor");
    auto tree_pc_robot_frame =
        transformPC(tree_pc, image1_msg->header.frame_id, "quadrotor");
    if (save_as_images_) {
      if ((save_image_counter_ % save_image_interval_) == 0) {
        int save_image_idx = save_image_counter_ / save_image_interval_;
        cv::imwrite(
            save_path_ + "/image_" + std::to_string(save_image_idx) + ".png",
            range_image_to_save);
        cv::imwrite(
            save_path_ + "/mask_" + std::to_string(save_image_idx) + ".png",
            range_image_mask);
        pcl::io::savePCDFileASCII(save_path_ + "/point_cloud_" +
                                      std::to_string(save_image_idx) + ".pcd",
                                  pc_robot_frame);
        pcl::io::savePCDFileASCII(save_path_ + "/tree_cloud_" +
                                      std::to_string(save_image_idx) + ".pcd",
                                  tree_pc_robot_frame);
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "[fake_lidar_node:] Range image save! Current "
                           "saved idx: "
                               << save_image_idx);
      }
      save_image_counter_++;
    }

    pub_range_image_.publish(*cv_image.toImageMsg(), cinfo_);
    pub_range_image_mask_.publish(*cv_mask_image.toImageMsg(), cinfo_);

    auto out_full = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(pc_robot_frame, *out_full);
    full_pc_pub_->publish(std::move(out_full));

    auto out_tree = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(tree_pc_robot_frame, *out_tree);
    tree_pc_pub_->publish(std::move(out_tree));
  }

  CloudT transformPC(const CloudT::Ptr pc, std::string source_frame,
                     std::string target_frame) {
    auto tf = LookupTransform(target_frame, source_frame);
    auto cloud_tf = TransformCloud(*pc, tf);
    cloud_tf.header.frame_id = target_frame;
    return cloud_tf;
  }

  CloudT::Ptr makeCloudPCL(const cv::Mat& points, const cv::Mat& mask,
                           float maxDist) {
    CloudT::Ptr range_cloud(new CloudT);
    for (auto i = 0; i < points.rows; ++i) {
      for (auto j = 0; j < points.cols; ++j) {
        float range = points.at<float>(i, j);
        PointT p;
        if (range > maxDist || range < 0 || mask.at<uchar>(i, j) != 255) {
          p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN();
        } else {
          double alt = -(i * d_altitude_ + min_alt_angle_);
          double azimuth = j * d_azimuth_;
          EuclideanSpherical(range, alt, azimuth, p);
          p.intensity = 1;
        }
        range_cloud->points.push_back(p);
      }
    }

    range_cloud->width = points.cols;
    range_cloud->height = points.rows;
    range_cloud->is_dense = true;
    return range_cloud;
  }

 private:
  static constexpr int kNumCameras = 4;
  std::vector<image_geometry::PinholeCameraModel> camera_models_;
  using CinfoSubFilter =
      message_filters::Subscriber<sensor_msgs::msg::CameraInfo>;
  using ImageSubFilter = image_transport::SubscriberFilter;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::array<ImageSubFilter, kNumCameras> image_subs_;
  std::array<CinfoSubFilter, kNumCameras> cinfo_subs_;
  std::array<ImageSubFilter, kNumCameras> semantic_subs_;
  std::unique_ptr<Synchonizer> sync_{nullptr};
  std::unique_ptr<SynchonizerCinfo> sync_cinfo_{nullptr};
  image_transport::CameraPublisher pub_range_image_;
  image_transport::CameraPublisher pub_range_image_mask_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tree_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr full_pc_pub_;

  bool cinfo_got_;
  int n_beams_;
  int rpm_;
  int sample_freq_;
  double min_alt_angle_, max_alt_angle_;
  double min_range_, max_range_;

  bool save_as_images_;
  std::string save_path_;
  int save_image_counter_ = 0;
  int save_image_interval_;

  std::vector<DepthCamera> depth_cameras_;
  double d_azimuth_, d_altitude_;
  int n_cols_;

  cv::Mat full_pc_mask_;

  sensor_msgs::msg::CameraInfo cinfo_;

  cv::Mat DecodeDepthImage(const Image::ConstSharedPtr& image_msg) {
    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(
          image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
      image = input_bridge->image;
    } catch (cv_bridge::Exception& ex) {
      RCLCPP_ERROR(this->get_logger(),
                   "[fake_lidar_node:] Failed to convert depth image");
    }
    return image;
  }

  cv::Mat DecodeSemanticImage(const Image::ConstSharedPtr& image_msg) {
    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge =
          cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
      image = input_bridge->image;
    } catch (cv_bridge::Exception& ex) {
      RCLCPP_ERROR(this->get_logger(),
                   "[fake_lidar_node:] Failed to convert semantic image");
    }
    return image;
  }
};

}  // namespace fake_lidar

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fake_lidar::FakeLidarNode>();
  node->Setup();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
