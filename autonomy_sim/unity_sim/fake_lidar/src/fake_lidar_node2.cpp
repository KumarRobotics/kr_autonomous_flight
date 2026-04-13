#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tbb/parallel_invoke.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.hpp>

#include <boost/timer/timer.hpp>
#include <memory>
#include <mutex>
#include <string>

namespace fake_lidar {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

template <typename PointT>
pcl::PointCloud<PointT> TransformCloud(
    const pcl::PointCloud<PointT>& cloud,
    const geometry_msgs::msg::TransformStamped& tf) {
  pcl::PointCloud<PointT> out;
  const auto transform = tf2::transformToEigen(tf);
  pcl::transformPointCloud(cloud, out, transform.cast<float>());
  return out;
}

class FakeLidarNode : public rclcpp::Node {
 public:
  using Policy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2,
      sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
  using Synchonizer = message_filters::Synchronizer<Policy>;

  FakeLidarNode() : Node("fake_lidar") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    parallel_ = this->declare_parameter<bool>("parallel", false);
    cloud_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("points", 1);
  }

  void Setup() {
    for (std::size_t i = 0; i < cloud_subs_.size(); ++i) {
      cloud_subs_[i].subscribe(this, "points" + std::to_string(i + 1));
      RCLCPP_DEBUG_STREAM(this->get_logger(), cloud_subs_[i].getTopic());
    }

    sync_ = std::make_unique<Synchonizer>(
        Policy(5), cloud_subs_[0], cloud_subs_[1], cloud_subs_[2],
        cloud_subs_[3]);
    sync_->registerCallback(std::bind(
        &FakeLidarNode::AllCloudsCb, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

    RCLCPP_DEBUG(this->get_logger(), "%s",
                 parallel_ ? "parallel" : "serial");
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

  void AddCloud(const Cloud& cloud) {
    auto tf = LookupTransform(merged_->header.frame_id, cloud.header.frame_id);
    auto cloud_tf = TransformCloud(cloud, tf);
    if (parallel_) {
      std::lock_guard<std::mutex> g(cloud_mtx_);
      *merged_ += cloud_tf;
    } else {
      *merged_ += cloud_tf;
    }
  }

  void AllCloudsCb(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud1_msg,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud2_msg,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud3_msg,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud4_msg) {
    Cloud c1, c2, c3, c4;
    pcl::fromROSMsg(*cloud1_msg, c1);
    pcl::fromROSMsg(*cloud2_msg, c2);
    pcl::fromROSMsg(*cloud3_msg, c3);
    pcl::fromROSMsg(*cloud4_msg, c4);

    merged_ = c1.makeShared();
    merged_->reserve(c1.size() + c2.size() + c3.size() + c4.size());

    timer_.start();
    if (parallel_) {
      tbb::parallel_invoke([&c2, this]() { AddCloud(c2); },
                           [&c3, this]() { AddCloud(c3); },
                           [&c4, this]() { AddCloud(c4); });
    } else {
      AddCloud(c2);
      AddCloud(c3);
      AddCloud(c4);
    }
    RCLCPP_DEBUG(this->get_logger(), "AddCloud : %d ms",
                 int(timer_.elapsed().wall / 1e6));
    merged_->width = merged_->size();
    merged_->height = 1;

    auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*merged_, *out);
    cloud_pub_->publish(std::move(out));
  }

 private:
  using CloudSubFilter =
      message_filters::Subscriber<sensor_msgs::msg::PointCloud2>;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  std::array<CloudSubFilter, 4> cloud_subs_;
  std::unique_ptr<Synchonizer> sync_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool parallel_{false};
  std::mutex cloud_mtx_;
  Cloud::Ptr merged_;

  boost::timer::cpu_timer timer_;
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
