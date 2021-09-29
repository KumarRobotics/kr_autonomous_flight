#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tbb/parallel_invoke.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <boost/timer/timer.hpp>
#include <mutex>

namespace fake_lidar {

template <typename PointT>
pcl::PointCloud<PointT> TransformCloud(
    const pcl::PointCloud<PointT>& cloud,
    const geometry_msgs::TransformStamped& tf) {
  pcl::PointCloud<PointT> out;
  const auto transform = tf2::transformToEigen(tf);
  pcl::transformPointCloud(cloud, out, transform.cast<float>());
  return out;
}

class FakeLidarNode {
 public:
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;
  using Policy = message_filters::sync_policies::ApproximateTime<Cloud, Cloud,
                                                                 Cloud, Cloud>;
  using Synchonizer = message_filters::Synchronizer<Policy>;

  explicit FakeLidarNode(const ros::NodeHandle& pnh)
      : pnh_{pnh},
        cloud_pub_{pnh_.advertise<Cloud>("points", 1)},
        tf_listener_{tf_buffer_},
        parallel_{pnh_.param<bool>("parallel", false)} {
    for (std::size_t i = 0; i < cloud_subs_.size(); ++i) {
      cloud_subs_[i].subscribe(pnh_, "points" + std::to_string(i + 1), 1);
      ROS_DEBUG_STREAM(cloud_subs_[i].getTopic());
    }

    sync_ =
        std::make_unique<Synchonizer>(Policy(5), cloud_subs_[0], cloud_subs_[1],
                                      cloud_subs_[2], cloud_subs_[3]);
    sync_->registerCallback(
        boost::bind(&FakeLidarNode::AllCloudsCb, this, _1, _2, _3, _4));

    ROS_DEBUG("%s", parallel_ ? "parallel" : "serial");
  }

  geometry_msgs::TransformStamped LookupTransform(const std::string& target,
                                                  const std::string& source) {
    try {
      return tf_buffer_.lookupTransform(target, source, ros::Time(0),
                                        ros::Duration(1e-3));
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
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

  void AllCloudsCb(const boost::shared_ptr<const Cloud>& cloud1_msg,
                   const boost::shared_ptr<const Cloud>& cloud2_msg,
                   const boost::shared_ptr<const Cloud>& cloud3_msg,
                   const boost::shared_ptr<const Cloud>& cloud4_msg) {
    merged_ = cloud1_msg->makeShared();
    merged_->reserve(cloud1_msg->size() + cloud2_msg->size() +
                     cloud3_msg->size() + cloud4_msg->size());

    timer_.start();
    if (parallel_) {
      tbb::parallel_invoke([&cloud2_msg, this]() { AddCloud(*cloud2_msg); },
                           [&cloud3_msg, this]() { AddCloud(*cloud3_msg); },
                           [&cloud4_msg, this]() { AddCloud(*cloud4_msg); });
    } else {
      AddCloud(*cloud2_msg);
      AddCloud(*cloud3_msg);
      AddCloud(*cloud4_msg);
    }
    ROS_DEBUG("AddCloud : %d ms", int(timer_.elapsed().wall / 1e6));
    merged_->width = merged_->size();
    merged_->height = 1;
    cloud_pub_.publish(*merged_);
  }

 private:
  using CloudSubFilter = message_filters::Subscriber<Cloud>;

  ros::NodeHandle pnh_;
  ros::Publisher cloud_pub_;
  std::array<CloudSubFilter, 4> cloud_subs_;
  std::unique_ptr<Synchonizer> sync_{nullptr};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool parallel_{false};
  std::mutex cloud_mtx_;
  Cloud::Ptr merged_;

  boost::timer::cpu_timer timer_;
};

}  // namespace fake_lidar

int main(int argc, char** argv) {
  ros::init(argc, argv, "fake_lidar");
  fake_lidar::FakeLidarNode node(ros::NodeHandle("~"));
  ros::spin();
}
