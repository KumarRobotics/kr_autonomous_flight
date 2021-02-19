#include <depth_image_proc/depth_traits.h>
#include <eigen_conversions/eigen_msg.h>
#include <image_geometry/pinhole_camera_model.h>
#include <mapper/pcl_utils.h>
#include <mapper/rgb_type.h>
#include <mapper/tf_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <mpl_basis/data_type.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <algorithm>
#include <limits>

ros::Publisher cloud_pub;
ros::Publisher virtual_cloud_pub;
std::string camera_frame_;
std::string robot_frame_;
bool to_robot_frame;
RGBValue colorRed;
RGBValue colorBlue;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image>
    Policy_RGBD;
typedef message_filters::Synchronizer<Policy_RGBD> SYNC_RGBD;

float upper_bound_, lower_bound_;
float u_off_ = 32, v_off_ = 24;
int data_skip_;
int binning_;
static int count_ = 0;

bool publish_virtual_cloud_;
bool outlier_removal_;
float res_ = 0.05;

boost::optional<image_geometry::PinholeCameraModel> model_ptr;

void assign_color() {
  colorRed.Red = 255;
  colorRed.Green = 0;
  colorRed.Blue = 0;
  colorRed.Alpha = 0;

  colorBlue.Red = 85;
  colorBlue.Green = 170;
  colorBlue.Blue = 255;
  colorBlue.Alpha = 1;
}

void info_callback(const sensor_msgs::CameraInfoConstPtr &info_msg) {
  if (!model_ptr) {
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(info_msg);
    model_ptr = model;
  }
}

void pointcloud_convert_depth(const sensor_msgs::ImageConstPtr depth_msg) {
  if (!model_ptr) return;

  if (count_ < data_skip_) {
    count_++;
    return;
  }
  count_ = 0;

  int index = 0;
  int virtual_index = 0;
  PCLPointCloud cloud_msg, virtual_cloud_msg;

  // Use correct principal point from calibration
  float center_x = model_ptr->cx();
  float center_y = model_ptr->cy();

  // Combine unit conversion (if necessary) with scaling by focal length for
  // computing (X,Y)
  float unit_scaling = depth_image_proc::DepthTraits<float>::toMeters(1.0f);
  float constant_x = unit_scaling / model_ptr->fx();
  float constant_y = unit_scaling / model_ptr->fy();
  //  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  const float *depth_row = reinterpret_cast<const float *>(&depth_msg->data[0]);
  int row_step = binning_ * depth_msg->step / sizeof(float);

  for (int v = 0; v < (int)depth_msg->height;
       v += binning_, depth_row += row_step) {
    for (int u = 0; u < (int)depth_msg->width; u += binning_) {
      if (u < u_off_ || u >= depth_msg->width - u_off_ || v < v_off_ ||
          v >= depth_msg->height - v_off_)
        continue;
      float depth = depth_row[u];
      // Check for invalid measurements
      if (!depth_image_proc::DepthTraits<float>::valid(depth) ||
          depth >= upper_bound_) {
        if (publish_virtual_cloud_) {
          virtual_cloud_msg.resize(virtual_index + 1);
          virtual_cloud_msg.points[virtual_index].x =
              (u - center_x) * upper_bound_ * constant_x;
          virtual_cloud_msg.points[virtual_index].y =
              (v - center_y) * upper_bound_ * constant_y;
          virtual_cloud_msg.points[virtual_index].z = upper_bound_;
          // Fill in color
          virtual_cloud_msg.points[virtual_index].rgb = colorRed.float_value;
          virtual_index++;
        }
      } else {
        cloud_msg.resize(index + 1);
        cloud_msg.points[index].x = (u - center_x) * depth * constant_x;
        cloud_msg.points[index].y = (v - center_y) * depth * constant_y;
        cloud_msg.points[index].z =
            depth_image_proc::DepthTraits<float>::toMeters(depth);

        // Fill in color
        cloud_msg.points[index].rgb = colorBlue.float_value;
        index++;
      }
    }
  }

  PCLUtils::voxel_filter(cloud_msg, res_);
  if (outlier_removal_) PCLUtils::outlier_removal(cloud_msg, res_, 2);
  sensor_msgs::PointCloud cloud_output = PCLUtils::toROS(cloud_msg);

  cloud_output.header.stamp = depth_msg->header.stamp;
  cloud_output.header.frame_id = camera_frame_;
  cloud_pub.publish(cloud_output);

  if (publish_virtual_cloud_) {
    PCLUtils::voxel_filter(virtual_cloud_msg, res_);
    sensor_msgs::PointCloud virtual_cloud_output =
        PCLUtils::toROS(virtual_cloud_msg);
    virtual_cloud_output.channels.resize(1);
    virtual_cloud_output.channels[0].name = "virtual";

    virtual_cloud_output.header.stamp = depth_msg->header.stamp;
    virtual_cloud_output.header.frame_id = camera_frame_;
    virtual_cloud_pub.publish(virtual_cloud_output);
  }
}

void pointcloud_convert(const sensor_msgs::ImageConstPtr depth_msg,
                        const sensor_msgs::ImageConstPtr rgb_msg) {
  if (!model_ptr) return;

  if (count_ < data_skip_) {
    count_++;
    return;
  }
  count_ = 0;

  PCLPointCloud cloud_msg;
  int red_offset = 0;
  int green_offset = 1;
  int blue_offset = 2;

  if (depth_msg->width != rgb_msg->width ||
      depth_msg->height != rgb_msg->height) {
    ROS_ERROR("Depth and RGB image sizes don't match, not processing");
    return;
  }
  if (rgb_msg->encoding != sensor_msgs::image_encodings::RGB8) {
    ROS_ERROR("RGB Encoding is wrong, not processing");
    return;
  }

  // Use correct principal point from calibration
  float center_x = model_ptr->cx();
  float center_y = model_ptr->cy();

  // Combine unit conversion (if necessary) with scaling by focal length for
  // computing (X,Y)
  float unit_scaling = depth_image_proc::DepthTraits<float>::toMeters(1.0f);
  float constant_x = unit_scaling / model_ptr->fx();
  float constant_y = unit_scaling / model_ptr->fy();

  const float *depth_row = reinterpret_cast<const float *>(&depth_msg->data[0]);
  int row_step = binning_ * depth_msg->step / sizeof(float);
  const uint8_t *rgb = &rgb_msg->data[0];
  static int color_step = 3 * binning_ * rgb_msg->width / depth_msg->width;
  // int rgb_skip = (rgb_msg->step - rgb_msg->width * color_step);
  int rgb_skip = rgb_msg->step * (binning_ - 1);

  int index = 0;
  for (int v = 0; v < (int)depth_msg->height;
       v += binning_, depth_row += row_step, rgb += rgb_skip) {
    for (int u = 0; u < (int)depth_msg->width;
         u += binning_, rgb += color_step) {
      if (u < u_off_ || u >= depth_msg->width - u_off_ || v < v_off_ ||
          v >= depth_msg->height - v_off_)
        continue;

      float depth = depth_row[u];

      // Check for invalid measurements
      if (!depth_image_proc::DepthTraits<float>::valid(depth) ||
          depth >= upper_bound_) {
      } else {
        cloud_msg.resize(index + 1);
        cloud_msg.points[index].x = (u - center_x) * depth * constant_x;
        cloud_msg.points[index].y = (v - center_y) * depth * constant_y;
        cloud_msg.points[index].z =
            depth_image_proc::DepthTraits<float>::toMeters(depth);

        // Fill in color
        RGBValue color;
        color.Red = rgb[red_offset];
        color.Green = rgb[green_offset];
        color.Blue = rgb[blue_offset];
        color.Alpha = 255;
        cloud_msg.points[index].rgb = color.float_value;
        index++;
      }
    }
  }

  if (cloud_msg.points.size() < 10) {
    sensor_msgs::PointCloud cloud_output;
    cloud_output.header.stamp = depth_msg->header.stamp;
    cloud_output.header.frame_id = camera_frame_;
    cloud_pub.publish(cloud_output);

    return;
  }
  PCLUtils::voxel_filter(cloud_msg, res_);
  PCLUtils::outlier_removal(cloud_msg, res_, 2);
  sensor_msgs::PointCloud cloud_output = PCLUtils::toROS(cloud_msg);
  /*
  sensor_msgs::PointCloud2 cloud_output;
  pcl::toROSMsg(cloud_msg, cloud_output);
  */
  cloud_output.header.stamp = depth_msg->header.stamp;
  cloud_output.header.frame_id = camera_frame_;
  cloud_pub.publish(cloud_output);
  /*
  ROS_INFO_THROTTLE(1, "Publish cloud! size: [%zu], res: [%f]",
      cloud_output.points.size(), res_);
      */
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_to_cloud");
  ros::NodeHandle nh("~");

  nh.param("publish_virtual_cloud", publish_virtual_cloud_, false);
  bool use_color;
  nh.param("leaf_size", res_, 0.05f);
  nh.param("min_range", lower_bound_, 0.3f);
  nh.param("max_range", upper_bound_, 4.5f);

  nh.param("data_skip", data_skip_, 0);
  nh.param("binning", binning_, 1);
  nh.param("use_color", use_color, true);
  nh.param("outlier_removal", outlier_removal_, true);
  nh.param("camera_frame", camera_frame_, std::string("camera"));
  nh.param("robot_frame", robot_frame_, std::string("juliett"));

  message_filters::Subscriber<sensor_msgs::Image> depth_image_sub;
  message_filters::Subscriber<sensor_msgs::Image> rgb_image_sub;
  SYNC_RGBD sync_rgbd(Policy_RGBD(200));

  ros::Subscriber only_depth_image_sub;

  if (use_color) {
    depth_image_sub.subscribe(nh, "depth_image_in", 5);
    rgb_image_sub.subscribe(nh, "rgb_image_in", 5);
    sync_rgbd.connectInput(depth_image_sub, rgb_image_sub);
    sync_rgbd.registerCallback(boost::bind(&pointcloud_convert, _1, _2));
    ROS_INFO("Use RGB!");
  } else
    only_depth_image_sub =
        nh.subscribe("depth_image_in", 5, pointcloud_convert_depth);

  ros::Subscriber info_sub = nh.subscribe("camera_info_in", 5, info_callback);

  cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
  if (publish_virtual_cloud_)
    virtual_cloud_pub =
        nh.advertise<sensor_msgs::PointCloud>("virtual_cloud", 1, true);

  ros::spin();

  return 0;
}
