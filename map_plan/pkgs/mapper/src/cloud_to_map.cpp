#include <eigen_conversions/eigen_msg.h>
#include <mapper/tf_listener.h>
#include <mapper/voxel_mapper.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>

// Timing stuff
ros::Publisher time_pub;

std::unique_ptr<mapper::VoxelMapper> voxel_mapper_;  // mapper
ros::Publisher map_pub;
ros::Publisher global_occ_map_pub;
ros::Publisher local_cloud_pub;

bool debug_;
bool real_robot_;          // define it's real-robot experiment or not
std::string map_frame_;    // map frame
std::string lidar_frame_;  // map frame
vec_Vec3i ns_;             // inflation array
double robot_r_, robot_h_, max_range_;
double occ_map_height_;
double local_dim_x_, local_dim_y_, local_dim_z_;
int update_interval_;
int counter_ = 0;
int counter_clear_ = 0;

void processCloud(const sensor_msgs::PointCloud &cloud) {
  if (voxel_mapper_ == nullptr) return;

  // get the transform from fixed frame to lidar frame
  static TFListener tf_listener;
  geometry_msgs::Pose pose_map_cloud;

  if (real_robot_) {
    // for real robot, the point cloud frame_id may not exist in the tf tree,
    // manually defining it here.
    // TODO(xu): make this automatic
    auto tf_map_cloud = tf_listener.LookupTransform(map_frame_, lidar_frame_,
                                                    cloud.header.stamp);
    if (!tf_map_cloud) {
      ROS_WARN("Failed to get transform from %s to %s", lidar_frame_.c_str(),
               map_frame_.c_str());
      return;
    }
    pose_map_cloud = *tf_map_cloud;
  } else {
    auto tf_map_cloud = tf_listener.LookupTransform(
        map_frame_, cloud.header.frame_id, cloud.header.stamp);
    if (!tf_map_cloud) {
      ROS_WARN("Failed to get transform from %s to %s",
               cloud.header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    pose_map_cloud = *tf_map_cloud;
  }

  const Eigen::Affine3d T_m_c = toTF(pose_map_cloud);

  ros::Time t0 = ros::Time::now();
  double min_range = 0.75;  // points within this distance will be discarded
  double min_range_squared;
  min_range_squared = min_range * min_range;
  const auto pts = cloud_to_vec_filter(cloud, min_range_squared);
  voxel_mapper_->addCloud(pts, T_m_c, ns_, false, max_range_);

  const Eigen::Vector3d pos(T_m_c.translation().x(), T_m_c.translation().y(),
                            T_m_c.translation().z());
  double dt1 = (ros::Time::now() - t0).toSec();

  t0 = ros::Time::now();
  planning_ros_msgs::VoxelMap map = voxel_mapper_->getInflatedMap();
  // planning_ros_msgs::VoxelMap map = voxel_mapper_->getMap();
  map.header.frame_id = map_frame_;
  map_pub.publish(map);

  planning_ros_msgs::VoxelMap global_occ_map =
      voxel_mapper_->getInflatedOccMap(occ_map_height_);
  global_occ_map.header.frame_id = map_frame_;
  global_occ_map_pub.publish(global_occ_map);

  const Eigen::Vector3d local_dim(local_dim_x_, local_dim_y_, local_dim_z_);
  const Eigen::Vector3d local_ori = -local_dim / 2;

  auto inflated_cloud =
      voxel_mapper_->getInflatedLocalCloud(pos, local_ori, local_dim);
  sensor_msgs::PointCloud local_cloud = vec_to_cloud(inflated_cloud);
  local_cloud.header.frame_id = map_frame_;
  local_cloud_pub.publish(local_cloud);

  double dt2 = (ros::Time::now() - t0).toSec();

  if (debug_) {
    ROS_INFO_THROTTLE(1, "[Mapper]: Got cloud, number of points: [%zu]",
                      cloud.points.size());
    ROS_INFO_THROTTLE(
        1, "[Mapper]: Time for updating map: %f, for publishing: %f", dt1, dt2);
  }

  sensor_msgs::Temperature tmsg;
  tmsg.header.stamp = t0;
  tmsg.temperature = dt1 * 1000;
  time_pub.publish(tmsg);
}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  // only update voxel once every update_interval_ point clouds
  if (counter_ % update_interval_ == 0) {
    ROS_WARN_ONCE("[Mapper]: got the point cloud!");
    sensor_msgs::PointCloud cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, cloud);
    processCloud(cloud);
    counter_ = 0;
  }

  ++counter_;
}

void mapInfoUpdate(const planning_ros_msgs::VoxelMap::ConstPtr &msg) {
  const Eigen::Vector3d origin(msg->origin.x, msg->origin.y, msg->origin.z);
  const Eigen::Vector3d dim(msg->dim.x, msg->dim.y, msg->dim.z);
  const double res = msg->resolution;
  // Initialize the mapper
  voxel_mapper_.reset(new mapper::VoxelMapper(origin, dim, res));

  ROS_WARN("[Mapper]: get 3D map info!");
  ns_.clear();

  int rn = std::ceil(robot_r_ / res);
  int hn = std::ceil(robot_h_ / res);

  for (int nx = -rn; nx <= rn; ++nx) {
    for (int ny = -rn; ny <= rn; ++ny) {
      for (int nz = -hn; nz <= hn; ++nz) {
        if (nx == 0 && ny == 0) continue;
        if (std::hypot(nx, ny) > rn) continue;
        ns_.push_back(Eigen::Vector3i(nx, ny, nz));
      }
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cloud_to_map");
  ros::NodeHandle nh("~");

  nh.param("map_frame", map_frame_, std::string("map"));
  nh.param("lidar_frame", lidar_frame_, std::string("lidar"));
  nh.param("real_robot", real_robot_, false);
  nh.param("debug", debug_, true);

  ros::Subscriber cloud_sub = nh.subscribe("cloud", 1, cloudCallback);
  // ros::Subscriber map_info_sub = nh.subscribe("map_info", 1, mapInfoUpdate);

  map_pub = nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
  global_occ_map_pub =
      nh.advertise<planning_ros_msgs::VoxelMap>("global_occ_map", 1, true);
  local_cloud_pub =
      nh.advertise<sensor_msgs::PointCloud>("local_cloud", 1, true);

  time_pub = nh.advertise<sensor_msgs::Temperature>("/timing/mapper", 1);

  double horizon_fov;
  int width, height;
  nh.param("occ_map_height", occ_map_height_, 2.0);

  nh.param("robot_r", robot_r_, 0.2);
  nh.param("robot_h", robot_h_, 0.0);
  nh.param("max_range", max_range_, 0.0);

  planning_ros_msgs::VoxelMap map_info;
  nh.param("resolution", map_info.resolution, 0.1f);
  nh.param("origin_x", map_info.origin.x, -0.5);
  nh.param("origin_y", map_info.origin.y, -0.5);
  nh.param("origin_z", map_info.origin.z, 1.0);
  nh.param("range_x", map_info.dim.x, 20.);
  nh.param("range_y", map_info.dim.y, 20.);
  nh.param("range_z", map_info.dim.z, 0.0);

  // only update voxel once every update_interval_ point clouds
  nh.param("num_point_cloud_skip", update_interval_, 5);

  ROS_INFO("range_x: %f, range_y: %f, range_z: %f", map_info.dim.x,
           map_info.dim.y, map_info.dim.z);

  nh.param("local/dim_x", local_dim_x_, 10.0);
  nh.param("local/dim_y", local_dim_y_, 10.0);
  nh.param("local/dim_z", local_dim_z_, 8.0);

  ROS_INFO("local_range_x: %f, local_range_y: %f, local_range_z: %f",
           local_dim_x_, local_dim_y_, local_dim_z_);

  // Initialize
  mapInfoUpdate(boost::make_shared<planning_ros_msgs::VoxelMap>(map_info));

  ros::spin();

  return 0;
}
