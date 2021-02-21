#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

// traj_opt stuff
#include <traj_opt_basic/msg_traj.h>
#include <traj_opt_quadrotor/convert.h>
#include <traj_opt_ros/ros_bridge.h>

// planning ros msgs stuff
#include <planning_ros_msgs/Trajectory_traj_opt.h>

static ros::Subscriber traj_sub;
static ros::Publisher warp_pub, cmd_pub;
static boost::shared_ptr<traj_opt::Trajectory> traj;
static ros::Time start;
static double kx_[3], kv_[3];

using kr_mav_msgs::PositionCommand;

void trajCB(const planning_ros_msgs::Trajectory_traj_opt::ConstPtr &traj_msg) {
  traj = boost::make_shared<traj_opt::MsgTrajectory>(
      TrajRosBridge::convert(*traj_msg));
  start = ros::Time::now();
  PositionCommand cmd;
  TrajToQuadCmd::evaluate(traj, 0, &cmd);
  geometry_msgs::Pose pose;
  pose.position.x = cmd.position.x;
  pose.position.y = cmd.position.y;
  pose.position.z = cmd.position.z;
  pose.orientation.w = std::cos(cmd.yaw / 2.0);
  pose.orientation.z = std::sin(cmd.yaw / 2.0);
  warp_pub.publish(pose);
}
void update() {
  ros::Time tn = ros::Time::now();
  double dt = (tn - start).toSec();
  if (!traj) return;
  PositionCommand cmd;
  TrajToQuadCmd::evaluate(traj, dt, &cmd);
  cmd.header.stamp = tn;
  cmd.kx[0] = kx_[0], cmd.kx[1] = kx_[1], cmd.kx[2] = kx_[2];
  cmd.kv[0] = kv_[0], cmd.kv[1] = kv_[1], cmd.kv[2] = kv_[2];
  cmd_pub.publish(cmd);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "quad_sim");

  ros::NodeHandle nh("~");

  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  warp_pub = nh.advertise<geometry_msgs::Pose>("warp", 5);
  cmd_pub = nh.advertise<PositionCommand>("position_cmd", 5);

  traj_sub = nh.subscribe("trajectory", 1, trajCB);

  ros::Rate r(100);
  while (nh.ok()) {
    ros::spinOnce();
    update();
    r.sleep();
  }

  return 0;
}
