#include <planning_ros_msgs/Trajectory.h>
#include <planning_ros_msgs/Trajectory_traj_opt.h>
#include <planning_ros_msgs/Spline.h>
#include <planning_ros_msgs/Polynomial.h>

class PrimitiveToTrajOpt
{
public:
    static planning_ros_msgs::Trajectory_traj_opt convert(const planning_ros_msgs::Trajectory &msg);
};
