#include <planning_ros_msgs/Trajectory.h>
#include <traj_opt_msgs/Trajectory.h>

class PrimitiveToTrajOpt
{
public:
    static traj_opt_msgs::Trajectory convert(const planning_ros_msgs::Trajectory &msg);
};
