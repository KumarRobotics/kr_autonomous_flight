#include <planning_ros_msgs/Trajectory.h>
#include <planning_ros_msgs/SplineTrajectory.h>
#include <planning_ros_msgs/Spline.h>
#include <planning_ros_msgs/Polynomial.h>

class PrimitiveToTrajOpt
{
public:
    static planning_ros_msgs::SplineTrajectory
		convert(const planning_ros_msgs::Trajectory &msg);
};
