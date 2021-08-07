// Copyright 2016 Michael Watterson

#ifndef MAP_PLAN_PLANNING_ROS_UTILS_SRC_PLANNING_RVIZ_PLUGINS_SPLINE_TRAJECTORY_VISUAL_H_
#define MAP_PLAN_PLANNING_ROS_UTILS_SRC_PLANNING_RVIZ_PLUGINS_SPLINE_TRAJECTORY_VISUAL_H_

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <planning_ros_msgs/SplineTrajectory.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

namespace Ogre {
class Vector3;
class Quaternion;
}  // namespace Ogre

namespace rviz {
class Arrow;
class Line;
class Shape;
class Object;
}  // namespace rviz

// BEGIN_TUTORIAL
// Declare the visual class for this display.
//
// Each instance of SplineTrajectoryVisual represents the visualization of a
// single planning_ros_msgs::SplineTrajectory message.  Currently it just shows
// an arrow with the direction and magnitude of the acceleration vector, but
// could easily be expanded to include more of the message data.
namespace planning_rviz_plugins {

class SplineTrajectoryVisual {
 public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  SplineTrajectoryVisual(Ogre::SceneManager* scene_manager,
                         Ogre::SceneNode* parent_node);

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~SplineTrajectoryVisual();

  void draw();
  // Configure the visual to show the data in the message.
  void setMessage(const planning_ros_msgs::SplineTrajectory::ConstPtr& msg);

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way SplineTrajectoryVisual is only
  // responsible for visualization.
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the SplineTrajectory message.
  void setColor(float r, float g, float b, float a);
  void setColorV(float r, float g, float b, float a);
  void setColorA(float r, float g, float b, float a);
  void setScale(float thickness);
  void setCurve();

  void resetTrajPoints(int traj_points, int tangent_points, bool use_v,
                       bool use_a);

 private:
  Eigen::Matrix3d matFromVecD(const Eigen::VectorXd& vec);
  Ogre::Vector3 vecFromVecD(const Eigen::VectorXd& vec);
  // Tangent velocity vectors
  std::vector<std::shared_ptr<rviz::Object> > vel_arrows_;
  std::vector<std::shared_ptr<rviz::Object> > acc_arrows_;
  // Lines making up the actual trajectory
  std::vector<std::shared_ptr<rviz::Object> > trajectory_lines_;
  std::vector<std::shared_ptr<rviz::Shape> > trajectory_balls_;
  Eigen::VectorXd evaluate(double t, uint deriv_num) const;

  double thickness_{0.1};
  int num_traj_points_{50};
  int num_vel_points_{50};
  bool vel_on_{true};
  bool acc_on_{false};

  static void setShapeFromPosePair(const Ogre::Vector3& p0,
                                   const Ogre::Vector3& p1, double scale,
                                   rviz::Shape* shape);
  static void setShapeFromPosePair(const Ogre::Vector3& p0,
                                   const Ogre::Vector3& p1, double scale,
                                   rviz::Arrow* shape);

  planning_ros_msgs::SplineTrajectory::ConstPtr traj_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the SplineTrajectory message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;
};
// END_TUTORIAL
}  // namespace planning_rviz_plugins
#endif  // MAP_PLAN_PLANNING_ROS_UTILS_SRC_PLANNING_RVIZ_PLUGINS_SPLINE_TRAJECTORY_VISUAL_H_
