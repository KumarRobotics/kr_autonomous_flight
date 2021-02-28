// Copyright 2016 Michael Watterson

#ifndef MOBILITY_PLANNER_TRAJ_OPT_SRC_TRAJECTORY_VISUAL_H_
#define MOBILITY_PLANNER_TRAJ_OPT_SRC_TRAJECTORY_VISUAL_H_

#include <planning_ros_msgs/SplineTrajectory.h>
#include <traj_opt_basic/trajectory.h>
#include <traj_opt_basic/types.h>

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

namespace traj_opt {

// BEGIN_TUTORIAL
// Declare the visual class for this display.
//
// Each instance of TrajectoryVisual represents the visualization of a single
// planning_ros_msgs::SplineTrajectory message.  Currently it just shows an
// arrow with the direction and magnitude of the acceleration vector, but could
// easily be expanded to include more of the message data.

enum Style { Mike, Sikang, CJ, Hopf };

class TrajectoryVisual {
 public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  TrajectoryVisual(Ogre::SceneManager* scene_manager,
                   Ogre::SceneNode* parent_node);

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~TrajectoryVisual();

  void draw();
  // Configure the visual to show the data in the message.
  void setMessage(const planning_ros_msgs::SplineTrajectory::ConstPtr& msg);

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way TrajectoryVisual is only
  // responsible for visualization.
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the Trajectory message.
  void setColor(float r, float g, float b, float a);
  void setColorV(float r, float g, float b, float a);
  void setColorA(float r, float g, float b, float a);
  void setScale(float thickness);
  void setCurve();
  void setStyle(int style);

  void resetTrajPoints(int traj_points, int tangent_points, bool use_v,
                       bool use_a, bool use_h);

 private:
  traj_opt::Mat3 matFromVecD(const traj_opt::VecD& vec);
  Ogre::Vector3 vecFromVecD(const traj_opt::VecD& vec);
  // Tangent velocity vectors
  std::vector<boost::shared_ptr<rviz::Object> > vel_arrows_;
  std::vector<boost::shared_ptr<rviz::Object> > acc_arrows_;
  std::vector<boost::shared_ptr<rviz::Object> > hopf_arrows_;
  // Lines making up the actual trajectory
  std::vector<boost::shared_ptr<rviz::Object> > trajectory_lines_;
  std::vector<boost::shared_ptr<rviz::Shape> > trajectory_balls_;

  double thickness_{0.1};
  int num_traj_points_{50};
  int num_vel_points_{50};
  bool vel_on_{true};
  bool acc_on_{false};
  bool hopf_on_{false};

  //    Style style_{Style::Sikang}; // default to working style
  Style style_{Style::Mike};  // default to better style

  static void setShapeFromPosePair(const Ogre::Vector3& p0,
                                   const Ogre::Vector3& p1, double scale,
                                   rviz::Shape* shape);
  static void setShapeFromPosePair(const Ogre::Vector3& p0,
                                   const Ogre::Vector3& p1, double scale,
                                   rviz::Arrow* shape);

  boost::shared_ptr<traj_opt::Trajectory> traj_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the Trajectory message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;
};
// END_TUTORIAL

}  // namespace traj_opt

#endif  // MOBILITY_PLANNER_TRAJ_OPT_SRC_TRAJECTORY_VISUAL_H_
