// Copyright 2016 Michael Watterson

#ifndef MAP_PLAN_PKGS_TRAJ_OPT_ROS_SRC_TRAJECTORY_DISPLAY_H_
#define MAP_PLAN_PKGS_TRAJ_OPT_ROS_SRC_TRAJECTORY_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <planning_ros_msgs/SplineTrajectory.h>
#include <rviz/message_filter_display.h>

#include <boost/circular_buffer.hpp>
#endif


#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <rviz/frame_manager.h>
#include <rviz/load_resource.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>

#include "spline_trajectory_visual.h"  // NOLINT()

namespace Ogre {
class SceneNode;
}

namespace rviz {
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
class BoolProperty;
}  // namespace rviz

namespace planning_rviz_plugins {

class SplineTrajectoryVisual;

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
//
// SplineTrajectoryDisplay will show a 3D arrow showing the direction and magnitude
// of the TRAJECTORY acceleration vector.  The base of the arrow will be at
// the frame listed in the header of the SplineTrajectory message, and the
// direction of the arrow will be relative to the orientation of that
// frame.  It will also optionally show a history of recent
// acceleration vectors, which will be stored in a circular buffer.
//
// The SplineTrajectoryDisplay class itself just implements the circular buffer,
// editable parameters, and Display subclass machinery.  The visuals
// themselves are represented by a separate class, SplineTrajectoryVisual.  The
// idiom for the visuals is that when the objects exist, they appear
// in the scene, and when they are deleted, they disappear.
class SplineTrajectoryDisplay
    : public rviz::MessageFilterDisplay<planning_ros_msgs::SplineTrajectory> {
  Q_OBJECT      // NOLINT
      public :  // NOLINT
                // Constructor.  pluginlib::ClassLoader creates instances by
                // calling the default constructor, so make sure you have one.
                SplineTrajectoryDisplay();
  virtual ~SplineTrajectoryDisplay();

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
 protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();

  // These Qt slots get connected to signals indicating changes in the
  // user-editable properties.
 private Q_SLOTS:  // NOLINT
  void updateColorAndAlpha();
  void randomizeColor();
  void updateHistoryLength();
  void updateScale();
  void updateStyle();
  void updateSampleLength();
  // Function to handle an incoming ROS message.
 private:
  void processMessage(const planning_ros_msgs::SplineTrajectory::ConstPtr& msg);

  // Storage for the list of visuals.  It is a circular buffer where
  // data gets popped from the front (oldest) and pushed to the back (newest)
  boost::circular_buffer<boost::shared_ptr<SplineTrajectoryVisual> > visuals_;

  // User-editable property variables.
  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::ColorProperty* color_property_v_;
  rviz::ColorProperty* color_property_a_;

  rviz::BoolProperty* use_v_property_;
  rviz::BoolProperty* use_a_property_;
  rviz::BoolProperty* use_h_property_;

  rviz::FloatProperty* thickness_property_;
  rviz::IntProperty* history_length_property_;
  rviz::IntProperty* traj_samples_property_;
  rviz::IntProperty* tangent_samples_property_;
  rviz::EnumProperty* style_property_;
};
// END_TUTORIAL
}

#endif  // MAP_PLAN_PKGS_TRAJ_OPT_ROS_SRC_TRAJECTORY_DISPLAY_H_
// %EndTag(FULL_SOURCE)%
