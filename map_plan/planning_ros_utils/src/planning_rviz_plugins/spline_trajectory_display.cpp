// Copyright 2016 Michael Watterson

#include "spline_trajectory_display.h"  // NOLINT()

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

#include "spline_trajectory_visual.h"  // NOLINT()

// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
namespace planning_rviz_plugins {

SplineTrajectoryDisplay::SplineTrajectoryDisplay() {
  color_property_ = new rviz::ColorProperty("Color", QColor(204, 51, 204),
                                            "Color of trajectory.", this,
                                            SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz::FloatProperty(
      "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this,
      SLOT(updateColorAndAlpha()));
  color_property_v_ = new rviz::ColorProperty(
      "Velocity Color", QColor(20, 251, 204), "Color of velocity.", this,
      SLOT(updateColorAndAlpha()));
  color_property_a_ = new rviz::ColorProperty(
      "Acceleration Color", QColor(241, 21, 24), "Color of acceleration.", this,
      SLOT(updateColorAndAlpha()));

  thickness_property_ =
      new rviz::FloatProperty("Line Thickness", 0.1,
                              "Does nothing for Sikang style trajectories "
                              "because lines are always 1px in Ogre api",
                              this, SLOT(updateScale()));
  thickness_property_->setMin(0.01);
  thickness_property_->setMax(3.00);

  use_v_property_ =
      new rviz::BoolProperty("Plot Velocity", true, "Turns arrow/lines on/off",
                             this, SLOT(updateSampleLength()));
  use_a_property_ = new rviz::BoolProperty("Plot Acceleration", false,
                                           "Turns arrow/lines on/off", this,
                                           SLOT(updateSampleLength()));

  history_length_property_ = new rviz::IntProperty(
      "History Length", 1,
      "Number of prior trajectories to display. Warning!! "
      "Setting this too high is not recommended, it will "
      "bog down rviz",
      this, SLOT(updateHistoryLength()));
  history_length_property_->setMin(1);
  history_length_property_->setMax(150);

  traj_samples_property_ =
      new rviz::IntProperty("SplineTrajectory Samples", 50,
                            "Number of samples used to draw trajectory", this,
                            SLOT(updateSampleLength()));
  traj_samples_property_->setMin(10);
  traj_samples_property_->setMax(500);

  tangent_samples_property_ =
      new rviz::IntProperty("Tangent Samples", 25,
                            "Number of samples used to draw velocity and "
                            "acceleration arrows or lines",
                            this, SLOT(updateSampleLength()));
  tangent_samples_property_->setMin(10);
  tangent_samples_property_->setMax(500);
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.

void SplineTrajectoryDisplay::onInitialize() {
  MFDClass::onInitialize();
  updateStyle();
  updateScale();
  updateHistoryLength();
  updateColorAndAlpha();
  updateSampleLength();
}

SplineTrajectoryDisplay::~SplineTrajectoryDisplay() {}

// Clear the visuals by deleting their objects.
void SplineTrajectoryDisplay::reset() {
  MFDClass::reset();
  visuals_.clear();
}

// Set the current color and alpha values for each visual.
void SplineTrajectoryDisplay::updateColorAndAlpha() {
  float alpha = alpha_property_->getFloat();

  Ogre::ColourValue color = color_property_->getOgreColor();
  Ogre::ColourValue colorv = color_property_v_->getOgreColor();
  Ogre::ColourValue colora = color_property_a_->getOgreColor();

  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->setColor(color.r, color.g, color.b, alpha);
    visuals_[i]->setColorV(colorv.r, colorv.g, colorv.b, alpha);
    visuals_[i]->setColorA(colora.r, colora.g, colora.b, alpha);
  }
}

void SplineTrajectoryDisplay::updateStyle() {
  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->draw();
  }
  updateColorAndAlpha();
}

// Set the scale for each visual.
void SplineTrajectoryDisplay::updateScale() {
  float thickness = thickness_property_->getFloat();

  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->setScale(thickness);
  }
}

// Set the number of past visuals to show.
void SplineTrajectoryDisplay::updateHistoryLength() {
  visuals_.rset_capacity(history_length_property_->getInt());
}
void SplineTrajectoryDisplay::updateSampleLength() {
  int traj_points = traj_samples_property_->getInt();
  int tangent_points = tangent_samples_property_->getInt();
  bool use_v = use_v_property_->getBool();
  bool use_a = use_a_property_->getBool();

  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->resetTrajPoints(traj_points, tangent_points, use_v, use_a);
    visuals_[i]->draw();
  }
  updateColorAndAlpha();
}

// This is our callback to handle an incoming message.
void SplineTrajectoryDisplay::processMessage(
    const planning_ros_msgs::SplineTrajectory::ConstPtr &msg) {
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this SplineTrajectory message. If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  // updateStyle();
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  // We are keeping a circular buffer of visual pointers.  This gets
  // the next one, or creates and stores it if the buffer is not full
  boost::shared_ptr<SplineTrajectoryVisual> visual;
  if (visuals_.full()) {
    visual = visuals_.front();
  } else {
    visual.reset(
        new SplineTrajectoryVisual(context_->getSceneManager(), scene_node_));
  }

  // Now set or update the contents of the chosen visual.
  float thickness = thickness_property_->getFloat();
  visual->setScale(thickness);
  int traj_points = traj_samples_property_->getInt();
  int tangent_points = tangent_samples_property_->getInt();
  bool use_v = use_v_property_->getBool();
  bool use_a = use_a_property_->getBool();
  visual->resetTrajPoints(traj_points, tangent_points, use_v, use_a);
  visual->draw();
  visual->setMessage(msg);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  Ogre::ColourValue colorv = color_property_v_->getOgreColor();
  Ogre::ColourValue colora = color_property_a_->getOgreColor();
  visual->setColor(color.r, color.g, color.b, alpha);
  visual->setColorV(colorv.r, colorv.g, colorv.b, alpha);
  visual->setColorA(colora.r, colora.g, colora.b, alpha);

  // And send it to the end of the circular buffer
  visuals_.push_back(visual);
}
void SplineTrajectoryDisplay::randomizeColor() {}
}  // namespace planning_rviz_plugins
// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>  // NOLINT()
PLUGINLIB_EXPORT_CLASS(planning_rviz_plugins::SplineTrajectoryDisplay,
                       rviz::Display)
