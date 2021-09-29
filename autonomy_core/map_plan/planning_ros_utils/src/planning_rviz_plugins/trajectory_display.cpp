#include "trajectory_display.h"

namespace planning_rviz_plugins {
TrajectoryDisplay::TrajectoryDisplay() {
  num_property_ = new rviz::IntProperty(
      "Num of samples", 100, "Number of samples of trajectory to display.",
      this, SLOT(updateNum()));
  yaw_triangle_scale_property_ = new rviz::FloatProperty(
      "YawTriangleScale", 0.5, "0.5 is the default value.", this,
      SLOT(updateYawTriangleScale()));
  yaw_triangle_angle_property_ = new rviz::FloatProperty(
      "YawTriangleAngle", 0.7, "0.7 is the default value.", this,
      SLOT(updateYawTriangleAngle()));
  yaw_num_property_ = new rviz::IntProperty(
      "NumYaw", 20, "Number of yaw samples of trajectory to display.", this,
      SLOT(updateYawNum()));
  pos_color_property_ = new rviz::ColorProperty(
      "PosColor", QColor(204, 51, 204), "Color to draw the Pos.", this,
      SLOT(updatePosColorAndAlpha()));
  vel_color_property_ = new rviz::ColorProperty("VelColor", QColor(85, 85, 255),
                                                "Color to draw the Vel.", this,
                                                SLOT(updateVelColorAndAlpha()));
  acc_color_property_ = new rviz::ColorProperty("AccColor", QColor(10, 200, 55),
                                                "Color to draw the Acc.", this,
                                                SLOT(updateAccColorAndAlpha()));
  jrk_color_property_ = new rviz::ColorProperty("JrkColor", QColor(200, 20, 55),
                                                "Color to draw the Acc.", this,
                                                SLOT(updateJrkColorAndAlpha()));
  yaw_color_property_ = new rviz::ColorProperty("YawColor", QColor(100, 20, 55),
                                                "Color to draw the Yaw.", this,
                                                SLOT(updateYawColorAndAlpha()));
  pos_scale_property_ =
      new rviz::FloatProperty("PosScale", 0.1, "0.1 is the default value.",
                              this, SLOT(updatePosScale()));
  vel_scale_property_ =
      new rviz::FloatProperty("VelScale", 0.02, "0.02 is the default value.",
                              this, SLOT(updateVelScale()));
  acc_scale_property_ =
      new rviz::FloatProperty("AccScale", 0.02, "0.02 is the default value.",
                              this, SLOT(updateAccScale()));
  jrk_scale_property_ =
      new rviz::FloatProperty("JrkScale", 0.02, "0.02 is the default value.",
                              this, SLOT(updateJrkScale()));
  yaw_scale_property_ =
      new rviz::FloatProperty("YawScale", 0.05, "0.05 is the default value.",
                              this, SLOT(updateYawScale()));
  vel_vis_property_ = new rviz::BoolProperty("VelVis", 0, "Visualize Vel?",
                                             this, SLOT(updateVelVis()));
  acc_vis_property_ = new rviz::BoolProperty("AccVis", 0, "Visualize Acc?",
                                             this, SLOT(updateAccVis()));
  jrk_vis_property_ = new rviz::BoolProperty("JrkVis", 0, "Visualize Jrk?",
                                             this, SLOT(updateJrkVis()));
  yaw_vis_property_ = new rviz::BoolProperty("YawVis", 0, "Visualize Yaw?",
                                             this, SLOT(updateYawVis()));
}

void TrajectoryDisplay::onInitialize() { MFDClass::onInitialize(); }

TrajectoryDisplay::~TrajectoryDisplay() {}

void TrajectoryDisplay::reset() {
  MFDClass::reset();
  visual_ = nullptr;
}

void TrajectoryDisplay::updateVelVis() { visualizeMessage(); }

void TrajectoryDisplay::updateAccVis() { visualizeMessage(); }

void TrajectoryDisplay::updateJrkVis() { visualizeMessage(); }

void TrajectoryDisplay::updateYawVis() { visualizeMessage(); }

void TrajectoryDisplay::updatePosColorAndAlpha() {
  Ogre::ColourValue color = pos_color_property_->getOgreColor();
  if (visual_) visual_->setPosColor(color.r, color.g, color.b, 1);
}

void TrajectoryDisplay::updateVelColorAndAlpha() {
  Ogre::ColourValue color = vel_color_property_->getOgreColor();
  if (visual_) visual_->setVelColor(color.r, color.g, color.b, 1);
}

void TrajectoryDisplay::updateAccColorAndAlpha() {
  Ogre::ColourValue color = acc_color_property_->getOgreColor();
  if (visual_) visual_->setAccColor(color.r, color.g, color.b, 1);
}

void TrajectoryDisplay::updateJrkColorAndAlpha() {
  Ogre::ColourValue color = jrk_color_property_->getOgreColor();
  if (visual_) visual_->setJrkColor(color.r, color.g, color.b, 1);
}

void TrajectoryDisplay::updateYawColorAndAlpha() {
  Ogre::ColourValue color = yaw_color_property_->getOgreColor();
  if (visual_) visual_->setYawColor(color.r, color.g, color.b, 1);
}

void TrajectoryDisplay::updatePosScale() {
  float s = pos_scale_property_->getFloat();
  if (visual_) visual_->setPosScale(s);
}

void TrajectoryDisplay::updateVelScale() {
  float s = vel_scale_property_->getFloat();
  if (visual_) visual_->setVelScale(s);
}

void TrajectoryDisplay::updateAccScale() {
  float s = acc_scale_property_->getFloat();
  if (visual_) visual_->setAccScale(s);
}

void TrajectoryDisplay::updateJrkScale() {
  float s = jrk_scale_property_->getFloat();
  if (visual_) visual_->setJrkScale(s);
}

void TrajectoryDisplay::updateYawScale() {
  float s = yaw_scale_property_->getFloat();
  if (visual_) visual_->setYawScale(s);
}

void TrajectoryDisplay::updateYawTriangleScale() { visualizeMessage(); }

void TrajectoryDisplay::updateYawTriangleAngle() { visualizeMessage(); }

void TrajectoryDisplay::updateNum() { visualizeMessage(); }

void TrajectoryDisplay::updateYawNum() { visualizeMessage(); }

void TrajectoryDisplay::processMessage(
    const planning_ros_msgs::Trajectory::ConstPtr &msg) {
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position_, orientation_)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  trajectory_ = *msg;

  visualizeMessage();
}

void TrajectoryDisplay::visualizeMessage() {
  visual_.reset(new TrajectoryVisual(context_->getSceneManager(), scene_node_));

  if (trajectory_.primitives.empty() || !pos_color_property_ ||
      !vel_color_property_ || !acc_color_property_ || !yaw_color_property_ ||
      !pos_scale_property_ || !vel_scale_property_ || !acc_scale_property_ ||
      !yaw_scale_property_ || !yaw_triangle_scale_property_ ||
      !yaw_triangle_angle_property_ || !vel_vis_property_ ||
      !acc_vis_property_ || !jrk_vis_property_ || !yaw_vis_property_ ||
      !num_property_ || !yaw_num_property_)
    return;

  float n = num_property_->getInt();
  visual_->setNum(n);

  float yaw_n = yaw_num_property_->getInt();
  visual_->setYawNum(yaw_n);

  bool vel_vis = vel_vis_property_->getBool();
  visual_->setVelVis(vel_vis);

  bool acc_vis = acc_vis_property_->getBool();
  visual_->setAccVis(acc_vis);

  bool jrk_vis = jrk_vis_property_->getBool();
  visual_->setJrkVis(jrk_vis);

  bool yaw_vis = yaw_vis_property_->getBool();
  visual_->setYawVis(yaw_vis);

  float yaw_tria_scale = yaw_triangle_scale_property_->getFloat();
  visual_->setYawTriangleScale(yaw_tria_scale);

  float yaw_tria_angle = yaw_triangle_angle_property_->getFloat();
  visual_->setYawTriangleAngle(yaw_tria_angle);

  visual_->setMessage(trajectory_);

  visual_->setFramePosition(position_);
  visual_->setFrameOrientation(orientation_);

  float pos_scale = pos_scale_property_->getFloat();
  visual_->setPosScale(pos_scale);
  float vel_scale = vel_scale_property_->getFloat();
  visual_->setVelScale(vel_scale);
  float acc_scale = acc_scale_property_->getFloat();
  visual_->setAccScale(acc_scale);
  float jrk_scale = jrk_scale_property_->getFloat();
  visual_->setJrkScale(jrk_scale);
  float yaw_scale = yaw_scale_property_->getFloat();
  visual_->setYawScale(yaw_scale);

  Ogre::ColourValue pos_color = pos_color_property_->getOgreColor();
  visual_->setPosColor(pos_color.r, pos_color.g, pos_color.b, 1);
  Ogre::ColourValue vel_color = vel_color_property_->getOgreColor();
  visual_->setVelColor(vel_color.r, vel_color.g, vel_color.b, 1);
  Ogre::ColourValue acc_color = acc_color_property_->getOgreColor();
  visual_->setAccColor(acc_color.r, acc_color.g, acc_color.b, 1);
  Ogre::ColourValue jrk_color = jrk_color_property_->getOgreColor();
  visual_->setJrkColor(jrk_color.r, jrk_color.g, jrk_color.b, 1);
  Ogre::ColourValue yaw_color = yaw_color_property_->getOgreColor();
  visual_->setYawColor(yaw_color.r, yaw_color.g, yaw_color.b, 1);
}
}  // namespace planning_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning_rviz_plugins::TrajectoryDisplay, rviz::Display)
