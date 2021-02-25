#include "primitive_visual.h"

namespace planning_rviz_plugins {

PrimitiveVisual::PrimitiveVisual(Ogre::SceneManager *scene_manager,
                                 Ogre::SceneNode *parent_node) {
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();
}

PrimitiveVisual::~PrimitiveVisual() {
  scene_manager_->destroySceneNode(frame_node_);
}

void PrimitiveVisual::setMessage(
    const std::vector<planning_ros_msgs::Primitive> &msgs) {
  poss_.clear();
  vels_.clear();
  accs_.clear();
  jrks_.clear();
  yaws_.clear();

  if (num_ < 2) return;

  for (const auto &pr : msgs) {
    for (size_t i = 0; i < pr.cx.size(); i++)
      if (std::isnan(pr.cx[i]) || std::isinf(pr.cx[i])) return;
    for (size_t i = 0; i < pr.cy.size(); i++)
      if (std::isnan(pr.cy[i]) || std::isinf(pr.cy[i])) return;
    for (size_t i = 0; i < pr.cz.size(); i++)
      if (std::isnan(pr.cz[i]) || std::isinf(pr.cz[i])) return;
    for (size_t i = 0; i < pr.cyaw.size(); i++)
      if (std::isnan(pr.cyaw[i]) || std::isinf(pr.cyaw[i])) return;
  }

  const size_t N = msgs.size();
  if (N > 10000) {
    printf(ANSI_COLOR_YELLOW
           "N [%zu] is greater than the max number "
           "limitation 10000, don't plot this "
           "message!\n" ANSI_COLOR_RESET,
           N);
    return;
  }

  poss_.resize(N * (num_ - 1));
  if (vel_vis_) vels_.resize(N * num_);
  if (acc_vis_) accs_.resize(N * num_);
  if (jrk_vis_) jrks_.resize(N * num_);
  if (yaw_vis_) yaws_.resize(N * yaw_num_);

  decimal_t theta = M_PI / 2;
  Mat3f R;
  R << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;

  for (size_t n = 0; n < N; n++) {
    const auto p = toPrimitive3D(msgs[n]);
    const auto waypoints = p.sample(num_ - 1);

    for (size_t i = 0; i < waypoints.size(); i++) {
      const auto p1 = waypoints[i];
      const Ogre::Vector3 pos1(p1.pos(0), p1.pos(1), p1.pos(2));

      if (i < waypoints.size() - 1) {
        const auto p2 = waypoints[i + 1];
        const Ogre::Vector3 pos2(p2.pos(0), p2.pos(1), p2.pos(2));
        poss_[n * (num_ - 1) + i].reset(
            new rviz::BillboardLine(scene_manager_, frame_node_));
        poss_[n * (num_ - 1) + i]->addPoint(pos1);
        poss_[n * (num_ - 1) + i]->addPoint(pos2);
      }

      if (vel_vis_) {
        const Vec3f p3 = p1.pos + R * p1.vel;
        const Ogre::Vector3 pos3(p3(0), p3(1), p3(2));
        vels_[n * num_ + i].reset(
            new rviz::BillboardLine(scene_manager_, frame_node_));
        vels_[n * num_ + i]->addPoint(pos1);
        vels_[n * num_ + i]->addPoint(pos3);
      }

      if (acc_vis_) {
        const Vec3f p3 = p1.pos + R * p1.acc;
        const Ogre::Vector3 pos3(p3(0), p3(1), p3(2));
        accs_[n * num_ + i].reset(
            new rviz::BillboardLine(scene_manager_, frame_node_));
        accs_[n * num_ + i]->addPoint(pos1);
        accs_[n * num_ + i]->addPoint(pos3);
      }

      if (jrk_vis_) {
        const Vec3f p3 = p1.pos + R * p1.jrk;
        const Ogre::Vector3 pos3(p3(0), p3(1), p3(2));
        jrks_[n * num_ + i].reset(
            new rviz::BillboardLine(scene_manager_, frame_node_));
        jrks_[n * num_ + i]->addPoint(pos1);
        jrks_[n * num_ + i]->addPoint(pos3);
      }
    }

    if (yaw_vis_ && yaw_num_ >= 2) {
      Vec3f d(syaw_, 0, 0);
      const auto yaw_waypoints = p.sample(yaw_num_ - 1);
      for (int i = 0; i < yaw_num_; i++) {
        yaws_[n * yaw_num_ + i].reset(
            new rviz::BillboardLine(scene_manager_, frame_node_));
        const auto keyframe = yaw_waypoints[i];
        decimal_t yaw = keyframe.yaw;
        decimal_t yaw1 = yaw + dyaw_;
        decimal_t yaw2 = yaw - dyaw_;
        Mat3f Ryaw1, Ryaw2;
        Ryaw1 << cos(yaw1), -sin(yaw1), 0, sin(yaw1), cos(yaw1), 0, 0, 0, 1;
        Ryaw2 << cos(yaw2), -sin(yaw2), 0, sin(yaw2), cos(yaw2), 0, 0, 0, 1;

        Vec3f p1 = keyframe.pos;
        Vec3f p2 = keyframe.pos + Ryaw1 * d;
        Vec3f p3 = keyframe.pos + Ryaw2 * d;
        Vec3f p4 = (p2 + p3) / 2;
        Ogre::Vector3 pos1(p1(0), p1(1), p1(2));
        Ogre::Vector3 pos2(p2(0), p2(1), p2(2));
        Ogre::Vector3 pos3(p3(0), p3(1), p3(2));
        Ogre::Vector3 pos4(p4(0), p4(1), p4(2));
        yaws_[n * yaw_num_ + i]->addPoint(pos1);
        yaws_[n * yaw_num_ + i]->addPoint(pos2);
        yaws_[n * yaw_num_ + i]->addPoint(pos3);
        yaws_[n * yaw_num_ + i]->addPoint(pos1);
        yaws_[n * yaw_num_ + i]->addPoint(pos4);
      }
    }
  }
}

void PrimitiveVisual::setNum(int n) { num_ = n; }

void PrimitiveVisual::setYawNum(int n) { yaw_num_ = n; }

void PrimitiveVisual::setVelVis(bool vis) { vel_vis_ = vis; }

void PrimitiveVisual::setAccVis(bool vis) { acc_vis_ = vis; }

void PrimitiveVisual::setJrkVis(bool vis) { jrk_vis_ = vis; }

void PrimitiveVisual::setYawVis(bool vis) { yaw_vis_ = vis; }

void PrimitiveVisual::setFramePosition(const Ogre::Vector3 &position) {
  frame_node_->setPosition(position);
}

void PrimitiveVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
  frame_node_->setOrientation(orientation);
}

void PrimitiveVisual::setPosColor(float r, float g, float b, float a) {
  for (auto &it : poss_) it->setColor(r, g, b, a);
}

void PrimitiveVisual::setVelColor(float r, float g, float b, float a) {
  for (auto &it : vels_) it->setColor(r, g, b, a);
}

void PrimitiveVisual::setAccColor(float r, float g, float b, float a) {
  for (auto &it : accs_) it->setColor(r, g, b, a);
}

void PrimitiveVisual::setJrkColor(float r, float g, float b, float a) {
  for (auto &it : jrks_) it->setColor(r, g, b, a);
}

void PrimitiveVisual::setYawColor(float r, float g, float b, float a) {
  for (auto &it : yaws_) it->setColor(r, g, b, a);
}

void PrimitiveVisual::setPosScale(float s) {
  for (auto &it : poss_) it->setLineWidth(s);
}

void PrimitiveVisual::setVelScale(float s) {
  for (auto &it : vels_) it->setLineWidth(s);
}

void PrimitiveVisual::setAccScale(float s) {
  for (auto &it : accs_) it->setLineWidth(s);
}

void PrimitiveVisual::setJrkScale(float s) {
  for (auto &it : jrks_) it->setLineWidth(s);
}

void PrimitiveVisual::setYawScale(float s) {
  for (auto &it : yaws_) it->setLineWidth(s);
}

void PrimitiveVisual::setYawTriangleScale(float s) { syaw_ = s; }

void PrimitiveVisual::setYawTriangleAngle(float d) { dyaw_ = d; }

}  // namespace planning_rviz_plugins
