// Copyright 2016 Michael Watterson

#include "spline_trajectory_visual.h"  // NOLINT()

#include <ros/console.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/line.h>
#include <rviz/ogre_helpers/shape.h>

namespace planning_rviz_plugins {

// BEGIN_TUTORIAL
SplineTrajectoryVisual::SplineTrajectoryVisual(
    Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node) {
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the SplineTrajectory's header
  // frame relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();

  // We create the arrow object within the frame node so that we can
  // set its position and direction relative to its header frame.
  //  acceleration_arrow_.reset(new rviz::Arrow( scene_manager_, frame_node_ ));

  // allocate space for line objects
  resetTrajPoints(num_traj_points_, num_vel_points_, vel_on_, acc_on_,
                  hopf_on_);

  // allocate space of tangent objects
}

SplineTrajectoryVisual::~SplineTrajectoryVisual() {
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(frame_node_);
}

// make visualization robust to arbitrary length vectors
Ogre::Vector3 SplineTrajectoryVisual::vecFromVecD(const Eigen::VectorXd &vec) {
  Ogre::Vector3 vecr;
  if (vec.rows() == 0)
    vecr = Ogre::Vector3(0.0, 0.0, 0.0);
  else if (vec.rows() == 1)
    vecr = Ogre::Vector3(vec(0), 0.0, 0.0);
  else if (vec.rows() == 2)
    vecr = Ogre::Vector3(vec(0), vec(1), 0.0);
  else
    vecr = Ogre::Vector3(vec(0), vec(1), vec(2));
  return vecr;
}

void SplineTrajectoryVisual::setStyle(int style) {
  if (style == 0)
    style_ = Style::Mike;
  else if (style == 1)
    style_ = Style::Sikang;
  else if (style == 2)
    style_ = Style::CJ;
  else
    style_ = Style::Hopf;
}

void SplineTrajectoryVisual::resetTrajPoints(int traj_points,
                                             int tangent_points, bool use_v,
                                             bool use_a, bool use_h) {
  num_traj_points_ = traj_points;
  num_vel_points_ = tangent_points;
  vel_on_ = use_v;
  acc_on_ = use_a;
  hopf_on_ = use_h;
}

void SplineTrajectoryVisual::draw() {
  trajectory_lines_.clear();
  trajectory_balls_.clear();
  vel_arrows_.clear();
  acc_arrows_.clear();
  hopf_arrows_.clear();

  // allocate space for line objects
  trajectory_lines_.reserve(num_traj_points_);
  if (vel_on_) vel_arrows_.reserve(num_vel_points_);
  if (acc_on_) acc_arrows_.reserve(num_vel_points_);
  if (hopf_on_) hopf_arrows_.reserve(3 * num_vel_points_);

  // allocate objects
  if (style_ == Style::Mike || style_ == Style::Hopf) {
    trajectory_balls_.reserve(num_traj_points_ + 1);
    trajectory_balls_.push_back(std::make_shared<rviz::Shape>(
        rviz::Shape::Type::Sphere, scene_manager_, frame_node_));
  }
  for (int i = 0; i < num_traj_points_; i++) {
    if (style_ == Style::Mike || style_ == Style::Hopf) {
      trajectory_lines_.push_back(std::make_shared<rviz::Shape>(
          rviz::Shape::Type::Cylinder, scene_manager_, frame_node_));
      trajectory_balls_.push_back(std::make_shared<rviz::Shape>(
          rviz::Shape::Type::Sphere, scene_manager_, frame_node_));
    } else {
      trajectory_lines_.push_back(std::make_shared<rviz::Shape>(
          rviz::Shape::Type::Cylinder, scene_manager_, frame_node_));
    }
  }

  if (style_ != Style::CJ) {
    for (int i = 0; i < num_vel_points_; i++) {
      if (style_ == Style::Sikang) {
        if (vel_on_) {
          vel_arrows_.push_back(std::make_shared<rviz::Shape>(
              rviz::Shape::Type::Cylinder, scene_manager_, frame_node_));
        }
        if (acc_on_) {
          acc_arrows_.push_back(std::make_shared<rviz::Shape>(
              rviz::Shape::Type::Cylinder, scene_manager_, frame_node_));
        }
      } else if (style_ == Style::Mike) {
        if (vel_on_) {
          vel_arrows_.push_back(
              std::make_shared<rviz::Arrow>(scene_manager_, frame_node_));
        }
        if (acc_on_) {
          acc_arrows_.push_back(
              std::make_shared<rviz::Arrow>(scene_manager_, frame_node_));
        }
        if (hopf_on_) {
          for (int c = 0; c < 3; c++) {
            hopf_arrows_.push_back(std::make_shared<rviz::Shape>(
                rviz::Shape::Type::Cylinder, scene_manager_, frame_node_));
          }
        }
      } else if (style_ == Style::Hopf) {
        if (vel_on_) {
          for (int i = 0; i < 3; i++)
            vel_arrows_.push_back(std::make_shared<rviz::Shape>(
                rviz::Shape::Type::Cylinder, scene_manager_, frame_node_));
        }
      }
    }
  }

  if (traj_ != NULL) setCurve();
}

void SplineTrajectoryVisual::setCurve() {
  if (traj_ == NULL) return;
  double t_total = 0;
  if (traj_->data.size() > 0) {
    t_total = traj_->data[0].t_total;
  }
  if (t_total <= 0) {
    ROS_ERROR("Latest trajectory is invalid");
    return;  // trajectory is invalid
  }
  //    ROS_INFO_STREAM("Drawing trajectory with " << num_traj_points_ << "
  //    points.");

  Eigen::VectorXd p1, p2, p3, p4;

  //  traj_opt::Vec4 p1 = traj_opt::Vec4::Zero();
  //  traj_opt::Vec4 p2 = traj_opt::Vec4::Zero();

  p1 = evaluate(0, 0);
  double v_max = 1;
  float r_max = 237 / 255.0;
  float g_max = 10 / 255.0;
  float b_max = 63 / 255.0;
  float r_min = 231 / 255.0;
  float g_min = 114 / 255.0;
  float b_min = 0 / 255.0;

  if (style_ == Style::CJ) {
    double dt = t_total / static_cast<double>(num_traj_points_);
    for (int i = 1; i <= num_traj_points_; i++) {
      double dtt = static_cast<double>(i) * dt;
      auto v = evaluate(dtt, 1);
      if (v.norm() > v_max) v_max = v.norm();
    }
  }

  Ogre::Vector3 op1 = vecFromVecD(p1);
  Ogre::Vector3 balls(thickness_, thickness_, thickness_);
  // draw curve
  double dt = t_total / static_cast<double>(num_traj_points_);
  for (int i = 1; i <= num_traj_points_; i++) {
    double dtt = static_cast<double>(i) * dt;
    p2 = evaluate(dtt, 0);
    Ogre::Vector3 op2 = vecFromVecD(p2);

    if (style_ == Style::Mike) {
      std::shared_ptr<rviz::Shape> sp =
          std::dynamic_pointer_cast<rviz::Shape>(trajectory_lines_.at(i - 1));
      setShapeFromPosePair(op1, op2, thickness_, sp.get());
      if (i == 1) {
        trajectory_balls_.at(0)->setPosition(op1);
        trajectory_balls_.at(0)->setScale(balls);
      }
      trajectory_balls_.at(i)->setPosition(op2);
      trajectory_balls_.at(i)->setScale(balls);
    } else if (style_ == Style::Sikang) {
      std::shared_ptr<rviz::Shape> sp =
          std::dynamic_pointer_cast<rviz::Shape>(trajectory_lines_.at(i - 1));
      setShapeFromPosePair(op1, op2, thickness_, sp.get());
      // sp->setPoints(op1, op2);
    } else if (style_ == Style::CJ) {
      std::shared_ptr<rviz::Shape> sp =
          std::dynamic_pointer_cast<rviz::Shape>(trajectory_lines_.at(i - 1));
      setShapeFromPosePair(op1, op2, thickness_, sp.get());
      if (vel_on_) {
        Eigen::VectorXd v;
        v = evaluate(dtt, 1);
        float r = v.norm() / v_max * (r_max - r_min) + r_min;
        float g = v.norm() / v_max * (g_max - g_min) + g_min;
        float b = v.norm() / v_max * (b_max - b_min) + b_min;
        sp->setColor(r, g, b, 1.0);
      }
    }
    op1 = op2;
  }

  // Eigen::Quaternion<double> rot(1, 0, 0, 0);
  // // rotate tangent vectors about z
  // if (style_ == Style::Sikang)
  //   rot = Eigen::Quaternion<double>(0.707, 0, 0, -0.707);

  // // draw tangents
  // dt = t_total / static_cast<double>(num_vel_points_);
  // for (int i = 0; i < num_vel_points_; i++) {
  //   //      std::cout << "i " << i << std::endl;
  //   if (!vel_on_ && !acc_on_ && !hopf_on_) break;
  //   double dtt = static_cast<double>(i) * dt;
  //   ;
  //   op1 = vecFromVecD(evaluate(dtt, 0));

  //   // velocity
  //   if (vel_on_) {
  //     p2 = evaluate(dtt, 1);
  //     Eigen::Vector3d p1_3d = Eigen::Vector3d::Zero();
  //     Eigen::Vector3d p2_3d = Eigen::Vector3d::Zero();
  //     for (int i = 0; i < 3; i++) {
  //       if (i < p1.rows()) p1_3d(i) = p1(i);
  //       if (i < p2.rows()) p2_3d(i) = p2(i);
  //     }
  //     p2_3d = rot.matrix() * p2_3d + p1_3d;
  //     Ogre::Vector3 op2(p2_3d(0), p2_3d(1), p2_3d(2));
  //     if (style_ == Style::Mike) {
  //       std::shared_ptr<rviz::Arrow> sp =
  //           std::dynamic_pointer_cast<rviz::Arrow>(vel_arrows_.at(i));
  //       setShapeFromPosePair(op1, op2, 0.5 * thickness_, sp.get());
  //     } else if (style_ == Style::Sikang) {
  //       std::shared_ptr<rviz::Shape> lp =
  //           std::dynamic_pointer_cast<rviz::Shape>(vel_arrows_.at(i));
  //       setShapeFromPosePair(op1, op2, 0.5 * thickness_, lp.get());
  //       // lp->setPoints(op1, op2);
  //     }
  //   }
  //   if (acc_on_) {
  //     p2 = evaluate(dtt, 2);
  //     Eigen::Vector3d p1_3d = Eigen::Vector3d::Zero();
  //     Eigen::Vector3d p2_3d = Eigen::Vector3d::Zero();
  //     for (int i = 0; i < 3; i++) {
  //       if (i < p1.rows()) p1_3d(i) = p1(i);
  //       if (i < p2.rows()) p2_3d(i) = p2(i);
  //     }

  //     p2_3d = rot.matrix() * p2_3d + p1_3d;
  //     Ogre::Vector3 op2(p2_3d(0), p2_3d(1), p2_3d(2));
  //     if (style_ == Style::Mike) {
  //       std::shared_ptr<rviz::Arrow> sp =
  //           std::dynamic_pointer_cast<rviz::Arrow>(acc_arrows_.at(i));
  //       setShapeFromPosePair(op1, op2, 0.5 * thickness_, sp.get());
  //     } else if (style_ == Style::Sikang) {
  //       std::shared_ptr<rviz::Shape> lp =
  //           std::dynamic_pointer_cast<rviz::Shape>(acc_arrows_.at(i));
  //       setShapeFromPosePair(op1, op2, 0.5 * thickness_, lp.get());
  //       // lp->setPoints(op1, op2);
  //     }
  //   }
  // }
}

void SplineTrajectoryVisual::setMessage(
    const planning_ros_msgs::SplineTrajectory::ConstPtr &msg) {
  // traj_.reset(my_ptr);
  traj_ = msg;
  setCurve();
}

// Position and orientation are passed through to the SceneNode.
void SplineTrajectoryVisual::setFramePosition(const Ogre::Vector3 &position) {
  frame_node_->setPosition(position);
}

void SplineTrajectoryVisual::setFrameOrientation(
    const Ogre::Quaternion &orientation) {
  frame_node_->setOrientation(orientation);
}

// Color is passed through to the Arrow object.
void SplineTrajectoryVisual::setColor(float r, float g, float b, float a) {
  if (style_ == Style::CJ) return;
  for (auto &line : trajectory_lines_) line->setColor(r, g, b, a);
  for (auto &ball : trajectory_balls_) ball->setColor(r, g, b, a);
}
void SplineTrajectoryVisual::setColorV(float r, float g, float b, float a) {
  for (auto &line : vel_arrows_) line->setColor(r, g, b, a);
}
void SplineTrajectoryVisual::setColorA(float r, float g, float b, float a) {
  for (auto &line : acc_arrows_) line->setColor(r, g, b, a);
}
void SplineTrajectoryVisual::setScale(float thick) {
  thickness_ = thick;
  setCurve();
  //    Ogre::Vector3 scale(thickness_, thickness_, thickness_);

  //    for (auto& line : trajectory_lines_) line->setScale(scale);
  // dont do this, it doesn't work
}
void SplineTrajectoryVisual::setShapeFromPosePair(const Ogre::Vector3 &p0,
                                                  const Ogre::Vector3 &p1,
                                                  double scale,
                                                  rviz::Shape *shape) {
  Ogre::Vector3 n = p1 - p0;

  Ogre::Quaternion quat;

  Ogre::Vector3 scalev(scale, n.length(), scale);

  if (!n.isZeroLength()) {
    n.normalise();
    Ogre::Vector3 e0(0, 1, 0);
    float dot = n.dotProduct(e0);
    float theta = std::acos(dot);
    //        std::cout << "dot " << dot << std::endl;
    //        std::cout << "theta " << theta << std::endl;

    Ogre::Vector3 axis = e0.crossProduct(n);
    axis.normalise();

    quat = Ogre::Quaternion(Ogre::Radian(theta), axis);
  }
  //    Ogre::Matrix3 rot;
  //    quat.Inverse().ToRotationMatrix(rot);
  //    scalev = rot * scalev;

  //    quat = Ogre::Quaternion::IDENTITY;
  Ogre::Vector3 pc = 0.5 * (p0 + p1);
  shape->setScale(scalev);
  shape->setPosition(pc);
  shape->setOrientation(quat);
}
void SplineTrajectoryVisual::setShapeFromPosePair(const Ogre::Vector3 &p0,
                                                  const Ogre::Vector3 &p1,
                                                  double scale,
                                                  rviz::Arrow *shape) {
  Ogre::Vector3 n = p1 - p0;

  shape->set(n.length(), scale, 0.25 * n.length(), 3.0 * scale);
  n.normalise();

  shape->setPosition(p0);
  shape->setDirection(n);
}

Eigen::VectorXd SplineTrajectoryVisual::evaluate(double t,
                                                 uint deriv_num) const {
  // TODO make work for derivatives
  Eigen::VectorXd result(traj_->dimensions);

  for (int dim = 0; dim < traj_->dimensions; dim++) {
    auto spline = traj_->data[dim];
    double dt = 0;
    for (auto poly : spline.segs) {
      if (t < dt + poly.dt) {
        result(dim) = 0;
        for (int j = 0; j < poly.coeffs.size(); j++) {
          result(dim) += poly.coeffs[j] * std::pow(t - dt, j);
        }
        break;
      }
      dt += poly.dt;
    }
  }
  return result;
}

}  // namespace planning_rviz_plugins
