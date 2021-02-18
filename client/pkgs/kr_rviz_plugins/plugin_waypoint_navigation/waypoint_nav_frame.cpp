/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */
/* Author: Dinesh Thakur - Modified for waypoint navigation */

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <interactive_markers/interactive_marker_server.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rviz/display_context.h>
#include <tf/tf.h>
#include <yaml-cpp/yaml.h>

#include <QFileDialog>
#include <boost/foreach.hpp>
#include <fstream>
#include <ostream>

#include "waypoint_nav_tool.h"

namespace kr_rviz_plugins {

struct MissionKeywords {
  inline static const std::string kPosition = "position";
};

WaypointFrame::WaypointFrame(
    rviz::DisplayContext *context, std::map<int, Ogre::SceneNode *> *map_ptr,
    interactive_markers::InteractiveMarkerServer *server, int *unique_ind,
    QWidget *parent, WaypointNavTool *wp_tool)
    : QWidget(parent), ui_(new Ui::WaypointNavigationWidget()),
      context_(context), wp_nav_tool_(wp_tool), sn_map_ptr_(map_ptr),
      unique_ind_(unique_ind), server_(server), default_height_(0.0),
      frame_id_("map"),
      selected_marker_name_(std::string(g_wp_name_prefix) + "1") {
  scene_manager_ = context_->getSceneManager();

  // set up the GUI
  ui_->setupUi(this);

  wp_pub_ = nh_.advertise<nav_msgs::Path>("waypoints", 1);
  sm_pub_ =
      nh_.advertise<mav_high_level_msgs::StateTransition>("state_trigger", 1);
  make_sub_ = nh_.subscribe("make_marker", 1, &WaypointFrame::makeCB, this);

  // connect the Qt signals and slots
  connect(ui_->publish_wp_button, SIGNAL(clicked()), this,
          SLOT(publishButtonClicked()));
  // connect(ui_->topic_line_edit, SIGNAL(editingFinished()), this,
  // SLOT(topicChanged())); connect(ui_->frame_line_edit,
  // SIGNAL(editingFinished()), this, SLOT(frameChanged()));
  connect(ui_->wp_height_doubleSpinBox, SIGNAL(valueChanged(double)), this,
          SLOT(heightChanged(double)));
  connect(ui_->clear_all_button, SIGNAL(clicked()), this,
          SLOT(clearAllWaypoints()));

  connect(ui_->x_doubleSpinBox, SIGNAL(valueChanged(double)), this,
          SLOT(poseChanged(double)));
  connect(ui_->y_doubleSpinBox, SIGNAL(valueChanged(double)), this,
          SLOT(poseChanged(double)));
  connect(ui_->z_doubleSpinBox, SIGNAL(valueChanged(double)), this,
          SLOT(poseChanged(double)));
  connect(ui_->yaw_doubleSpinBox, SIGNAL(valueChanged(double)), this,
          SLOT(poseChanged(double)));

  connect(ui_->save_wp_button, SIGNAL(clicked()), this,
          SLOT(saveButtonClicked()));
  connect(ui_->load_wp_button, SIGNAL(clicked()), this,
          SLOT(loadButtonClicked()));
}

WaypointFrame::~WaypointFrame() {
  delete ui_;
  sn_map_ptr_ = nullptr;
}

void WaypointFrame::enable() {
  // activate the frame
  show();
}

void WaypointFrame::disable() {
  wp_pub_.shutdown();
  hide();
}

void WaypointFrame::saveButtonClicked() {
  QString filename =
      QFileDialog::getSaveFileName(0, tr("Save Mission"), "waypoints",
                                   tr("Mission Files (*.bag *.yaml *.json)"));

  if (filename.isEmpty()) {
    ROS_ERROR("No mission filename selected");
    return;
  }

  const std::string filename_str = filename.toStdString();
  ROS_INFO_STREAM("saving waypoints to " << filename_str);
  if (filename.endsWith(".bag")) {
    saveToBag(filename_str);
  } else if (filename.endsWith(".yaml")) {
    saveToYaml(filename_str);
  } else if (filename.endsWith(".json")) {
    saveToJson(filename_str);
  } else {
    ROS_INFO_STREAM("Invalid mission file format: " << filename_str);
  }
}

void WaypointFrame::saveToBag(const std::string &filename) {
  rosbag::Bag bag;
  try {
    bag.open(filename, rosbag::bagmode::Write);
  } catch (const rosbag::BagIOException &e) {
    ROS_ERROR("could not open bag %s", filename.c_str());
    return;
  }

  nav_msgs::Path path;

  std::map<int, Ogre::SceneNode *>::iterator sn_it;
  for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); ++sn_it) {
    Ogre::Vector3 position;
    position = sn_it->second->getPosition();

    geometry_msgs::PoseStamped pos;
    pos.pose.position.x = position.x;
    pos.pose.position.y = position.y;
    pos.pose.position.z = position.z;

    Ogre::Quaternion quat;
    quat = sn_it->second->getOrientation();
    pos.pose.orientation.x = quat.x;
    pos.pose.orientation.y = quat.y;
    pos.pose.orientation.z = quat.z;
    pos.pose.orientation.w = quat.w;

    path.poses.push_back(pos);
  }

  path.header.frame_id = frame_id_.toStdString();

  bag.write("waypoints", ros::Time::now(), path);
  bag.close();
}

void WaypointFrame::saveToYaml(const std::string &filename) {
  YAML::Emitter out;
  std::map<int, Ogre::SceneNode *>::iterator sn_it;
  out << YAML::BeginSeq;
  for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); ++sn_it) {
    const Ogre::Vector3 position = sn_it->second->getPosition();

    out << YAML::BeginMap;
    out << YAML::Key << MissionKeywords::kPosition;
    out << YAML::Value;
    out << YAML::Flow;
    out << YAML::BeginSeq;
    out << position.x << position.y << position.z;
    out << YAML::EndSeq;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;

  std::ofstream fout(filename);
  fout << out.c_str();
}

void WaypointFrame::saveToJson(const std::string &filename) {}

void WaypointFrame::loadButtonClicked() {
  const QString filename = QFileDialog::getOpenFileName(
      0, tr("Load Mission"), "~/", tr("Mission Files (*.bag *.yaml *.json)"));

  if (filename.isEmpty()) {
    ROS_ERROR("No mission file selected");
    return;
  }

  const std::string filename_str = filename.toStdString();
  ROS_INFO("loading waypoints from %s", filename_str.c_str());
  if (filename.endsWith(".bag")) {
    loadFromBag(filename_str);
  } else if (filename.endsWith(".yaml")) {
    loadFromYaml(filename_str);
  } else if (filename.endsWith(".json")) {
    loadFromJson(filename_str);
  } else {
    ROS_INFO_STREAM("Invalid mission file format: " << filename_str);
  }
}

void WaypointFrame::loadFromBag(const std::string &filename) {
  rosbag::Bag bag;
  try {
    bag.open(filename, rosbag::bagmode::Read);
  } catch (const rosbag::BagIOException &e) {
    ROS_ERROR("could not open bag %s", filename.c_str());
    return;
  }

  std::vector<std::string> topics;
  topics.push_back(std::string("waypoints"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  BOOST_FOREACH (rosbag::MessageInstance const m, view) {
    nav_msgs::Path::ConstPtr p = m.instantiate<nav_msgs::Path>();
    if (p == nullptr)
      continue;
    ROS_INFO("n waypoints %zu", p->poses.size());

    for (size_t i = 0; i < p->poses.size(); i++) {
      geometry_msgs::PoseStamped pos = p->poses[i];
      Ogre::Vector3 position;
      position.x = pos.pose.position.x;
      position.y = pos.pose.position.y;
      position.z = pos.pose.position.z;

      Ogre::Quaternion quat;
      quat.x = pos.pose.orientation.x;
      quat.y = pos.pose.orientation.y;
      quat.z = pos.pose.orientation.z;
      quat.w = pos.pose.orientation.w;

      wp_nav_tool_->makeIm(position, quat,
                           ui_->sixDcheckBox->checkState() == Qt::Checked);
    }
  }
}

void WaypointFrame::loadFromYaml(const std::string &filename) {
  YAML::Node root_node = YAML::LoadFile(filename);

  // Iterate over each waypoint
  for (auto it_wpt = root_node.begin(); it_wpt != root_node.end(); ++it_wpt) {
    // Get waypoint node, which is a list of key-value pairs
    YAML::Node wpt_node = *it_wpt;
    for (auto it_map = wpt_node.begin(); it_map != wpt_node.end(); ++it_map) {
      // Do stuff depends on the key
      const std::string key = it_map->first.as<std::string>();
      if (key == MissionKeywords::kPosition) {
        YAML::Node pos_node = it_map->second;

        Ogre::Vector3 position;
        position.x = pos_node[0].as<double>();
        position.y = pos_node[1].as<double>();
        position.z = pos_node[2].as<double>();
        Ogre::Quaternion quat;
        wp_nav_tool_->makeIm(position, quat,
                             ui_->sixDcheckBox->checkState() == Qt::Checked);
      }
    }
  }
}

void WaypointFrame::loadFromJson(const std::string &filename) {}

void WaypointFrame::publishButtonClicked() {
  nav_msgs::Path path;

  std::map<int, Ogre::SceneNode *>::iterator sn_it;
  for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); ++sn_it) {
    Ogre::Vector3 position;
    position = sn_it->second->getPosition();

    geometry_msgs::PoseStamped pos;
    pos.pose.position.x = position.x;
    pos.pose.position.y = position.y;
    pos.pose.position.z = position.z;

    Ogre::Quaternion quat;
    quat = sn_it->second->getOrientation();
    pos.pose.orientation.x = quat.x;
    pos.pose.orientation.y = quat.y;
    pos.pose.orientation.z = quat.z;
    pos.pose.orientation.w = quat.w;

    path.poses.push_back(pos);
  }

  path.header.frame_id = frame_id_.toStdString();
  wp_pub_.publish(path);

  ros::Rate r(20);
  r.sleep();
  mav_high_level_msgs::StateTransition st;
  st.transition.data = std::string("waypoints");
  sm_pub_.publish(st);
}

void WaypointFrame::clearAllWaypoints() {
  // destroy the ogre scene nodes
  std::map<int, Ogre::SceneNode *>::iterator sn_it;
  for (sn_it = sn_map_ptr_->begin(); sn_it != sn_map_ptr_->end(); ++sn_it) {
    scene_manager_->destroySceneNode(sn_it->second);
  }

  // clear the waypoint map and reset index
  sn_map_ptr_->clear();
  *unique_ind_ = 0;

  // clear the interactive markers
  server_->clear();
  server_->applyChanges();
}

void WaypointFrame::heightChanged(double h) { default_height_ = h; }

void WaypointFrame::setSelectedMarkerName(std::string name) {
  selected_marker_name_ = name;
}

void WaypointFrame::poseChanged(double val) {
  auto sn_entry = sn_map_ptr_->end();
  try {
    const int selected_marker_idx =
        std::stoi(selected_marker_name_.substr(strlen(g_wp_name_prefix)));
    sn_entry = sn_map_ptr_->find(selected_marker_idx);
  } catch (const std::logic_error &e) {
    ROS_ERROR_STREAM(e.what());
    return;
  }

  if (sn_entry == sn_map_ptr_->end())
    ROS_ERROR("%s not found in map", selected_marker_name_.c_str());
  else {
    Ogre::Vector3 position;
    Ogre::Quaternion quat;
    getPose(position, quat);

    sn_entry->second->setPosition(position);
    sn_entry->second->setOrientation(quat);

    std::stringstream wp_name;
    wp_name << g_wp_name_prefix << sn_entry->first;
    std::string wp_name_str(wp_name.str());

    visualization_msgs::InteractiveMarker int_marker;
    if (server_->get(wp_name_str, int_marker)) {
      int_marker.pose.position.x = position.x;
      int_marker.pose.position.y = position.y;
      int_marker.pose.position.z = position.z;

      int_marker.pose.orientation.x = quat.x;
      int_marker.pose.orientation.y = quat.y;
      int_marker.pose.orientation.z = quat.z;
      int_marker.pose.orientation.w = quat.w;

      server_->setPose(wp_name_str, int_marker.pose, int_marker.header);
    }
    server_->applyChanges();
    publishButtonClicked();
  }
}

void WaypointFrame::frameChanged() {}

void WaypointFrame::topicChanged() {}

void WaypointFrame::makeCB(const geometry_msgs::Pose &pose) {
  Ogre::Vector3 pos(pose.position.x, pose.position.y, pose.position.z);
  Ogre::Quaternion quat(pose.orientation.w, pose.orientation.x,
                        pose.orientation.y, pose.orientation.z);
  wp_nav_tool_->makeIm(pos, quat,
                       ui_->sixDcheckBox->checkState() == Qt::Checked);
  server_->applyChanges();
  publishButtonClicked();
}

void WaypointFrame::setWpCount(int size) {
  std::ostringstream stringStream;
  stringStream << "num wp: " << size;
  std::string st = stringStream.str();

  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  ui_->waypoint_count_label->setText(QString::fromStdString(st));
}

void WaypointFrame::setConfig(QString topic, QString frame, float height) {
  {
    boost::mutex::scoped_lock lock(frame_updates_mutex_);
    ui_->wp_height_doubleSpinBox->blockSignals(true);
    ui_->wp_height_doubleSpinBox->setValue(height);
    ui_->wp_height_doubleSpinBox->blockSignals(false);
  }
  topicChanged();
  frameChanged();
  heightChanged(height);
}

void WaypointFrame::getPose(Ogre::Vector3 &position, Ogre::Quaternion &quat) {
  {
    boost::mutex::scoped_lock lock(frame_updates_mutex_);
    position.x = ui_->x_doubleSpinBox->value();
    position.y = ui_->y_doubleSpinBox->value();
    position.z = ui_->z_doubleSpinBox->value();
    double yaw = ui_->yaw_doubleSpinBox->value();

    tf::Quaternion qt = tf::createQuaternionFromYaw(yaw);
    quat.x = qt.x();
    quat.y = qt.y();
    quat.z = qt.z();
    quat.w = qt.w();
  }
}

void WaypointFrame::setPose(const Ogre::Vector3 &position,
                            const Ogre::Quaternion &quat) {
  {
    // block spinbox signals
    ui_->x_doubleSpinBox->blockSignals(true);
    ui_->y_doubleSpinBox->blockSignals(true);
    ui_->z_doubleSpinBox->blockSignals(true);
    ui_->yaw_doubleSpinBox->blockSignals(true);

    ui_->x_doubleSpinBox->setValue(position.x);
    ui_->y_doubleSpinBox->setValue(position.y);
    ui_->z_doubleSpinBox->setValue(position.z);

    tf::Quaternion qt(quat.x, quat.y, quat.z, quat.w);
    ui_->yaw_doubleSpinBox->setValue(tf::getYaw(qt));

    // enable the signals
    ui_->x_doubleSpinBox->blockSignals(false);
    ui_->y_doubleSpinBox->blockSignals(false);
    ui_->z_doubleSpinBox->blockSignals(false);
    ui_->yaw_doubleSpinBox->blockSignals(false);
  }
}

void WaypointFrame::setWpLabel(Ogre::Vector3 position) {
  {
    std::ostringstream stringStream;
    stringStream.precision(2);
    stringStream << "x: " << position.x << " y: " << position.y
                 << " z: " << position.z;
    std::string label = stringStream.str();

    ui_->sel_wp_label->setText(QString::fromStdString(label));
  }
}

double WaypointFrame::getDefaultHeight() {
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return default_height_;
}

QString WaypointFrame::getFrameId() {
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return frame_id_;
}

QString WaypointFrame::getOutputTopic() {
  boost::mutex::scoped_lock lock(frame_updates_mutex_);
  return output_topic_;
}

} // namespace kr_rviz_plugins
