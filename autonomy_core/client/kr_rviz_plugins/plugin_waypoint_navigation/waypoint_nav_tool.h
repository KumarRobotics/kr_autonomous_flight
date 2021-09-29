/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Dinesh Thakur - Modified for waypoint navigation */

#pragma once

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <rviz/tool.h>
#include <visualization_msgs/InteractiveMarker.h>

#include "waypoint_nav_frame.h"

namespace Ogre {
class SceneNode;
class Vector3;
}  // namespace Ogre

namespace rviz {
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
class PanelDockWidget;
}  // namespace rviz

namespace kr_rviz_plugins {

class WaypointNavTool : public rviz::Tool {
  Q_OBJECT
 public:
  WaypointNavTool();
  ~WaypointNavTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;
  void makeIm(const Ogre::Vector3& position, const Ogre::Quaternion& quat,
              bool full_dof = false);

 private:
  void processFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void getMarkerPoses();
  void clearAllWaypoints();

  Ogre::SceneNode* moving_flag_node_;
  std::string flag_resource_;

  // the waypoint nav Qt frame
  WaypointFrame* frame_;
  rviz::PanelDockWidget* frame_dock_;

  interactive_markers::InteractiveMarkerServer server_;
  interactive_markers::MenuHandler menu_handler_;

  // map that stores waypoints based on unique names
  typedef std::map<int, Ogre::SceneNode*> M_StringToSNPtr;
  M_StringToSNPtr sn_map_;

  // index used for creating unique marker names
  int unique_ind_;
};

}  // end namespace kr_rviz_plugins
