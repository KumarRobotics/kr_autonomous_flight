/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Brian Gerkey */

#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>

#include <fstream>

#include "image_loader.hpp"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "create_map", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");
  ros::Publisher map_pub =
      nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);

  std::string mapfname;
  double origin[3];
  int negate;
  double occ_th, free_th;
  double res;
  MapMode mode = TRINARY;
  std::string frame_id;

  nh.param("resolution", res, 0.1);
  nh.param("frame_id", frame_id, std::string("map"));
  nh.param("negate", negate, 0);
  nh.param("occ_th", occ_th, 0.65);
  nh.param("free_th", free_th, 0.2);

  nh.param("origin_x", origin[0], 0.0);
  nh.param("origin_y", origin[1], 0.0);
  nh.param("origin_z", origin[2], 0.0);

  nh.param("file", mapfname, std::string(""));

  ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
  planning_ros_msgs::VoxelMap map_resp;
  try {
    map_server::loadMapFromFile(map_resp, mapfname.c_str(), res, negate, occ_th,
                                free_th, origin, mode);
  } catch (std::runtime_error e) {
    ROS_ERROR("%s", e.what());
    exit(-1);
  }
  // To make sure get a consistent time in simulation
  map_resp.header.frame_id = frame_id;
  map_resp.header.stamp = ros::Time::now();
  ROS_INFO("Read a %.1f X %.1f map @ %.3lf m/cell", map_resp.dim.x,
           map_resp.dim.y, map_resp.resolution);

  // Latched publisher for data
  map_pub.publish(map_resp);

  ros::spin();

  return 0;
}
