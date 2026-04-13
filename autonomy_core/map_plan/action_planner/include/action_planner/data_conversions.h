#pragma once

#include <jps/map_util.h>
#include <kr_planning_msgs/msg/voxel_map.hpp>
#include <mpl_collision/map_util.h>
#include <memory>


void setMap(const std::shared_ptr<MPL::VoxelMapUtil>& map_util,
            const kr_planning_msgs::msg::VoxelMap& msg);

void getMap(const std::shared_ptr<MPL::VoxelMapUtil>& map_util,
            kr_planning_msgs::msg::VoxelMap* map);

void setMap(const std::shared_ptr<JPS::VoxelMapUtil>& map_util,
            const kr_planning_msgs::msg::VoxelMap& msg);

void getMap(const std::shared_ptr<JPS::VoxelMapUtil>& map_util,
            kr_planning_msgs::msg::VoxelMap* map);

void setMap(const std::shared_ptr<JPS::OccMapUtil>& map_util,
            const kr_planning_msgs::msg::VoxelMap& msg);

void getMap(const std::shared_ptr<JPS::OccMapUtil>& map_util,
            kr_planning_msgs::msg::VoxelMap* map);

// NOTE: This function is the same as getInflatedOccMap function in
// voxel_mapper.cpp, should merge them.
kr_planning_msgs::msg::VoxelMap sliceMap(
    const kr_planning_msgs::msg::VoxelMap& map,
    double h,
    double hh = 0);
