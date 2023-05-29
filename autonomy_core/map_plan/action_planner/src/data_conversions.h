#pragma once

#include <jps/map_util.h>
#include <kr_planning_msgs/VoxelMap.h>
#include <mpl_collision/map_util.h>
#include <memory>


void setMap(const std::shared_ptr<MPL::VoxelMapUtil>& map_util,
            const kr_planning_msgs::VoxelMap& msg);

void getMap(const std::shared_ptr<MPL::VoxelMapUtil>& map_util,
            kr_planning_msgs::VoxelMap* map);

void setMap(const std::shared_ptr<JPS::VoxelMapUtil>& map_util,
            const kr_planning_msgs::VoxelMap& msg);

void getMap(const std::shared_ptr<JPS::VoxelMapUtil>& map_util,
            kr_planning_msgs::VoxelMap* map);

void setMap(const std::shared_ptr<JPS::OccMapUtil>& map_util,
            const kr_planning_msgs::VoxelMap& msg);

void getMap(const std::shared_ptr<JPS::OccMapUtil>& map_util,
            kr_planning_msgs::VoxelMap* map);

// NOTE: This function is the same as getInflatedOccMap function in
// voxel_mapper.cpp, should merge them.
kr_planning_msgs::VoxelMap sliceMap(const kr_planning_msgs::VoxelMap& map,
                                    double h,
                                    double hh = 0);
