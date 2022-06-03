#include <data_conversions.h>

#include <algorithm>

void setMap(const std::shared_ptr<MPL::VoxelMapUtil>& map_util,
            const kr_planning_msgs::VoxelMap& msg) {
  Vec3f ori(msg.origin.x, msg.origin.y, msg.origin.z);
  Vec3i dim(msg.dim.x, msg.dim.y, msg.dim.z);
  map_util->setMap(ori, dim, msg.data, msg.resolution);
}

void getMap(const std::shared_ptr<MPL::VoxelMapUtil>& map_util,
            kr_planning_msgs::VoxelMap* map) {
  Vec3f ori = map_util->getOrigin();
  Vec3i dim = map_util->getDim();
  double res = map_util->getRes();

  map->origin.x = ori(0);
  map->origin.y = ori(1);
  map->origin.z = ori(2);

  map->dim.x = dim(0);
  map->dim.y = dim(1);
  map->dim.z = dim(2);
  map->resolution = res;

  map->data = map_util->getMap();
}

void setMap(const std::shared_ptr<JPS::VoxelMapUtil>& map_util,
            const kr_planning_msgs::VoxelMap& msg) {
  Vec3f ori(msg.origin.x, msg.origin.y, msg.origin.z);
  Vec3i dim(msg.dim.x, msg.dim.y, msg.dim.z);
  map_util->setMap(ori, dim, msg.data, msg.resolution);
}

void getMap(const std::shared_ptr<JPS::VoxelMapUtil>& map_util,
            kr_planning_msgs::VoxelMap* map) {
  Vec3f ori = map_util->getOrigin();
  Vec3i dim = map_util->getDim();
  double res = map_util->getRes();

  map->origin.x = ori(0);
  map->origin.y = ori(1);
  map->origin.z = ori(2);

  map->dim.x = dim(0);
  map->dim.y = dim(1);
  map->dim.z = dim(2);
  map->resolution = res;

  map->data = map_util->getMap();
}

void setMap(const std::shared_ptr<JPS::OccMapUtil>& map_util,
            const kr_planning_msgs::VoxelMap& msg) {
  Vec2f ori(msg.origin.x, msg.origin.y);
  Vec2i dim(msg.dim.x, msg.dim.y);
  map_util->setMap(ori, dim, msg.data, msg.resolution);
}

void getMap(const std::shared_ptr<JPS::OccMapUtil>& map_util,
            kr_planning_msgs::VoxelMap* map) {
  Vec2f ori = map_util->getOrigin();
  Vec2i dim = map_util->getDim();
  double res = map_util->getRes();

  map->origin.x = ori(0);
  map->origin.y = ori(1);
  map->origin.z = 0;

  map->dim.x = dim(0);
  map->dim.y = dim(1);
  map->dim.z = 1;
  map->resolution = res;

  map->data = map_util->getMap();
}

kr_planning_msgs::VoxelMap sliceMap(const kr_planning_msgs::VoxelMap& map,
                                    double h,
                                    double hh) {
  // slice a 3D voxel map
  double res = map.resolution;
  int hhi = hh / res;
  int h_min = (h - map.origin.z) / res - hhi;
  h_min = h_min >= 0 ? h_min : 0;
  h_min = h_min < map.dim.z ? h_min : map.dim.z - 1;
  int h_max = (h - map.origin.z) / res + hhi + 1;
  h_max = h_max > 0 ? h_max : 1;
  h_max = h_max <= map.dim.z ? h_max : map.dim.z;

  // slice a 3D voxel map
  kr_planning_msgs::VoxelMap voxel_map;
  voxel_map.origin.x = map.origin.x;
  voxel_map.origin.y = map.origin.y;
  voxel_map.origin.z = h;
  voxel_map.dim.x = map.dim.x;
  voxel_map.dim.y = map.dim.y;
  voxel_map.dim.z = 1;
  voxel_map.resolution = map.resolution;

  voxel_map.data.resize(map.dim.x * map.dim.y, -1);

  for (int nx = 0; nx < map.dim.x; nx++) {
    for (int ny = 0; ny < map.dim.y; ny++) {
      for (int hi = h_min; hi < h_max; hi++) {
        int map_idx = nx + map.dim.x * ny + map.dim.x * map.dim.y * hi;
        int idx = nx + map.dim.x * ny;
        voxel_map.data[idx] = std::max(voxel_map.data[idx], map.data[map_idx]);
      }
    }
  }

  return voxel_map;
}
