/**
 * @file map_util.h
 * @brief MapUtil classes
 */
#pragma once

#include "jps/data_type.h"
#include <kr_planning_msgs/VoxelMap.h>

namespace JPS {

/// The type of map data Tmap is defined as a 1D array
using Tmap = std::vector<signed char>;
/**
 * @biref The map util class for collision checking
 * @param Dim is the dimension of the workspace
 */
template <int Dim>
class MapUtil {
 public:
  /// Simple constructor
  MapUtil() {}
  /// Get map data
  Tmap getMap() { return map_; }

  /// Get resolution
  decimal_t getRes() { return res_; }

  /// Get dimensions
  Veci<Dim> getDim() { return dim_; }

  /// Get origin
  Vecf<Dim> getOrigin() { return origin_d_; }

  /// Get index of a cell
  int getIndex(const Veci<Dim> &pn);

  /// Check if the given cell is outside of the map in i-the dimension
  bool isOutsideXYZ(const Veci<Dim> &n, int i) {
    return n(i) < 0 || n(i) >= dim_(i);
  }
  /// Check if the cell is free by index
  bool isFree(int idx) { return map_[idx] == val_free; }
  /// Check if the cell is unknown by index
  bool isUnknown(int idx) { return map_[idx] == val_unknown; }
  /// Check if the cell is occupied by index
  bool isOccupied(int idx) { return map_[idx] > val_free; }

  /// Check if the cell is outside by coordinate
  bool isOutside(const Veci<Dim> &pn) {
    for (int i = 0; i < Dim; i++)
      if (pn(i) < 0 || pn(i) >= dim_(i)) return true;
    return false;
  }
  /// Check if the given cell is free by coordinate
  bool isFree(const Veci<Dim> &pn) {
    if (isOutside(pn))
      return false;
    else
      return isFree(getIndex(pn));
  }
  /// Check if the given cell is occupied by coordinate
  bool isOccupied(const Veci<Dim> &pn) {
    if (isOutside(pn))
      return false;
    else
      return isOccupied(getIndex(pn));
  }
  /// Check if the given cell is unknown by coordinate
  bool isUnknown(const Veci<Dim> &pn) {
    if (isOutside(pn)) return false;
    return map_[getIndex(pn)] == val_unknown;
  }

  /**
   * @brief Set map
   *
   * @param ori origin position
   * @param dim number of cells in each dimension
   * @param map array of cell values
   * @param res map resolution
   */
  void setMap(const Vecf<Dim> &ori, const Veci<Dim> &dim, const Tmap &map,
              decimal_t res);

  /// Print basic information about the util
  void info() const;

  /// Float position to discrete cell coordinate
  Veci<Dim> floatToInt(const Vecf<Dim> &pt);

  /// Discrete cell coordinate to float position
  Vecf<Dim> intToFloat(const Veci<Dim> &pn);

  /// Raytrace from float point pt1 to pt2
  vec_Veci<Dim> rayTrace(const Vecf<Dim> &pt1, const Vecf<Dim> &pt2);

  /// Check if the ray from p1 to p2 is occluded
  bool isBlocked(const Vecf<Dim> &p1, const Vecf<Dim> &p2, int8_t val = 100);

  /// Map entity
  Tmap map_;

  kr_planning_msgs::VoxelMap voxel_map;

 protected:
  /// Resolution
  decimal_t res_;
  /// Origin, float type
  Vecf<Dim> origin_d_;
  /// Dimension, int type
  Veci<Dim> dim_;
  /// Assume occupied cell has value 100
  // Now replaced with parameter value from VoxelMap.msg
  int8_t val_occ = voxel_map.val_occ;
  /// Assume free cell has value 0
  // Now replaced with parameter value from VoxelMap.msg
  int8_t val_free = voxel_map.val_free;
  /// Assume unknown cell has value -1
  // Now replaced with parameter value from VoxelMap.msg
  int8_t val_unknown = voxel_map.val_unknown;
};

typedef MapUtil<2> OccMapUtil;

typedef MapUtil<3> VoxelMapUtil;

}  // namespace JPS
