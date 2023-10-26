#pragma once

#include "mpl_basis/data_type.h"
#include <kr_planning_msgs/VoxelMap.h>
#include <vector>

namespace MPL {

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
  MapUtil() = default;
  /// Get map data
  /// TODO: this returns a copy, maybe change it to const&?
  Tmap getMap() { return map_; }
  /// Get resolution
  decimal_t getRes() { return res_; }
  /// Get dimensions
  Veci<Dim> getDim() { return dim_; }
  /// Get origin
  Vecf<Dim> getOrigin() { return origin_d_; }

  /// Get index of a cell for 2D
  int getIndex(const Veci<Dim> &pn);

  /// Check if the cell is free by index
  bool isFree(int idx) { return map_[idx] < val_occ && map_[idx] >= val_free; }
  /// Check if the cell is unknown by index
  bool isUnknown(int idx) { return map_[idx] == val_unknown; }
  /// Check if the cell is occupied by index
  bool isOccupied(int idx) { return map_[idx] == val_occ; }

  /// Check if the cell is outside by coordinate
  bool isOutside(const Veci<Dim> &pn);
  /// Check if the given cell is free by coordinate
  bool isFree(const Veci<Dim> &pn);
  /// Check if the given cell is occupied by coordinate
  bool isOccupied(const Veci<Dim> &pn);
  /// Check if the given cell is unknown by coordinate
  bool isUnknown(const Veci<Dim> &pn);

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
  void info();

  /// Float position to discrete cell coordinate
  Veci<Dim> floatToInt(const Vecf<Dim> &pt);
  /// Discrete cell coordinate to float position
  Vecf<Dim> intToFloat(const Veci<Dim> &pn);

  /// Raytrace from float point pt1 to pt2
  vec_Veci<Dim> rayTrace(const Vecf<Dim> &pt1, const Vecf<Dim> &pt2);

  /// Get occupied voxels for 3D
  vec_Vecf<Dim> getCloud();

  /// Get free voxels for 3D
  vec_Vecf<Dim> getFreeCloud();

  /// Get unknown voxels for 3D
  vec_Vecf<Dim> getUnknownCloud();

  kr_planning_msgs::VoxelMap voxel_map;

 protected:
  /// Map resolution
  decimal_t res_;
  /// Origin, float type
  Vecf<Dim> origin_d_;
  /// Dimension, int type
  Veci<Dim> dim_;
  /// Map entity
  Tmap map_;

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

template <int Dim>
inline int MapUtil<Dim>::getIndex(const Veci<Dim> &pn) {
  if constexpr (Dim == 3) {
    return pn(0) + dim_(0) * pn(1) + dim_(0) * dim_(1) * pn(2);
  } else {
    return pn(0) + dim_(0) * pn(1);
  }
}

template <int Dim>
inline bool MapUtil<Dim>::isOutside(const Veci<Dim> &pn) {
  if constexpr (Dim == 3) {
    return (pn(0) < 0 || pn(0) >= dim_(0) ||
            pn(1) < 0 || pn(1) >= dim_(1) ||
            pn(2) < 0 || pn(2) >= dim_(2));
  } else {
    return (pn(0) < 0 || pn(0) >= dim_(0) ||
            pn(1) < 0 || pn(1) >= dim_(1));
  }
}

template <int Dim>
inline bool MapUtil<Dim>::isFree(const Veci<Dim> &pn) {
  // First check if the voxel is within the bounds of the map and if it isn't,
  // return false. This is accomplished by taking advantage of short-circuit
  // evaluation. An if-else statement could also be used here but it isn't
  // to avoid issues with branch predictions and inline.
  return (!isOutside(pn) && isFree(getIndex(pn)));
}

template <int Dim>
inline bool MapUtil<Dim>::isOccupied(const Veci<Dim> &pn) {
  // First check if the voxel is within the bounds of the map and if it isn't,
  // return false. This is accomplished by taking advantage of short-circuit
  // evaluation. An if-else statement could also be used here but it isn't
  // to avoid issues with branch predictions and inline.
  return (!isOutside(pn) && isOccupied(getIndex(pn)));
}

template <int Dim>
inline bool MapUtil<Dim>::isUnknown(const Veci<Dim> &pn) {
  // First check if the voxel is within the bounds of the map and if it isn't,
  // return false. This is accomplished by taking advantage of short-circuit
  // evaluation. An if-else statement could also be used here but it isn't
  // to avoid issues with branch predictions and inline.
  return (!isOutside(pn) && isUnknown(getIndex(pn)));
}

}  // namespace MPL
