#pragma once

#include <iostream>

#include "mpl_basis/data_type.h"

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
  MapUtil() {}
  /// Get map data
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
  template <int U = Dim>
  typename std::enable_if<U == 3, vec_Vec3f>::type getCloud() {
    vec_Vecf<Dim> cloud;
    Veci<Dim> n;
    for (n(0) = 0; n(0) < dim_(0); n(0)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        for (n(2) = 0; n(2) < dim_(2); n(2)++) {
          if (isOccupied(getIndex(n))) cloud.push_back(intToFloat(n));
        }
      }
    }
    return cloud;
  }
  /// Get occupied voxels for 2D
  template <int U = Dim>
  typename std::enable_if<U == 2, vec_Vec2f>::type getCloud() {
    vec_Vecf<Dim> cloud;
    Veci<Dim> n;

    for (n(0) = 0; n(0) < dim_(0); n(0)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        if (isOccupied(getIndex(n))) cloud.push_back(intToFloat(n));
      }
    }
    return cloud;
  }

  /// Get free voxels for 3D
  template <int U = Dim>
  typename std::enable_if<U == 3, vec_Vec3f>::type getFreeCloud() {
    vec_Vecf<Dim> cloud;
    Veci<Dim> n;
    for (n(0) = 0; n(0) < dim_(0); n(0)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        for (n(2) = 0; n(2) < dim_(2); n(2)++) {
          if (isFree(getIndex(n))) cloud.push_back(intToFloat(n));
        }
      }
    }
    return cloud;
  }

  /// Get free voxels for 2D
  template <int U = Dim>
  typename std::enable_if<U == 2, vec_Vec2f>::type getFreeCloud() {
    vec_Vecf<Dim> cloud;
    Veci<Dim> n;
    for (n(0) = 0; n(0) < dim_(0); n(0)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        if (isFree(getIndex(n))) cloud.push_back(intToFloat(n));
      }
    }
    return cloud;
  }

  /// Get unknown voxels for 3D
  template <int U = Dim>
  typename std::enable_if<U == 3, vec_Vec3f>::type getUnknownCloud() {
    vec_Vecf<Dim> cloud;
    Veci<Dim> n;
    for (n(0) = 0; n(0) < dim_(0); n(0)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        for (n(2) = 0; n(2) < dim_(2); n(2)++) {
          if (isUnknown(getIndex(n))) cloud.push_back(intToFloat(n));
        }
      }
    }
    return cloud;
  }

  /// Get unknown voxels for 2D
  template <int U = Dim>
  typename std::enable_if<U == 2, vec_Vec2f>::type getUnknownCloud() {
    vec_Vecf<Dim> cloud;
    Veci<Dim> n;
    for (n(0) = 0; n(0) < dim_(0); n(0)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        if (isUnknown(getIndex(n))) cloud.push_back(intToFloat(n));
      }
    }
    return cloud;
  }

  /// Dilate occupied cells
  template <int U = Dim>
  typename std::enable_if<U == 3>::type dilate(
      const vec_Veci<Dim> &dilate_neighbor) {
    Tmap map = map_;
    Veci<Dim> n = Veci<Dim>::Zero();
    for (n(0) = 0; n(0) < dim_(0); n(0)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        for (n(2) = 0; n(2) < dim_(2); n(2)++) {
          if (isOccupied(getIndex(n))) {
            for (const auto &it : dilate_neighbor) {
              if (!isOutside(n + it)) map[getIndex(n + it)] = val_occ;
            }
          }
        }
      }
    }
    map_ = map;
  }

  template <int U = Dim>
  typename std::enable_if<U == 2>::type dilate(
      const vec_Veci<Dim> &dilate_neighbor) {
    Tmap map = map_;
    Veci<Dim> n = Veci<Dim>::Zero();
    for (n(0) = 0; n(0) < dim_(0); n(0)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        if (isOccupied(getIndex(n))) {
          for (const auto &it : dilate_neighbor) {
            if (!isOutside(n + it)) map[getIndex(n + it)] = val_occ;
          }
        }
      }
    }

    map_ = map;
  }

  /// Free unknown voxels
  void freeUnknown();

  /// Free all voxels
  void freeAll();

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
  int8_t val_occ = 100;
  /// Assume free cell has value 0
  int8_t val_free = 0;
  /// Assume unknown cell has value -1
  int8_t val_unknown = -1;
};

typedef MapUtil<2> OccMapUtil;

typedef MapUtil<3> VoxelMapUtil;

}  // namespace MPL
