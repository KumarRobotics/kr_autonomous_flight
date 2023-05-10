#include "mapper/voxel_mapper.h"

#include <ros/ros.h>

namespace mapper {

VoxelMapper::VoxelMapper(const Eigen::Vector3d& origin,
                         const Eigen::Vector3d& dim,
                         double res,
                         int8_t default_val,
                         int decay_times_to_empty) {
  origin_d_ = Eigen::Vector3d::Zero();
  dim_ = Eigen::Vector3i::Zero();
  res_ = res;
  val_default_ = default_val;

  allocate(origin, dim);
  if (decay_times_to_empty >= 1) {
    val_decay_ =
        std::ceil(static_cast<float>(val_occ_ - val_even_)
        / static_cast<float>(decay_times_to_empty));
  } else {
    val_decay_ = 0;  // no decay
  }

  int val_temp = val_occ_ + val_add_;
  if (val_temp > 128) {
    ROS_ERROR_STREAM("val_occ + val_add is larger than 128, the value is: "
                     << val_temp
                     << ", this will cause overflow!!! "
                        "Reducing it to < 128 now...");
    val_occ_ = 100;
    val_add_ = 20;
  }
}

void VoxelMapper::setMapUnknown() {
  val_default_ = val_unknown_;

  std::vector<signed char> base_map(dim_[0] * dim_[1] * dim_[2], val_default_);
  map_.setMap(origin_d_, dim_, base_map, res_);
  inflated_map_.setMap(origin_d_, dim_, base_map, res_);
}

void VoxelMapper::setMapFree() {
  val_default_ = val_free_;

  std::vector<signed char> base_map(dim_[0] * dim_[1] * dim_[2], val_default_);
  map_.setMap(origin_d_, dim_, base_map, res_);
  inflated_map_.setMap(origin_d_, dim_, base_map, res_);
}

kr_planning_msgs::VoxelMap VoxelMapper::getMap() {
  kr_planning_msgs::VoxelMap voxel_map;
  voxel_map.origin.x = origin_d_(0);
  voxel_map.origin.y = origin_d_(1);
  voxel_map.origin.z = origin_d_(2);
  voxel_map.dim.x = dim_(0);
  voxel_map.dim.y = dim_(1);
  voxel_map.dim.z = dim_(2);

  voxel_map.resolution = res_;

  int size = dim_(0) * dim_(1) * dim_(2);
  voxel_map.data.resize(size, val_default_);

  for (int i = 0; i < size; i++) {
    const auto &voxel = map_[i];
    if (voxel > val_even_) {
      voxel_map.data[i] = val_occ_;
    } else if (voxel >= val_free_) {
      voxel_map.data[i] = val_free_;
    }
  }

  return voxel_map;
}

kr_planning_msgs::VoxelMap VoxelMapper::getInflatedMap() {
  kr_planning_msgs::VoxelMap voxel_map;
  voxel_map.origin.x = origin_d_(0);
  voxel_map.origin.y = origin_d_(1);
  voxel_map.origin.z = origin_d_(2);
  voxel_map.dim.x = dim_(0);
  voxel_map.dim.y = dim_(1);
  voxel_map.dim.z = dim_(2);

  voxel_map.resolution = res_;

  int size = dim_(0) * dim_(1) * dim_(2);
  voxel_map.data.resize(size, val_default_);

  for (int i = 0; i < size; i++) {
    const auto &voxel = inflated_map_[i];
    if (voxel > val_even_) {
      voxel_map.data[i] = val_occ_;
    } else if (voxel >= val_free_) {
      voxel_map.data[i] = val_free_;
    }
  }

  return voxel_map;
}

kr_planning_msgs::VoxelMap VoxelMapper::getInflatedLocalMap(
    const Eigen::Vector3d& ori_d, const Eigen::Vector3d& dim_d) {
  kr_planning_msgs::VoxelMap voxel_map;

  voxel_map.resolution = res_;
  voxel_map.origin.x = ori_d(0);
  voxel_map.origin.y = ori_d(1);
  voxel_map.origin.z = ori_d(2);

  // Calculated dimesion of local voxel map in voxels
  Eigen::Vector3i dim(dim_d(0) / res_, dim_d(1) / res_, dim_d(2) / res_);
  voxel_map.dim.x = dim(0);
  voxel_map.dim.y = dim(1);
  voxel_map.dim.z = dim(2);

  voxel_map.data.resize(dim(0) * dim(1) * dim(2), val_default_);

  // Offset between the local map and the global map (in voxels)
  Eigen::Vector3i offset = map_.floatToInt(ori_d);

  // The voxel in the global map with index global_vox corresponds to the
  // voxel in the local map with index local_vox
  Eigen::Vector3i global_vox;
  Eigen::Vector3i local_vox;

  for (local_vox(2) = 0; local_vox(2) < dim(2); local_vox(2)++) {
    for (local_vox(1) = 0; local_vox(1) < dim(1); local_vox(1)++) {
      for (local_vox(0) = 0; local_vox(0) < dim(0); local_vox(0)++) {
        global_vox = local_vox + offset;
        int local_idx = local_vox(0)
                        + dim(0) * local_vox(1)
                        + dim(0) * dim(1) * local_vox(2);
        // Check if inside the global map, outside portion will be regarded as
        // occupied for safety
        if ((global_vox(0) >= 0) && (global_vox(0) < dim_(0)) &&
            (global_vox(1) >= 0) && (global_vox(1) < dim_(1)) &&
            (global_vox(2) >= 0) && (global_vox(2) < dim_(2))) {
          int global_idx = inflated_map_.getIndex(global_vox);
          if (inflated_map_[global_idx] > val_even_) {
            voxel_map.data[local_idx] = val_occ_;
          } else if (inflated_map_[global_idx] >= val_free_) {
            voxel_map.data[local_idx] = val_free_;
          }
        } else {
          // Outside global map portion will be regarded as occupied for safety
          voxel_map.data[local_idx] = val_occ_;
        }
      }
    }
  }
  return voxel_map;
}

// TODO(xu): This function is the same as sliceMap function in
// data_conversions.cpp, should merge them.
kr_planning_msgs::VoxelMap VoxelMapper::getInflatedOccMap(double h,
                                                           double hh) {
  kr_planning_msgs::VoxelMap voxel_map;
  voxel_map.origin.x = origin_d_(0);
  voxel_map.origin.y = origin_d_(1);
  voxel_map.origin.z = 0;
  voxel_map.dim.x = dim_(0);
  voxel_map.dim.y = dim_(1);
  voxel_map.dim.z = 1;

  voxel_map.resolution = res_;

  voxel_map.data.resize(dim_(0) * dim_(1), val_free_);

  // discretize the thickness into number of voxels
  int hi = hh / res_;
  // calculate the starting z-axis value for the slice
  int h_min = (h - origin_d_(2)) / res_ - hi;
  h_min = h_min >= 0 ? h_min : 0;
  h_min = h_min < dim_(2) ? h_min : dim_(2) - 1;
  int h_max = (h - origin_d_(2)) / res_ + hi + 1;
  h_max = h_max > 0 ? h_max : 1;
  h_max = h_max <= dim_(2) ? h_max : dim_(2);

  Eigen::Vector3i n;
  for (n(2) = h_min; n(2) < h_max; n(2)++) {
    for (n(1) = 0; n(1) < dim_(1); n(1)++) {
      for (n(0) = 0; n(0) < dim_(0); n(0)++) {
        int voxel_idx = inflated_map_.getIndex(n);
        if (inflated_map_[voxel_idx] > val_even_)
          voxel_map.data[n(0) + dim_(0) * n(1)] = val_occ_;
      }
    }
  }

  return voxel_map;
}

void VoxelMapper::addCloud(const vec_Vec3d& pts,
                           const Eigen::Affine3d& TF,
                           const vec_Vec3i& ns,
                           bool ray_trace,
                           double max_range) {
  const Eigen::Vector3d pos(
      TF.translation().x(), TF.translation().y(), TF.translation().z());

  // Decay cloud which is within a local region around the robot
  if (val_decay_ > 0) {
    double max_decay_range = max_range * 2.0;
    decayLocalCloud(pos, max_decay_range);
  }

  for (const auto& it : pts) {
    // Throw away points outside max_range first to save computation
    if ((max_range > 0) && (it.norm() > max_range)) continue;

    // Transform points from lidar frame to global frame
    const Eigen::Vector3d pt = TF * it;
    const Eigen::Vector3i n = map_.floatToInt(pt);

    // Throw away points outside voxel box to save computation.
    // TODO(xu): if unknown vs known matters (i.e. planning algorithm
    // differentiates unknown and free), need to move this after ray_trace.
    // Won't add much computation according to timer feedback.
    if (map_.isOutside(n)) continue;

    // for each point do ray trace
    if (ray_trace) {
      vec_Vec3i rays = map_.rayTrace(pos, pt);
      for (const auto& pn : rays) {
        int idx = map_.getIndex(pn);
        if (map_[idx] == val_unknown_) {
          map_[idx] = val_free_;
          if (inflated_map_[idx] == val_unknown_)
            inflated_map_[idx] = val_free_;
        }
      }
    }

    // Add val_add to the voxel whenever a point lies in it. The voxel will be
    // occupied after N*T > (val_occ - val_even) / val_add, where N is the
    // number of points and T is number of scans.
    int vox_idx = map_.getIndex(n);

    if (map_[vox_idx] < val_occ_) {
      map_[vox_idx] += val_add_;

      // Do the same to voxels in the inflation region
      if (inflated_map_[vox_idx] < val_occ_) {
        inflated_map_[vox_idx] += val_add_;
      }

      // ns is a vector of values from -inflation_range to +inflation_range
      // excluding 0
      for (const auto& it_n : ns) {
        Eigen::Vector3i neighbor_vox = n + it_n;
        int neighbor_vox_idx = inflated_map_.getIndex(neighbor_vox);

        if (!inflated_map_.isOutside(neighbor_vox)
            && inflated_map_[neighbor_vox_idx] < val_occ_) {
          inflated_map_[neighbor_vox_idx] += val_add_;
        }
      }
    }
  }
}

vec_Vec3d VoxelMapper::getCloud() {
  vec_Vec3d pts;
  Eigen::Vector3i n;
  for (n(2) = 0; n(2) < dim_(2); n(2)++) {
    for (n(1) = 0; n(1) < dim_(1); n(1)++) {
      for (n(0) = 0; n(0) < dim_(0); n(0)++) {
        int vox_idx = map_.getIndex(n);
        if (map_[vox_idx] > val_even_) pts.push_back(map_.intToFloat(n));
      }
    }
  }
  return pts;
}

// GUI: NOT USED
vec_Vec3d VoxelMapper::getInflatedCloud() {
  vec_Vec3d pts;
  Eigen::Vector3i n;
  for (n(2) = 0; n(2) < dim_(2); n(2)++) {
    for (n(1) = 0; n(1) < dim_(1); n(1)++) {
      for (n(0) = 0; n(0) < dim_(0); n(0)++) {
        int vox_idx = inflated_map_.getIndex(n);
        if (inflated_map_[vox_idx] > val_even_)
          pts.push_back(inflated_map_.intToFloat(n));
      }
    }
  }
  return pts;
}

vec_Vec3d VoxelMapper::getLocalCloud(const Eigen::Vector3d& ori,
                                     const Eigen::Vector3d& dim) {
  Eigen::Vector3i dim_low, dim_up;

  Eigen::Vector3i dim1 = map_.floatToInt(ori);
  for (int i = 0; i < 3; i++) dim_low(i) = dim1(i) < 0 ? 0 : dim1(i);

  Eigen::Vector3i dim2 = map_.floatToInt(ori + dim);
  for (int i = 0; i < 3; i++)
    dim_up(i) = dim2(i) >= dim_(i) ? dim_(i)-1 : dim2(i);

  vec_Vec3d pts;
  Eigen::Vector3i n;
  for (n(2) = dim_low(2); n(2) <= dim_up(2); n(2)++) {
    for (n(1) = dim_low(1); n(1) <= dim_up(1); n(1)++) {
      for (n(0) = dim_low(0); n(0) <= dim_up(0); n(0)++) {
        int vox_idx = map_.getIndex(n);
        if (map_[vox_idx] > val_even_) pts.push_back(map_.intToFloat(n));
      }
    }
  }
  return pts;
}

vec_Vec3d VoxelMapper::getInflatedLocalCloud(const Eigen::Vector3d& ori,
                                             const Eigen::Vector3d& dim) {
  Eigen::Vector3i dim_low, dim_up;

  Eigen::Vector3i dim1 = map_.floatToInt(ori);
  for (int i = 0; i < 3; i++) dim_low(i) = dim1(i) < 0 ? 0 : dim1(i);

  Eigen::Vector3i dim2 = map_.floatToInt(ori + dim);
  for (int i = 0; i < 3; i++)
    dim_up(i) = dim2(i) >= dim_(i) ? dim_(i)-1 : dim2(i);

  vec_Vec3d pts;
  Eigen::Vector3i n;
  for (n(2) = dim_low(2); n(2) <= dim_up(2); n(2)++) {
    for (n(1) = dim_low(1); n(1) <= dim_up(1); n(1)++) {
      for (n(0) = dim_low(0); n(0) <= dim_up(0); n(0)++) {
        int vox_idx = inflated_map_.getIndex(n);
        if (inflated_map_[vox_idx] > val_free_)
          pts.push_back(inflated_map_.intToFloat(n));
      }
    }
  }
  return pts;
}

void VoxelMapper::decayLocalCloud(const Eigen::Vector3d& pos,
                                  double max_decay_range) {
  Eigen::Vector3i dim_low, dim_up;
  Eigen::Vector3d start_pos;
  Eigen::Vector3d end_pos;

  start_pos(0) = pos(0) - max_decay_range;
  start_pos(1) = pos(1) - max_decay_range;
  start_pos(2) = pos(2) - max_decay_range;

  end_pos(0) = pos(0) + max_decay_range;
  end_pos(1) = pos(1) + max_decay_range;
  end_pos(2) = pos(2) + max_decay_range;

  Eigen::Vector3i dim1 = map_.floatToInt(start_pos);
  for (int i = 0; i < 3; i++) dim_low(i) = dim1(i) < 0 ? 0 : dim1(i);

  Eigen::Vector3i dim2 = map_.floatToInt(end_pos);
  for (int i = 0; i < 3; i++)
    dim_up(i) = dim2(i) >= dim_(i) ? dim_(i)-1 : dim2(i);

  // Decaying voxels within robot's local region (voxels will disappear if
  // unobserved for (val_occ - val_even) / val_decay times)
  Eigen::Vector3i n;
  for (n(2) = dim_low(2); n(2) <= dim_up(2); n(2)++) {
    for (n(1) = dim_low(1); n(1) <= dim_up(1); n(1)++) {
      for (n(0) = dim_low(0); n(0) <= dim_up(0); n(0)++) {
        int vox_idx = map_.getIndex(n);
        if (map_[vox_idx] > val_even_)
          map_[vox_idx] -= val_decay_;
        if (inflated_map_[vox_idx] > val_even_)
          inflated_map_[vox_idx] -= val_decay_;
      }
    }
  }
}

void VoxelMapper::freeVoxels(const Eigen::Vector3d& pt, const vec_Vec3i& ns) {
  const Eigen::Vector3i pn = map_.floatToInt(pt);
  int vox_idx = map_.getIndex(pn);

  if (!map_.isOutside(pn) && map_[vox_idx] != val_free_) {
    map_[vox_idx] = val_free_;
    if (inflated_map_[vox_idx] != val_free_)
      inflated_map_[vox_idx] = val_free_;
  }

  for (const auto& n : ns) {
    Eigen::Vector3i pnn = pn + n;
    int neighbor_vox_idx = map_.getIndex(pnn);
    if (!map_.isOutside(pnn) && map_[neighbor_vox_idx] != val_free_) {
      map_[neighbor_vox_idx] = val_free_;
      if (inflated_map_[neighbor_vox_idx] != val_free_)
        inflated_map_[neighbor_vox_idx] = val_free_;
    }
  }
}

void VoxelMapper::freeCloud(const vec_Vec3d& pts, const Eigen::Affine3d& tf) {
  const Eigen::Vector3d pos(
      tf.translation().x(), tf.translation().y(), tf.translation().z());

  for (const auto& it : pts) {
    const Eigen::Vector3d pt = tf * it;

    vec_Vec3i rays = map_.rayTrace(pos, pt);
    for (const auto& pn : rays) {
      int vox_idx = map_.getIndex(pn);
      if (map_[vox_idx] == val_unknown_) {
        map_[vox_idx] = val_free_;
        if (inflated_map_[vox_idx] == val_unknown_)
          inflated_map_[vox_idx] = val_free_;
      }
    }
  }
}

bool VoxelMapper::allocate(const Eigen::Vector3d& new_ori_d,
                           const Eigen::Vector3d& new_dim_d) {
  static Eigen::Vector3i prev_origin = Eigen::Vector3i::Zero();

  Eigen::Vector3i new_dim(new_dim_d(0) / res_,
                          new_dim_d(1) / res_,
                          new_dim_d(2) / res_);
  Eigen::Vector3i new_ori(new_ori_d(0) / res_,
                          new_ori_d(1) / res_,
                          new_ori_d(2) / res_);

  if (new_dim(2) == 0)  // 2d case, set the z dimension to be 1
    new_dim(2) = 1;

  // Check if new dimensions and origin are the same as before, in which case
  // do not perform reallocation
  if (new_dim == dim_ && new_ori == prev_origin) {
    return false;
  } else {
    int size = new_dim(0) * new_dim(1) * new_dim(2);
    std::vector<signed char> new_map(size, val_default_);

    // Iterate through all elements in new map
    Eigen::Vector3i new_vox;
    for (new_vox(0) = 0; new_vox(0) < new_dim(0); new_vox(0)++) {
      for (new_vox(1) = 0; new_vox(1) < new_dim(1); new_vox(1)++) {
        for (new_vox(2) = 0; new_vox(2) < new_dim(2); new_vox(2)++) {
          // Check if voxel in the new map corresponds to a voxel in the
          // previous map. If voxel is within the bounds of the previous map,
          // then copy the value of the corresponding voxel
          if (new_vox(0) + new_ori(0) >= prev_origin(0) &&
              new_vox(1) + new_ori(1) >= prev_origin(1) &&
              new_vox(2) + new_ori(2) >= prev_origin(2) &&
              new_vox(0) + new_ori(0) < prev_origin(0) + dim_(0) &&
              new_vox(1) + new_ori(1) < prev_origin(1) + dim_(1) &&
              new_vox(2) + new_ori(2) < prev_origin(2) + dim_(2)) {
            Eigen::Vector3i prev_vox = new_vox + new_ori - prev_origin;
            int prev_vox_idx = map_.getIndex(prev_vox);
            int new_vox_idx = new_vox(0) +
                              new_dim(0) * new_vox(1) +
                              new_dim(0) * new_dim(1) * new_vox(2);

            new_map[new_vox_idx] = map_[prev_vox_idx];
          }
        }
      }
    }

    origin_d_ = new_ori_d;
    dim_ = new_dim;
    prev_origin = new_ori;

    map_.setMap(origin_d_, dim_, new_map, res_);
    inflated_map_.setMap(origin_d_, dim_, new_map, res_);

    return true;
  }
}

}  // namespace mapper
