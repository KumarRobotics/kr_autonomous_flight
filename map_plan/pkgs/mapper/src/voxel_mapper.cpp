#include <mapper/voxel_mapper.h>

VoxelMapper::VoxelMapper(Vec3f origin, Vec3f dim, decimal_t res, int8_t val) {
  origin_ = Vec3i::Zero();
  origin_d_ = Vec3f::Zero();
  dim_ = Vec3i::Zero();
  lidar_rot_ = Aff3f::Identity();

  res_ = res;
  val_default = val;
  allocate(dim, origin);
}

void VoxelMapper::setMapUnknown() {
  val_default = val_unknown;
  std::fill(map_.data(), map_.data() + map_.num_elements(), val_unknown);
  std::fill(inflated_map_.data(),
            inflated_map_.data() + inflated_map_.num_elements(), val_unknown);
}

void VoxelMapper::setMapFree() {
  val_default = val_free;
  std::fill(map_.data(), map_.data() + map_.num_elements(), val_free);
  std::fill(inflated_map_.data(),
            inflated_map_.data() + inflated_map_.num_elements(), val_free);
}

void VoxelMapper::freeVoxels(const Vec3f& pt, const vec_Vec3i ns) {
  const Vec3i pn = floatToInt(pt);

  if (!isOutSide(pn) && map_[pn(0)][pn(1)][pn(2)] != val_free) {
    map_[pn(0)][pn(1)][pn(2)] = val_free;
    if (inflated_map_[pn(0)][pn(1)][pn(2)] != val_free)
      inflated_map_[pn(0)][pn(1)][pn(2)] = val_free;
  }

  for (const auto& n : ns) {
    Vec3i pnn = pn + n;
    if (!isOutSide(pnn) && map_[pnn(0)][pnn(1)][pnn(2)] != val_free) {
      map_[pnn(0)][pnn(1)][pnn(2)] = val_free;
      if (inflated_map_[pnn(0)][pnn(1)][pnn(2)] != val_free)
        inflated_map_[pnn(0)][pnn(1)][pnn(2)] = val_free;
    }
  }
}

vec_Vec3f VoxelMapper::getCloud() {
  vec_Vec3f pts;
  Vec3i n;
  for (n(0) = 0; n(0) < dim_(0); n(0)++) {
    for (n(1) = 0; n(1) < dim_(1); n(1)++) {
      for (n(2) = 0; n(2) < dim_(2); n(2)++) {
        if (map_[n(0)][n(1)][n(2)] > val_even) pts.push_back(intToFloat(n));
      }
    }
  }
  return pts;
}

vec_Vec3f VoxelMapper::getInflatedCloud() {
  vec_Vec3f pts;
  Vec3i n;
  for (n(0) = 0; n(0) < dim_(0); n(0)++) {
    for (n(1) = 0; n(1) < dim_(1); n(1)++) {
      for (n(2) = 0; n(2) < dim_(2); n(2)++) {
        if (inflated_map_[n(0)][n(1)][n(2)] > val_even)
          pts.push_back(intToFloat(n));
      }
    }
  }
  return pts;
}

vec_Vec3f VoxelMapper::getLocalCloud(const Vec3f& pos, const Vec3f& ori,
                                     const Vec3f& dim) {
  Vec3i dim_low, dim_up;

  Vec3i dim1 = floatToInt(pos + ori);
  for (int i = 0; i < 3; i++) dim_low(i) = dim1(i) < 0 ? 0 : dim1(i);

  Vec3i dim2 = floatToInt(pos + ori + dim);
  for (int i = 0; i < 3; i++) dim_up(i) = dim2(i) > dim_(i) ? dim_(i) : dim2(i);

  vec_Vec3f pts;
  Vec3i n;
  for (n(0) = dim_low(0); n(0) < dim_up(0); n(0)++) {
    for (n(1) = dim_low(1); n(1) < dim_up(1); n(1)++) {
      for (n(2) = dim_low(2); n(2) < dim_up(2); n(2)++) {
        if (map_[n(0)][n(1)][n(2)] > val_even) pts.push_back(intToFloat(n));
      }
    }
  }
  return pts;
}

void VoxelMapper::decayLocalCloud(const Vec3f& pos, double max_decay_range) {
  Vec3i dim_low, dim_up;
  Vec3f start_pos;
  Vec3f end_pos;

  start_pos(0) = pos(0) - max_decay_range;
  start_pos(1) = pos(1) - max_decay_range;
  start_pos(2) = pos(2) - max_decay_range;

  end_pos(0) = pos(0) + max_decay_range;
  end_pos(1) = pos(1) + max_decay_range;
  end_pos(2) = pos(2) + max_decay_range;

  Vec3i dim1 = floatToInt(start_pos);
  for (int i = 0; i < 3; i++) dim_low(i) = dim1(i) < 0 ? 0 : dim1(i);

  Vec3i dim2 = floatToInt(end_pos);
  for (int i = 0; i < 3; i++) dim_up(i) = dim2(i) > dim_(i) ? dim_(i) : dim2(i);

  // Decaying voxels within robot's local region (voxels will disappear if
  // unobserved for (val_occ - val_even) / val_decay times)
  Vec3i n;
  for (n(0) = dim_low(0); n(0) < dim_up(0); n(0)++) {
    for (n(1) = dim_low(1); n(1) < dim_up(1); n(1)++) {
      for (n(2) = dim_low(2); n(2) < dim_up(2); n(2)++) {
        if (map_[n(0)][n(1)][n(2)] > val_even)
          map_[n(0)][n(1)][n(2)] = map_[n(0)][n(1)][n(2)] - val_decay;
        if (inflated_map_[n(0)][n(1)][n(2)] > val_even)
          inflated_map_[n(0)][n(1)][n(2)] =
              inflated_map_[n(0)][n(1)][n(2)] - val_decay;
      }
    }
  }
}

// crop a local voxel map from the global voxel map (local voxel map is a subset
// of global voxel map)
vec_Vec3f VoxelMapper::getInflatedLocalCloud(const Vec3f& pos, const Vec3f& ori,
                                             const Vec3f& dim) {
  Vec3i dim_low, dim_up;

  Vec3i dim1 = floatToInt(pos + ori);
  for (int i = 0; i < 3; i++) dim_low(i) = dim1(i) < 0 ? 0 : dim1(i);

  Vec3i dim2 = floatToInt(pos + ori + dim);
  for (int i = 0; i < 3; i++) dim_up(i) = dim2(i) > dim_(i) ? dim_(i) : dim2(i);

  vec_Vec3f pts;
  Vec3i n;
  for (n(0) = dim_low(0); n(0) < dim_up(0); n(0)++) {
    for (n(1) = dim_low(1); n(1) < dim_up(1); n(1)++) {
      for (n(2) = dim_low(2); n(2) < dim_up(2); n(2)++) {
        if (inflated_map_[n(0)][n(1)][n(2)] > val_free)
          pts.push_back(intToFloat(n));
      }
    }
  }
  return pts;
}

planning_ros_msgs::VoxelMap VoxelMapper::getMap() {
  planning_ros_msgs::VoxelMap voxel_map;
  voxel_map.origin.x = origin_d_(0);
  voxel_map.origin.y = origin_d_(1);
  voxel_map.origin.z = origin_d_(2);
  voxel_map.dim.x = dim_(0);
  voxel_map.dim.y = dim_(1);
  voxel_map.dim.z = dim_(2);

  voxel_map.resolution = res_;

  voxel_map.data.resize(dim_(0) * dim_(1) * dim_(2), val_default);
  Vec3i n;
  for (n(0) = 0; n(0) < dim_(0); n(0)++) {
    for (n(1) = 0; n(1) < dim_(1); n(1)++) {
      for (n(2) = 0; n(2) < dim_(2); n(2)++) {
        if (map_[n(0)][n(1)][n(2)] > val_even) {
          int idx = n(0) + dim_(0) * n(1) + dim_(0) * dim_(1) * n(2);
          voxel_map.data[idx] = val_occ;
        } else if (map_[n(0)][n(1)][n(2)] >= val_free) {
          int idx = n(0) + dim_(0) * n(1) + dim_(0) * dim_(1) * n(2);
          voxel_map.data[idx] = val_free;
        }
      }
    }
  }
  return voxel_map;
}

planning_ros_msgs::VoxelMap VoxelMapper::getInflatedMap() {
  planning_ros_msgs::VoxelMap voxel_map;
  voxel_map.origin.x = origin_d_(0);
  voxel_map.origin.y = origin_d_(1);
  voxel_map.origin.z = origin_d_(2);
  voxel_map.dim.x = dim_(0);
  voxel_map.dim.y = dim_(1);
  voxel_map.dim.z = dim_(2);

  voxel_map.resolution = res_;

  voxel_map.data.resize(dim_(0) * dim_(1) * dim_(2), val_default);
  Vec3i n;
  for (n(0) = 0; n(0) < dim_(0); n(0)++) {
    for (n(1) = 0; n(1) < dim_(1); n(1)++) {
      for (n(2) = 0; n(2) < dim_(2); n(2)++) {
        if (inflated_map_[n(0)][n(1)][n(2)] > val_even) {
          int idx = n(0) + dim_(0) * n(1) + dim_(0) * dim_(1) * n(2);
          voxel_map.data[idx] = val_occ;
        } else if (inflated_map_[n(0)][n(1)][n(2)] >= val_free) {
          int idx = n(0) + dim_(0) * n(1) + dim_(0) * dim_(1) * n(2);
          voxel_map.data[idx] = val_free;
        }
      }
    }
  }
  return voxel_map;
}

// crop a local voxel map from the global voxel map
planning_ros_msgs::VoxelMap VoxelMapper::getInflatedLocalMap(
    const Vec3f& ori_d, const Vec3f& dim_d) {
  planning_ros_msgs::VoxelMap voxel_map;

  voxel_map.resolution = res_;
  voxel_map.origin.x = ori_d(0);
  voxel_map.origin.y = ori_d(1);
  voxel_map.origin.z = ori_d(2);

  Vec3i dim(
      dim_d(0) / res_, dim_d(1) / res_,
      dim_d(2) / res_);  // calculated dimesion of local voxel map in voxels
  voxel_map.dim.x = dim(0);
  voxel_map.dim.y = dim(1);
  voxel_map.dim.z = dim(2);

  voxel_map.data.resize(dim(0) * dim(1) * dim(2), val_default);
  Vec3i n;

  Vec3i offset_n = floatToInt(
      ori_d);  // offset between the local map and the storage map (in voxels)
  Vec3i ori_map_idx;  // index of voxel in storage map (corresponding to voxel
                      // with index n in local map)

  for (n(0) = 0; n(0) < dim(0); n(0)++) {
    for (n(1) = 0; n(1) < dim(1); n(1)++) {
      for (n(2) = 0; n(2) < dim(2); n(2)++) {
        ori_map_idx = n + offset_n;
        // check if inside the storage map, outside portion will be regarded as
        // occupied for safety
        if ((ori_map_idx(0) >= 0) && (ori_map_idx(0) < dim_(0)) &&
            (ori_map_idx(1) >= 0) && (ori_map_idx(1) < dim_(1)) &&
            (ori_map_idx(2) >= 0) && (ori_map_idx(2) < dim_(2))) {
          if (inflated_map_[ori_map_idx(0)][ori_map_idx(1)][ori_map_idx(2)] >
              val_even) {
            int idx = n(0) + dim(0) * n(1) + dim(0) * dim(1) * n(2);
            voxel_map.data[idx] = val_occ;
          } else if (inflated_map_[ori_map_idx(0)][ori_map_idx(1)]
                                  [ori_map_idx(2)] >= val_free) {
            int idx = n(0) + dim(0) * n(1) + dim(0) * dim(1) * n(2);
            voxel_map.data[idx] = val_free;
          }
        } else {
          int idx = n(0) + dim(0) * n(1) + dim(0) * dim(1) * n(2);
          voxel_map.data[idx] = val_occ;  // outside storage map portion will be
                                          // regarded as occupied for safety
        }
      }
    }
  }
  return voxel_map;
}

// TODO: This function is the same as sliceMap function in data_conversions.cpp,
// should merge them.
planning_ros_msgs::VoxelMap VoxelMapper::getInflatedOccMap(double h,
                                                           double hh) {
  planning_ros_msgs::VoxelMap voxel_map;
  voxel_map.origin.x = origin_d_(0);
  voxel_map.origin.y = origin_d_(1);
  voxel_map.origin.z = 0;
  voxel_map.dim.x = dim_(0);
  voxel_map.dim.y = dim_(1);
  voxel_map.dim.z = 1;

  voxel_map.resolution = res_;

  voxel_map.data.resize(dim_(0) * dim_(1), val_free);

  // discretize the thickness into number of voxels
  int hi = hh / res_;
  // calculate the starting z-axis value for the slice
  int h_min = (h - origin_d_(2)) / res_ - hi;
  h_min = h_min >= 0 ? h_min : 0;
  h_min = h_min < dim_(2) ? h_min : dim_(2) - 1;
  int h_max = (h - origin_d_(2)) / res_ + hi + 1;
  h_max = h_max > 0 ? h_max : 1;
  h_max = h_max <= dim_(2) ? h_max : dim_(2);

  Vec3i n;
  for (n(0) = 0; n(0) < dim_(0); n(0)++) {
    for (n(1) = 0; n(1) < dim_(1); n(1)++) {
      for (n(2) = h_min; n(2) < h_max; n(2)++) {
        if (inflated_map_[n(0)][n(1)][n(2)] > val_even)
          voxel_map.data[n(0) + dim_(0) * n(1)] = val_occ;
      }
    }
  }

  return voxel_map;
}

bool VoxelMapper::allocate(const Vec3f& new_dim_d, const Vec3f& new_ori_d) {
  Vec3i new_dim(new_dim_d(0) / res_, new_dim_d(1) / res_, new_dim_d(2) / res_);
  Vec3i new_ori(new_ori_d(0) / res_, new_ori_d(1) / res_, new_ori_d(2) / res_);
  if (new_dim(2) == 0)  // 2d case, set the z dimension to be 1
    new_dim(2) = 1;

  if (new_dim(0) == dim_(0) && new_dim(1) == dim_(1) && new_dim(2) == dim_(2) &&
      new_ori(0) == origin_(0) && new_ori(1) == origin_(1) &&
      new_ori(2) == origin_(2))
    return false;
  else {
    boost::multi_array<int8_t, 3> new_map(
        boost::extents[new_dim(0)][new_dim(1)][new_dim(2)]);
    std::fill(new_map.data(), new_map.data() + new_map.num_elements(),
              val_default);
    for (int l = 0; l < new_dim(0); l++) {
      for (int w = 0; w < new_dim(1); w++) {
        for (int h = 0; h < new_dim(2); h++) {
          if (l + new_ori(0) >= origin_(0) && w + new_ori(1) >= origin_(1) &&
              h + new_ori(2) >= origin_(2) &&
              l + new_ori(0) < origin_(0) + dim_(0) &&
              w + new_ori(1) < origin_(1) + dim_(1) &&
              h + new_ori(2) < origin_(2) + dim_(2)) {
            int new_l = l + new_ori(0) - origin_(0);
            int new_w = w + new_ori(1) - origin_(1);
            int new_h = h + new_ori(2) - origin_(2);

            new_map[l][w][h] = map_[new_l][new_w][new_h];
          }
        }
      }
    }

    map_.resize(boost::extents[new_dim(0)][new_dim(1)][new_dim(2)]);
    map_ = new_map;
    inflated_map_.resize(boost::extents[new_dim(0)][new_dim(1)][new_dim(2)]);
    inflated_map_ = new_map;

    dim_ = new_dim;
    origin_ = new_ori;
    origin_d_ = new_ori_d;

    return true;
  }
}

// void VoxelMapper::addCloud2D(const vec_Vec3f& pts, const Aff3f& TF,
//                              const vec_Vec3i& ns, bool ray_trace,
//                              double upper_h, double lower_h, double
//                              max_range) {
//   const Vec3f pos(TF.translation().x(), TF.translation().y(),
//                   TF.translation().z());
//   const Vec3f origin = origin_d_;

//   for (const auto& it : pts) {
//     Vec3f pt = TF * it;
//     if (pt(2) > upper_h || pt(2) < lower_h ||
//         (max_range > 0 && it.norm() > max_range))
//       continue;
//     pt(2) = origin(2);
//     const Vec3i pn = floatToInt(pt);

//    // for each point (even outside), do ray trace
//     if (ray_trace) {
//       vec_Vec3i rays = rayTrace(pos, pt);
//       for (const auto& pn : rays) {
//         if (map_[pn(0)][pn(1)][pn(2)] == val_unknown) {
//           map_[pn(0)][pn(1)][pn(2)] = val_free;
//           if (inflated_map_[pn(0)][pn(1)][pn(2)] == val_unknown)
//             inflated_map_[pn(0)][pn(1)][pn(2)] = val_free;
//         }
//       }
//     }

//     if (isOutSide(pn)) continue;

//     if (map_[pn(0)][pn(1)][pn(2)] <= val_even) {
//       for (const auto& it_n : ns) {
//         Vec3i n2 = pn + it_n;
//         if (!isOutSide(n2) && inflated_map_[n2(0)][n2(1)][n2(2)] <= val_occ)
//         {
//           inflated_map_[n2(0)][n2(1)][n2(2)] = val_occ;
//         }
//       }
//       map_[pn(0)][pn(1)][pn(2)] = val_occ;
//       inflated_map_[pn(0)][pn(1)][pn(2)] = val_occ;
//     }
//   }
// }

void VoxelMapper::addCloud(const vec_Vec3f& pts, const Aff3f& TF,
                           const vec_Vec3i& ns, bool ray_trace,
                           double max_range) {
  const Vec3f pos(TF.translation().x(), TF.translation().y(),
                  TF.translation().z());

  // Decay cloud which is within a local region around the robot
  if (val_decay > 0) {
    double max_decay_range = max_range * 2.0;
    ROS_WARN_ONCE("[Mapper]: dacaying the point cloud within local range");
    decayLocalCloud(pos, max_decay_range);
  } else {
    ROS_WARN_ONCE("[Mapper]: dacaying is disabled");
  };

  for (const auto& it : pts) {
    // through away points outside max_range first to save computation
    if ((max_range > 0) && (it.norm() > max_range)) continue;

    // transform points from lidar frame to global frame
    const Vec3f pt = TF * lidar_rot_ * it;
    const Vec3i n = floatToInt(pt);

    // through away points outside voxel box to save computation.
    // TODO: if unknown vs known matters (i.e. planning algorithm differentiates
    // unknown and free), need to move this after ray_trace. Won't add much
    // computation according to timer feedback.
    if (isOutSide(n)) continue;

    // for each point do ray trace
    if (ray_trace) {
      vec_Vec3i rays = rayTrace(pos, pt);
      for (const auto& pn : rays) {
        if (map_[pn(0)][pn(1)][pn(2)] == val_unknown) {
          map_[pn(0)][pn(1)][pn(2)] = val_free;
          if (inflated_map_[pn(0)][pn(1)][pn(2)] == val_unknown)
            inflated_map_[pn(0)][pn(1)][pn(2)] = val_free;
        }
      }
    }

    // Add val_add to the voxel whenever a point lies in it. The voxel will be
    // occupied after N*T > (val_occ - val_free) / val_add, where N is the
    // number of points and T is number of scans.
    if (map_[n(0)][n(1)][n(2)] < val_occ) {
      map_[n(0)][n(1)][n(2)] =
          map_[n(0)][n(1)][n(2)] +
          val_add;  //
                    // Do the same to voxels in the inflation region
      if (inflated_map_[n(0)][n(1)][n(2)] < val_occ) {
        inflated_map_[n(0)][n(1)][n(2)] =
            inflated_map_[n(0)][n(1)][n(2)] + val_add;
      }
      // ns is a vector of values from -inflation_range to +inflation_range
      // excluding 0
      for (const auto& it_n : ns) {
        Vec3i n2 = n + it_n;
        if (!isOutSide(n2) && inflated_map_[n2(0)][n2(1)][n2(2)] < val_occ) {
          inflated_map_[n2(0)][n2(1)][n2(2)] =
              inflated_map_[n2(0)][n2(1)][n2(2)] + val_add;
        }
      }
    }
  }
}

void VoxelMapper::freeCloud(const vec_Vec3f& pts, const Aff3f& TF) {
  const Vec3f pos(TF.translation().x(), TF.translation().y(),
                  TF.translation().z());

  for (const auto& it : pts) {
    const Vec3f pt = TF * it;

    vec_Vec3i rays = rayTrace(pos, pt);
    for (const auto& pn : rays) {
      if (map_[pn(0)][pn(1)][pn(2)] == val_unknown) {
        map_[pn(0)][pn(1)][pn(2)] = val_free;
        if (inflated_map_[pn(0)][pn(1)][pn(2)] == val_unknown)
          inflated_map_[pn(0)][pn(1)][pn(2)] = val_free;
      }
    }
  }
}

vec_Vec3i VoxelMapper::rayTrace(const Vec3f& pt1, const Vec3f& pt2) {
  Vec3f diff = pt2 - pt1;
  decimal_t k = 0.8;
  int max_diff = (diff / res_).lpNorm<Eigen::Infinity>() / k;
  decimal_t s = 1.0 / max_diff;
  Vec3f step = diff * s;

  vec_Vec3i pns;
  Vec3i prev_pn = Vec3i::Constant(-1000000);
  for (int n = 1; n < max_diff; n++) {
    Vec3i new_pn = floatToInt(pt1 + step * n);
    if (isOutSide(new_pn)) continue;
    if (new_pn != prev_pn) pns.push_back(new_pn);
    prev_pn = new_pn;
  }
  return pns;
}

Vec3i VoxelMapper::floatToInt(const Vec3f& pt) {
  return Vec3i(std::round((pt(0) - origin_d_(0)) / res_),
               std::round((pt(1) - origin_d_(1)) / res_),
               std::round((pt(2) - origin_d_(2)) / res_));
}

Vec3f VoxelMapper::intToFloat(const Vec3i& pn) {
  return pn.cast<decimal_t>() * res_ + origin_d_;
}

bool VoxelMapper::isOutSide(const Vec3i& pn) {
  return pn(0) < 0 || pn(0) >= dim_(0) || pn(1) < 0 || pn(1) >= dim_(1) ||
         pn(2) < 0 || pn(2) >= dim_(2);
}
