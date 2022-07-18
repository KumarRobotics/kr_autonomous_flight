#include "mpl_collision/map_util.h"

#include <iostream>

namespace MPL {

template <int Dim>
void MapUtil<Dim>::setMap(const Vecf<Dim> &ori, const Veci<Dim> &dim,
                          const Tmap &map, decimal_t res) {
  map_ = map;
  dim_ = dim;
  origin_d_ = ori;
  res_ = res;
}

template <int Dim>
void MapUtil<Dim>::info() {
  Vecf<Dim> range = dim_.template cast<decimal_t>() * res_;
  std::cout << "MapUtil Info ========================== " << std::endl;
  std::cout << "   res: [" << res_ << "]" << std::endl;
  std::cout << "   origin: [" << origin_d_.transpose() << "]" << std::endl;
  std::cout << "   range: [" << range.transpose() << "]" << std::endl;
  std::cout << "   dim: [" << dim_.transpose() << "]" << std::endl;
}

template <int Dim>
Veci<Dim> MapUtil<Dim>::floatToInt(const Vecf<Dim> &pt) {
  Veci<Dim> pn;
  for (int i = 0; i < Dim; i++)
    pn(i) = std::floor((pt(i) - origin_d_(i)) / res_);
  return pn;
}

template <int Dim>
Vecf<Dim> MapUtil<Dim>::intToFloat(const Veci<Dim> &pn) {
  // return pn.template cast<decimal_t>() * res_ + origin_d_;
  return (pn.template cast<decimal_t>() + Vecf<Dim>::Constant(0.5)) * res_ +
         origin_d_;
}

template <int Dim>
vec_Veci<Dim> MapUtil<Dim>::rayTrace(const Vecf<Dim> &pt1,
                                     const Vecf<Dim> &pt2) {
  Vecf<Dim> diff = pt2 - pt1;
  decimal_t k = 0.8;
  int max_diff = (diff / res_).template lpNorm<Eigen::Infinity>() / k;
  decimal_t s = 1.0 / max_diff;
  Vecf<Dim> step = diff * s;

  vec_Veci<Dim> pns;
  Veci<Dim> prev_pn = Veci<Dim>::Constant(-1);
  for (int n = 1; n < max_diff; n++) {
    Vecf<Dim> pt = pt1 + step * n;
    Veci<Dim> new_pn = floatToInt(pt);
    if (isOutside(new_pn)) break;
    if (new_pn != prev_pn) pns.push_back(new_pn);
    prev_pn = new_pn;
  }
  return pns;
}

template <int Dim>
vec_Vecf<Dim> MapUtil<Dim>::getCloud() {
  vec_Vecf<Dim> cloud;
  Veci<Dim> n;
  if constexpr (Dim == 3) {
    for (n(2) = 0; n(2) < dim_(2); n(2)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        for (n(0) = 0; n(0) < dim_(0); n(0)++) {
          if (isOccupied(getIndex(n))) cloud.push_back(intToFloat(n));
        }
      }
    }
  } else {
    for (n(1) = 0; n(1) < dim_(1); n(1)++) {
      for (n(0) = 0; n(0) < dim_(0); n(0)++) {
        if (isOccupied(getIndex(n))) cloud.push_back(intToFloat(n));
      }
    }
  }
  return cloud;
}

template <int Dim>
vec_Vecf<Dim> MapUtil<Dim>::getFreeCloud() {
  vec_Vecf<Dim> cloud;
  Veci<Dim> n;

  if constexpr (Dim == 3) {
    for (n(2) = 0; n(2) < dim_(2); n(2)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        for (n(0) = 0; n(0) < dim_(0); n(0)++) {
          if (isFree(getIndex(n))) cloud.push_back(intToFloat(n));
        }
      }
    }
  } else {
    for (n(1) = 0; n(1) < dim_(1); n(1)++) {
      for (n(0) = 0; n(0) < dim_(0); n(0)++) {
        if (isFree(getIndex(n))) cloud.push_back(intToFloat(n));
      }
    }
  }
  return cloud;
}

template <int Dim>
vec_Vecf<Dim> MapUtil<Dim>::getUnknownCloud() {
  vec_Vecf<Dim> cloud;
  Veci<Dim> n;

  if constexpr (Dim == 3) {
    for (n(2) = 0; n(2) < dim_(2); n(2)++) {
      for (n(1) = 0; n(1) < dim_(1); n(1)++) {
        for (n(0) = 0; n(0) < dim_(0); n(0)++) {
          if (isUnknown(getIndex(n))) cloud.push_back(intToFloat(n));
        }
      }
    }
  } else {
    for (n(1) = 0; n(1) < dim_(1); n(1)++) {
      for (n(0) = 0; n(0) < dim_(0); n(0)++) {
        if (isUnknown(getIndex(n))) cloud.push_back(intToFloat(n));
      }
    }
  }
  return cloud;
}

// Explicit instantiation
template class MapUtil<2>;
template class MapUtil<3>;

}  // namespace MPL
