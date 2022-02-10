#pragma once

#include <mpl_basis/data_type.h>

#include <cmath>
#include <iostream>
#include <limits>   // numeric_limits
#include <utility>  // pair
template <int Dim>
struct Hyperplane {
  Hyperplane() {}
  Hyperplane(const Vecf<Dim>& p, const Vecf<Dim>& n) : p_(p), n_(n) {}

  /// Calculate the signed distance from point
  double signed_dist(const Vecf<Dim>& pt) const { return n_.dot(pt - p_); }

  /// Calculate the distance from point
  double dist(const Vecf<Dim>& pt) const { return std::abs(signed_dist(pt)); }

  /// Point on the plane
  Vecf<Dim> p_;
  /// Normal of the plane, directional
  Vecf<Dim> n_;
};

/// Hyperplane2D: first is the point on the hyperplane, second is the normal
typedef Hyperplane<2> Hyperplane2D;
/// Hyperplane3D: first is the point on the hyperplane, second is the normal
typedef Hyperplane<3> Hyperplane3D;

/// Polyhedron class
template <int Dim>
struct Polyhedron {
  /// Null constructor
  Polyhedron() {}
  /// Construct from Hyperplane array
  explicit Polyhedron(const vec_E<Hyperplane<Dim>>& vs) : vs_(vs) {}

  /// Append Hyperplane
  void add(const Hyperplane<Dim>& v) { vs_.push_back(v); }

  /// Check if the point is inside polyhedron, non-exclusive
  bool inside(const Vecf<Dim>& pt) const {
    for (const auto& v : vs_) {
      if (v.signed_dist(pt) > 1e-10) {
        // printf("rejected pt: (%f, %f), d: %f\n",pt(0), pt(1),
        // v.signed_dist(pt));
        return false;
      }
    }
    return true;
  }

  /// Calculate points inside polyhedron, non-exclusive
  vec_Vecf<Dim> points_inside(const vec_Vecf<Dim>& O) const {
    vec_Vecf<Dim> new_O;
    for (const auto& it : O) {
      if (inside(it)) new_O.push_back(it);
    }
    return new_O;
  }

  /// Calculate normals, used for visualization
  vec_E<std::pair<Vecf<Dim>, Vecf<Dim>>> cal_normals() const {
    vec_E<std::pair<Vecf<Dim>, Vecf<Dim>>> ns(vs_.size());
    for (size_t i = 0; i < vs_.size(); i++)
      ns[i] = std::make_pair(vs_[i].p_,
                             vs_[i].n_);  // fist is point, second is normal
    return ns;
  }

  /// Get the hyperplane array
  vec_E<Hyperplane<Dim>> hyperplanes() const { return vs_; }

  /// Hyperplane array
  vec_E<Hyperplane<Dim>> vs_;  // normal must go outside
};

/// Polyhedron2D, consists of 2D hyperplane
typedef Polyhedron<2> Polyhedron2D;
/// Polyhedron3D, consists of 3D hyperplane
typedef Polyhedron<3> Polyhedron3D;

///[A, b] for \f$Ax < b\f$
template <int Dim>
struct LinearConstraint {
  /// Null constructor
  LinearConstraint() {}
  /// Construct from \f$A, b\f$ directly, s.t \f$Ax < b\f$
  LinearConstraint(const MatDNf<Dim>& A, const VecDf& b) : A_(A), b_(b) {}
  /**
   * @brief Construct from a inside point and hyperplane array
   * @param p0 point that is inside
   * @param vs hyperplane array, normal should go outside
   */
  LinearConstraint(const Vecf<Dim> p0, const vec_E<Hyperplane<Dim>>& vs) {
    const unsigned int size = vs.size();
    MatDNf<Dim> A(size, Dim);
    VecDf b(size);

    for (unsigned int i = 0; i < size; i++) {
      auto n = vs[i].n_;
      double c = vs[i].p_.dot(n);
      if (n.dot(p0) - c > 0) {
        n = -n;
        c = -c;
      }
      A.row(i) = n;
      b(i) = c;
    }

    A_ = A;
    b_ = b;
  }

  /// Check if the point is inside polyhedron using linear constraint
  bool inside(const Vecf<Dim>& pt) {
    VecDf d = A_ * pt - b_;
    for (unsigned int i = 0; i < d.rows(); i++) {
      if (d(i) > 0) return false;
    }
    return true;
  }

  /// Get \f$A\f$ matrix
  MatDNf<Dim> A() const { return A_; }

  /// Get \f$b\f$ matrix
  VecDf b() const { return b_; }

  MatDNf<Dim> A_;
  VecDf b_;
};

/// LinearConstraint 2D
typedef LinearConstraint<2> LinearConstraint2D;
/// LinearConstraint 3D
typedef LinearConstraint<3> LinearConstraint3D;

template <int Dim>
struct Ellipsoid {
  Ellipsoid() {}
  Ellipsoid(const Matf<Dim, Dim>& C, const Vecf<Dim>& d) : C_(C), d_(d) {}

  /// Calculate distance to the center
  double dist(const Vecf<Dim>& pt) const {
    return (C_.inverse() * (pt - d_)).norm();
  }

  /// Check if the point is inside, non-exclusive
  bool inside(const Vecf<Dim>& pt) const { return dist(pt) <= 1; }

  /// Calculate points inside ellipsoid, non-exclusive
  vec_Vecf<Dim> points_inside(const vec_Vecf<Dim>& O) const {
    vec_Vecf<Dim> new_O;
    for (const auto& it : O) {
      if (inside(it)) new_O.push_back(it);
    }
    return new_O;
  }

  /// Find the closest point
  Vecf<Dim> closest_point(const vec_Vecf<Dim>& O) const {
    Vecf<Dim> pt = Vecf<Dim>::Zero();
    double min_dist = std::numeric_limits<double>::max();
    for (const auto& it : O) {
      double d = dist(it);
      if (d < min_dist) {
        min_dist = d;
        pt = it;
      }
    }
    return pt;
  }

  /// Find the closest hyperplane from the closest point
  Hyperplane<Dim> closest_hyperplane(const vec_Vecf<Dim>& O) const {
    const auto closest_pt = closest_point(O);
    const auto n = C_.inverse() * C_.inverse().transpose() * (closest_pt - d_);
    return Hyperplane<Dim>(closest_pt, n.normalized());
  }

  /// Sample n points along the contour
  template <int U = Dim>
  typename std::enable_if<U == 2, vec_Vecf<U>>::type sample(int num) const {
    vec_Vecf<Dim> pts;
    double dyaw = M_PI * 2 / num;
    for (double yaw = 0; yaw < M_PI * 2; yaw += dyaw) {
      Vecf<Dim> pt;
      pt << cos(yaw), sin(yaw);
      pts.push_back(C_ * pt + d_);
    }
    return pts;
  }

  void print() const {
    std::cout << "C: " << C_ << std::endl;
    std::cout << "d: " << d_ << std::endl;
  }

  /// Get ellipsoid volume
  double volume() const { return C_.determinant(); }

  /// Get C matrix
  Matf<Dim, Dim> C() const { return C_; }

  /// Get center
  Vecf<Dim> d() const { return d_; }

  Matf<Dim, Dim> C_;
  Vecf<Dim> d_;
};

typedef Ellipsoid<2> Ellipsoid2D;

typedef Ellipsoid<3> Ellipsoid3D;
