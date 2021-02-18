// Copyright 2015 Michael Watterson
#ifndef TRAJ_OPT_BASIC_TYPES_H_
#define TRAJ_OPT_BASIC_TYPES_H_

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>

// This file is to define typdefs for all the eigen types as either float or
// double
// Add flag "-DTRAJ_OPT_USE_SINGLE_PRECISION" to compile the entire package with
// float,
// otherwise, we will default to use double

namespace traj_opt {

#ifndef TRAJ_OPT_USE_SINGLE_PRECISION
typedef double decimal_t;
#else
typedef float decimal_t;
#endif

typedef Eigen::Matrix<decimal_t, 3, 1> Vec3;
typedef Eigen::Matrix<decimal_t, 4, 1> Vec4;
typedef Eigen::Matrix<decimal_t, 5, 1> Vec5;
typedef Eigen::Matrix<decimal_t, 6, 1> Vec6;
typedef Eigen::Matrix<decimal_t, 3, 3> Mat3;
typedef Eigen::Matrix<decimal_t, 4, 4> Mat4;

typedef std::vector<Vec4, Eigen::aligned_allocator<Vec4>> Vec4Vec;
typedef std::vector<Vec3, Eigen::aligned_allocator<Vec3>> Vec3Vec;
typedef std::vector<Mat4, Eigen::aligned_allocator<Mat4>> Mat4Vec;

typedef Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> VecD;
typedef Eigen::Matrix<decimal_t, Eigen::Dynamic, 3> MatD3;
typedef Eigen::Matrix<decimal_t, Eigen::Dynamic, Eigen::Dynamic> MatD;

typedef std::vector<VecD> VecDVec;
typedef std::vector<MatD> MatDVec;

typedef Eigen::Quaternion<decimal_t> Quat;
}  // namespace traj_opt
#endif  // TRAJ_OPT_BASIC_TYPES_H_
