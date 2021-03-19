// Copyright 2015 Michael Watterson
#pragma once

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>

namespace traj_opt {

typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 4, 1> Vec4;
typedef Eigen::Matrix<double, 5, 1> Vec5;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::Matrix<double, 3, 3> Mat3;
typedef Eigen::Matrix<double, 4, 4> Mat4;

typedef std::vector<Vec4, Eigen::aligned_allocator<Vec4>> Vec4Vec;
typedef std::vector<Vec3, Eigen::aligned_allocator<Vec3>> Vec3Vec;
typedef std::vector<Mat4, Eigen::aligned_allocator<Mat4>> Mat4Vec;

typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecD;
typedef Eigen::Matrix<double, Eigen::Dynamic, 3> MatD3;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatD;

typedef std::vector<VecD> VecDVec;
typedef std::vector<MatD> MatDVec;

typedef Eigen::Quaternion<double> Quat;

}  // namespace traj_opt
