// Copyright 2016 KumarRobotics - Kartik Mohta
#include "fla_ukf/fla_ukf.h"

#include <angles/angles.h>

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <cmath>
#include <utility>

FLAUKF::FLAUKF() {
  // Init State
  xa_.setZero();

  // Init Covariance
  Pa_.setZero();
  Pa_(0, 0) = 0.1 * 0.1;
  Pa_(1, 1) = 0.1 * 0.1;
  Pa_(2, 2) = 0.1 * 0.1;
  Pa_(3, 3) = 0.1 * 0.1;
  Pa_(4, 4) = 0.1 * 0.1;
  Pa_(5, 5) = 0.1 * 0.1;
  Pa_(6, 6) = 10 * M_PI / 180 * 10 * M_PI / 180;
  Pa_(7, 7) = 10 * M_PI / 180 * 10 * M_PI / 180;
  Pa_(8, 8) = 10 * M_PI / 180 * 10 * M_PI / 180;
  Pa_(9, 9) = 0.01 * 0.01;
  Pa_(10, 10) = 0.01 * 0.01;
  Pa_(11, 11) = 0.01 * 0.01;
  Pa_(12, 12) = 0.01 * 0.01;
  Pa_(13, 13) = 0.01 * 0.01;
  Pa_(14, 14) = 0.01 * 0.01;
  Pa_(15, 15) = 0.1 * 0.1;
  Rv_.setIdentity();
  // Init Sigma Points
  alpha_ = 0.001;
  beta_ = 2;
  kappa_ = 0;
  // Other Inits
  g_ = 9.81;
  init_process_ = false;
  init_meas_ = false;

  // Handle multi-floor stuff
  curr_floor_height_ = 0.0;
  known_floor_heights_.push_back(0);
  floor_change_threshold_ = 0.2;
  floor_merge_threshold_ = 0.1;
#if 0
  height_hist_duration_ = 0.1;
#else
  height_hist_length_ = 3;
#endif
}

// Need this as per:
// https://stackoverflow.com/questions/8016780/undefined-reference-to-static-constexpr-char
constexpr std::array<unsigned int, 3> FLAUKF::meas_laser_idx_;
constexpr std::array<unsigned int, 6> FLAUKF::meas_cam_idx_;
constexpr std::array<unsigned int, 6> FLAUKF::meas_gps_idx_;
constexpr std::array<unsigned int, 1> FLAUKF::meas_yaw_idx_;
#if VIO_NO_POSITION
constexpr std::array<unsigned int, 6> FLAUKF::meas_vio_idx_;
#else
constexpr std::array<unsigned int, 9> FLAUKF::meas_vio_idx_;
#endif

const FLAUKF::StateVec &FLAUKF::GetState() const { return xa_; }

const FLAUKF::StateCov &FLAUKF::GetStateCovariance() const { return Pa_; }

void FLAUKF::SetGravity(Scalar_t _g) { g_ = _g; }

void FLAUKF::SetCurrentFloorHeight(Scalar_t h) { curr_floor_height_ = h; }

void FLAUKF::SetFloorChangeThreshold(Scalar_t t) {
  floor_change_threshold_ = t;
}

void FLAUKF::SetFloorMergeThreshold(Scalar_t t) { floor_merge_threshold_ = t; }

void FLAUKF::SetImuCovariance(const ProcNoiseCov &_Rv) { Rv_ = _Rv; }

void FLAUKF::SetParameters(Scalar_t _alpha, Scalar_t _beta, Scalar_t _kappa) {
  alpha_ = _alpha;
  beta_ = _beta;
  kappa_ = _kappa;
}

bool FLAUKF::ProcessUpdate(const InputVec &u, const ros::Time &time) {
  // Init Time
  Scalar_t dt;
  if (!init_process_ || !init_meas_) {
    init_process_ = true;
    prev_proc_update_time_ = time;
    return false;
  }

  dt = (time - prev_proc_update_time_).toSec();
  prev_proc_update_time_ = time;

  constexpr unsigned int L_proc = state_count_ + proc_noise_count_;

  // Generate sigma points
  const Mat<L_proc, 2 *L_proc + 1> Xaa = GenerateSigmaPoints(Rv_);

  // Extract state and noise from augmented state
  // NOTE(Kartik): The fixed size version causes a segfault in Release mode
  // Mat<state_count_, 2 *L_proc + 1> Xa = Xaa.topRows<state_count_>();
  Mat<Eigen::Dynamic, 2 *L_proc + 1> Xa = Xaa.topRows<state_count_>();
  const Mat<proc_noise_count_, 2 *L_proc + 1> Wa =
      Xaa.bottomRows<proc_noise_count_>();

  for (unsigned int k = 0; k < Xa.cols(); k++)
    Xa.col(k) = ProcessModel(Xa.col(k), u, Wa.col(k), dt);

  // Handle jump between +pi and -pi !
  const Scalar_t minYaw = Xa.row(8).minCoeff();
  const Scalar_t maxYaw = Xa.row(8).maxCoeff();
  if (std::abs(minYaw - maxYaw) > M_PI) {
    for (unsigned int k = 0; k < Xa.cols(); k++)
      if (Xa(8, k) < 0) Xa(8, k) += 2 * M_PI;
  }

  // Mean
  xa_ = wm_.replicate<state_count_, 1>().cwiseProduct(Xa).rowwise().sum();

  // Covariance
  Pa_.setZero();
  for (unsigned int k = 0; k < Xa.cols(); k++) {
    const StateVec d = Xa.col(k) - xa_;
    Pa_.noalias() += wc_(k) * d * d.transpose();
  }
  return true;
}

void FLAUKF::GenerateWeights(unsigned int L) {
  lambda_ = alpha_ * alpha_ * (L + kappa_) - L;
  wm_.resize(1, 2 * L + 1);
  wc_.resize(1, 2 * L + 1);
  wm_(0) = lambda_ / (L + lambda_);
  wc_(0) = lambda_ / (L + lambda_) + (1 - alpha_ * alpha_ + beta_);
  for (unsigned int k = 1; k <= 2 * L; k++) {
    wm_(k) = 1 / (2 * (L + lambda_));
    wc_(k) = 1 / (2 * (L + lambda_));
  }
  gamma_ = std::sqrt(L + lambda_);
}

static FLAUKF::Mat<3, 3> rpy_to_R(const FLAUKF::Vec<3> &rpy) {
  return FLAUKF::Mat<3, 3>{
      Eigen::AngleAxis<FLAUKF::Scalar_t>(rpy(2), FLAUKF::Vec<3>::UnitZ()) *
      Eigen::AngleAxis<FLAUKF::Scalar_t>(rpy(1), FLAUKF::Vec<3>::UnitY()) *
      Eigen::AngleAxis<FLAUKF::Scalar_t>(rpy(0), FLAUKF::Vec<3>::UnitX())};
}

static FLAUKF::Vec<3> R_to_rpy(const FLAUKF::Mat<3, 3> &R) {
  FLAUKF::Vec<3> rpy;
  rpy(0) = std::atan2(R(2, 1), R(2, 2));
  rpy(1) = -std::asin(R(2, 0));
  rpy(2) = std::atan2(R(1, 0), R(0, 0));
  return rpy;
}

/**
 * Copied from
 * https://github.com/KumarRobotics/kr_utils/blob/master/kr_math/include/kr_math/SO3.hpp
 *
 *  @brief Create a skew-symmetric matrix from a 3-element vector.
 *  @note Performs the operation:
 *  w   ->  [  0 -w3  w2]
 *          [ w3   0 -w1]
 *          [-w2  w1   0]
 */
FLAUKF::Mat<3, 3> skewSymmetric(const FLAUKF::Vec<3> &w) {
  FLAUKF::Mat<3, 3> W;
  W(0, 0) = 0;
  W(0, 1) = -w(2);
  W(0, 2) = w(1);
  W(1, 0) = w(2);
  W(1, 1) = 0;
  W(1, 2) = -w(0);
  W(2, 0) = -w(1);
  W(2, 1) = w(0);
  W(2, 2) = 0;
  return W;
}

FLAUKF::StateVec FLAUKF::ProcessModel(const StateVec &x, const InputVec &u,
                                      const ProcNoiseVec &w, Scalar_t dt) {
  const Mat<3, 3> R = rpy_to_R(x.segment<3>(6));
  const Vec<3> ag(0, 0, g_);

  // Acceleration
  const Vec<3> a = u.segment<3>(0) - x.segment<3>(9) + w.segment<3>(0);
  const Vec<3> ddx = R * a - ag;

  // Rotation
  const Vec<3> omega = u.segment<3>(3) - x.segment<3>(12) + w.segment<3>(3);
  const Mat<3, 3> dR = Mat<3, 3>::Identity() + skewSymmetric(omega) * dt;
  const Mat<3, 3> Rt = R * dR;

  // State
  StateVec xt{StateVec::Zero()};
  xt.segment<3>(0) = x.segment<3>(0) + x.segment<3>(3) * dt + ddx * dt * dt / 2;
  xt.segment<3>(3) = x.segment<3>(3) + ddx * dt;
  xt.segment<3>(6) = R_to_rpy(Rt);
  xt.segment<3>(9) = x.segment<3>(9) + w.segment<3>(6) * dt;
  xt.segment<3>(12) = x.segment<3>(12) + w.segment<3>(9) * dt;
  xt(15) = x(15) + w(12) * dt;

  return xt;
}

bool FLAUKF::MeasurementUpdateLaser(const MeasLaserVec &z,
                                    const MeasLaserCov &RnLaser,
                                    const ros::Time &time) {
  // Init
  if (!init_process_ || !init_meas_) {
    std::cout << "MeasurementUpdateLaser:" << std::endl;
    for (unsigned int i = 0; i < meas_laser_count_; ++i) {
      xa_(meas_laser_idx_[i]) = z(i);
      std::cout << "z(" << i << "): " << z(i) << std::endl;
    }
    init_meas_ = true;
    return false;
  }

  // Get Measurement
  Mat<meas_laser_count_, state_count_> H;
  H.setZero();
  for (unsigned int i = 0; i < meas_laser_count_; ++i) {
    H(i, meas_laser_idx_[i]) = 1;
  }
  MeasLaserCov S = H * Pa_ * H.transpose() + RnLaser;
  // Kalman Gain;
  Mat<state_count_, meas_laser_count_> K = Pa_ * H.transpose() * S.inverse();
  // Innovation
  MeasLaserVec inno = z - H * xa_;
  // Handle yaw angle jumps
  inno(meas_laser_count_ - 1) =
      angles::normalize_angle(inno(meas_laser_count_ - 1));
  // Posterior Mean
  xa_ += K * inno;
  // Posterior Covariance
  Pa_ -= K * H * Pa_;

  return true;
}

bool FLAUKF::MeasurementUpdateCam(const MeasCamVec &z, const MeasCamCov &RnCam,
                                  const ros::Time &time) {
  // Init
  if (!init_process_ || !init_meas_) {
    std::cout << "MeasurementUpdateCam:" << std::endl;
    for (unsigned int i = 0; i < meas_cam_count_; ++i) {
      xa_(meas_cam_idx_[i]) = z(i);
      std::cout << "z(" << i << "): " << z(i) << std::endl;
    }
    init_meas_ = true;
    return false;
  }

#if 0  // Linear update
  // Get Measurement
  Mat<meas_cam_count_, state_count_> H;
  H.setZero();
  for (unsigned int i = 0; i < meas_cam_count_; ++i) {
    H(i, meas_cam_idx_[i]) = 1;
  }
  MeasCamCov S = H * Pa_ * H.transpose() + RnCam;
  // Kalman Gain;
  Mat<state_count_, meas_cam_count_> K = Pa_ * H.transpose() * S.inverse();
  // Innovation
  MeasCamVec inno = z - H * xa_;
  // Handle angle jumps
  inno(8) = std::asin(std::sin(inno(8)));

  // Posterior Mean
  xa_ += K * inno;
  // Posterior Covariance
  Pa_ -= K * H * Pa_;
#else
  // Generate sigma points
  constexpr unsigned int L = state_count_;
  const Mat<L, 2 *L + 1> Xaa = GenerateSigmaPoints(Mat<0, 0>::Zero());
  // Extract state and noise from augmented state
  const Mat<state_count_, 2 *L + 1> Xa = Xaa.topRows<state_count_>();
  Mat<meas_cam_count_, 2 * L + 1> Za;
  // Mean
  for (unsigned int k = 0; k < Xa.cols(); k++)
    Za.col(k) = MeasurementModelCam(Xa.col(k));

  MeasCamVec z_pred =
      wm_.replicate<meas_cam_count_, 1>().cwiseProduct(Za).rowwise().sum();

  // Covariance
  MeasCamCov Pzz;
  Mat<state_count_, meas_cam_count_> Pxz;
  Pzz.setZero();
  Pxz.setZero();
  for (unsigned int k = 0; k < Xa.cols(); k++) {
    const MeasCamVec d_z = Za.col(k) - z_pred;
    const StateVec d_x = Xa.col(k) - xa_;
    Pzz.noalias() += wc_(k) * d_z * d_z.transpose();
    Pxz.noalias() += wc_(k) * d_x * d_z.transpose();
  }
  Pzz += RnCam;

  Mat<state_count_, meas_cam_count_> K = Pxz * Pzz.inverse();

  // Calculate innovation
  MeasCamVec inno = z - z_pred;

  // Take proper angle difference
  const Vec<3> rpy_pred = z_pred.segment<3>(3);
  const Vec<3> rpy_meas = z.segment<3>(3);
  const auto R_pred = rpy_to_R(rpy_pred);
  const auto R_meas = rpy_to_R(rpy_meas);
  const auto delta_R = R_pred.transpose() * R_meas;
  const auto delta_rpy = R_to_rpy(delta_R);

  inno.segment<3>(3) = delta_rpy;
  // std::cout << "Inno: " << inno.segment<3>(3).transpose() << std::endl;

  xa_ += K * inno;
  Pa_ -= K * Pzz * K.transpose();

#endif

  return true;
}

FLAUKF::MeasCamVec FLAUKF::MeasurementModelCam(const StateVec &x) {
  MeasCamVec z;
  z.segment<3>(0) = x.segment<3>(0);
  z.segment<3>(3) = x.segment<3>(6);

  return z;
}

bool FLAUKF::MeasurementUpdateHeight(const MeasHeightVec &z,
                                     const MeasHeightCov &RnHeight,
                                     const ros::Time &time) {
  // ROS_INFO("GOT HEIGHT");
  if (!init_process_ || !init_meas_) return false;

  constexpr unsigned int L = state_count_;

  // Generate sigma points (linear noise model)
  const Mat<L, 2 *L + 1> Xaa = GenerateSigmaPoints(Mat<0, 0>::Zero());
  // Extract state and noise from augmented state
  const Mat<state_count_, 2 *L + 1> Xa = Xaa.topRows<state_count_>();
  Mat<meas_height_count_, 2 * L + 1> Za;
  // Mean
  for (unsigned int k = 0; k < Xa.cols(); k++)
    Za.col(k) = MeasurementModelHeight(Xa.col(k));

  MeasHeightVec z_pred =
      wm_.replicate<meas_height_count_, 1>().cwiseProduct(Za).rowwise().sum();

  const Scalar_t h_meas = z(0) * std::cos(xa_(7)) * std::cos(xa_(6));

  // Filter out very low or very high measurements
  if (h_meas < 0.2 || h_meas > 20) return false;

#if 0
  const Scalar_t h_prev =
      height_hist_.size() > 0 ? height_hist_.back().second : h_meas;

  if (std::abs(h_meas - h_prev) > floor_change_threshold_ / 4.0) {
    ROS_WARN_THROTTLE(1, "Height measurement changing too fast, ignoring!");
    return false;
  }
#endif

  height_hist_.push_back(std::make_pair(time, h_meas));

#if 0
  // Remove old height estimates
  while ((time - height_hist_[0].first).toSec() > height_hist_duration_) {
    height_hist_.pop_front();
  }
#else
  // Remove old height estimates
  while (height_hist_.size() > height_hist_length_) {
    height_hist_.pop_front();
  }
#endif

  // h_pred = expected height above current floor
  // h_ref = height above current floor
  // const Scalar_t h_pred = z_pred(0) * std::cos(xa_(7)) * std::cos(xa_(6));
  // const Scalar_t h_pred = xa_(2) - curr_floor_height_;
  const Scalar_t h_ref = height_hist_[0].second;
  // ROS_INFO_STREAM("z: " << z(0) << ", h_meas: " << h_meas
  //                      << ", h_ref: " << h_ref);
  if (std::abs(h_ref - h_meas) >= floor_change_threshold_) {
    ROS_WARN("----- Floor Level Changed -----");
    height_hist_ = decltype(height_hist_)();  // Clear queue
    height_hist_.push_back(std::make_pair(time, h_meas));

    curr_floor_height_ += h_ref - h_meas;

    // Merge floor heights which are close to each other
    bool set_to_known_floor_height = false;
    for (const auto &floor_height : known_floor_heights_) {
      if (std::abs(curr_floor_height_ - floor_height) <
          floor_merge_threshold_) {
        ROS_WARN_STREAM("Setting current floor height to known floor height: "
                        << floor_height);
        curr_floor_height_ = floor_height;
        set_to_known_floor_height = true;
        break;
      }
    }
    if (!set_to_known_floor_height)
      known_floor_heights_.push_back(curr_floor_height_);

    ROS_WARN("New floor height: %f", curr_floor_height_);

    // Recalculate z_pred
    for (unsigned int k = 0; k < Xa.cols(); k++)
      Za.col(k) = MeasurementModelHeight(Xa.col(k));

    z_pred =
        wm_.replicate<meas_height_count_, 1>().cwiseProduct(Za).rowwise().sum();
  }

  // ROS_INFO("z_pred: %f, z_actual: %f", z_pred(0), z(0));
  // Covariance
  MeasHeightCov Pzz;
  Mat<state_count_, meas_height_count_> Pxz;
  Pzz.setZero();
  Pxz.setZero();
  for (unsigned int k = 0; k < Xa.cols(); k++) {
    const MeasHeightVec d_z = Za.col(k) - z_pred;
    const StateVec d_x = Xa.col(k) - xa_;
    Pzz.noalias() += wc_(k) * d_z * d_z.transpose();
    Pxz.noalias() += wc_(k) * d_x * d_z.transpose();
  }
  Pzz += RnHeight;

  Mat<state_count_, meas_height_count_> K = Pxz * Pzz.inverse();
  xa_ += K * (z - z_pred);
  Pa_ -= K * Pzz * K.transpose();

  return true;
}

FLAUKF::MeasHeightVec FLAUKF::MeasurementModelHeight(const StateVec &x) {
  // d = z/(cos(theta)*cos(phi))
  MeasHeightVec z;
  z(0) = (x(2) - curr_floor_height_) / (std::cos(x(7)) * std::cos(x(6)));
  return z;
}

bool FLAUKF::MeasurementUpdateGps(const MeasGpsVec &z, const MeasGpsCov &RnGps,
                                  const ros::Time &time) {
  // Init
  if (!init_process_ || !init_meas_) {
    std::cout << "MeasurementUpdateGps:" << std::endl;
    for (unsigned int i = 0; i < meas_gps_count_; ++i) {
      xa_(meas_gps_idx_[i]) = z(i);
      std::cout << "z(" << i << "): " << z(i) << std::endl;
    }
    init_meas_ = true;
    return false;
  }

  // Get Measurement
  Mat<meas_gps_count_, state_count_> H;
  H.setZero();
  for (unsigned int i = 0; i < meas_gps_count_; ++i) {
    H(i, meas_gps_idx_[i]) = 1;
  }
  MeasGpsCov S = H * Pa_ * H.transpose() + RnGps;
  // Kalman Gain;
  Mat<state_count_, meas_gps_count_> K = Pa_ * H.transpose() * S.inverse();
  // Innovation
  MeasGpsVec inno = z - H * xa_;
  // Posterior Mean
  xa_ += K * inno;
  // Posterior Covariance
  Pa_ -= K * H * Pa_;

  return true;
}

bool FLAUKF::MeasurementUpdateYaw(const MeasYawVec &z, const MeasYawCov &RnYaw,
                                  const ros::Time &time) {
  // Init
  if (!init_process_ || !init_meas_) {
    std::cout << "MeasurementUpdateYaw:" << std::endl;
    for (unsigned int i = 0; i < meas_yaw_count_; ++i) {
      xa_(meas_yaw_idx_[i]) = z(i);
      std::cout << "z(" << i << "): " << z(i) << std::endl;
    }
    return false;
  }

  // Get Measurement
  Mat<meas_yaw_count_, state_count_> H;
  H.setZero();
  for (unsigned int i = 0; i < meas_yaw_count_; ++i) {
    H(i, meas_yaw_idx_[i]) = 1;
  }
  MeasYawCov S = H * Pa_ * H.transpose() + RnYaw;
  // Kalman Gain;
  Mat<state_count_, meas_yaw_count_> K = Pa_ * H.transpose() * S.inverse();
  // Innovation
  MeasYawVec inno = z - H * xa_;
  // Handle yaw angle jumps
  inno(0) = angles::normalize_angle(inno(0));
  // Posterior Mean
  xa_ += K * inno;
  // Posterior Covariance
  Pa_ -= K * H * Pa_;

  return true;
}

bool FLAUKF::MeasurementUpdateVio(const MeasVioVec &z, const MeasVioCov &RnVio,
                                  const ros::Time &time) {
  // Init
  if (!init_process_ || !init_meas_) {
    std::cout << "MeasurementUpdateVio:" << std::endl;
    std::cout << "z:" << z.transpose() << std::endl;
    for (unsigned int i = 0; i < meas_vio_count_; ++i) {
      xa_(meas_vio_idx_[i]) = z(i);
    }
    init_meas_ = true;
    return false;
  }

#if ENABLE_VIO_YAW_OFFSET  // Non-linear update
  // Generate sigma points
  constexpr unsigned int L = state_count_;
  const Mat<L, 2 *L + 1> Xaa = GenerateSigmaPoints(Mat<0, 0>::Zero());
  // Extract state and noise from augmented state
  const Mat<state_count_, 2 *L + 1> Xa = Xaa.topRows<state_count_>();
  Mat<meas_vio_count_, 2 * L + 1> Za;
  // Mean
  for (unsigned int k = 0; k < Xa.cols(); k++)
    Za.col(k) = MeasurementModelVio(Xa.col(k));

  MeasVioVec z_pred =
      wm_.replicate<meas_vio_count_, 1>().cwiseProduct(Za).rowwise().sum();

  // Covariance
  MeasVioCov Pzz;
  Mat<state_count_, meas_vio_count_> Pxz;
  Pzz.setZero();
  Pxz.setZero();
  for (unsigned int k = 0; k < Xa.cols(); k++) {
    const MeasVioVec d_z = Za.col(k) - z_pred;
    const StateVec d_x = Xa.col(k) - xa_;
    Pzz.noalias() += wc_(k) * d_z * d_z.transpose();
    Pxz.noalias() += wc_(k) * d_x * d_z.transpose();
  }
  Pzz += RnVio;

  const Mat<state_count_, meas_vio_count_> K = Pxz * Pzz.inverse();

  // Calculate innovation
  MeasVioVec inno = z - z_pred;

  // Take proper angle difference
#if VIO_NO_POSITION
  inno(3) = angles::normalize_angle(inno(3));
  inno(4) = angles::normalize_angle(inno(4));
  inno(5) = angles::normalize_angle(inno(5));
#else
  inno(6) = angles::normalize_angle(inno(6));
  inno(7) = angles::normalize_angle(inno(7));
  inno(8) = angles::normalize_angle(inno(8));
#endif
#if 0
  const Vec<3> rpy_pred = z_pred.segment<3>(6);
  const Vec<3> rpy_meas = z.segment<3>(6);
  const auto R_pred = rpy_to_R(rpy_pred);
  const auto R_meas = rpy_to_R(rpy_meas);
  const auto delta_R = R_pred.transpose() * R_meas;
  const auto delta_rpy = R_to_rpy(delta_R);
  inno.segment<3>(6) = delta_rpy;
  // std::cout << "Inno: " << inno.segment<3>(6).transpose() << std::endl;
#endif

  // Posterior Mean
  xa_ += K * inno;
  // Posterior Covariance
  Pa_ -= K * Pzz * K.transpose();

#else  // Linear update
  Mat<meas_vio_count_, state_count_> H;
  H.setZero();
  for (unsigned int i = 0; i < meas_vio_count_; ++i) {
    H(i, meas_vio_idx_[i]) = 1;
  }

  const MeasVioCov S = H * Pa_ * H.transpose() + RnVio;

  // Kalman Gain
  const Mat<state_count_, meas_vio_count_> K =
      Pa_ * H.transpose() * S.inverse();

  // Innovation
  const MeasVioVec z_pred = H * xa_;
  MeasVioVec inno = z - z_pred;

  // Take proper angle difference
#if VIO_NO_POSITION
  inno(3) = angles::normalize_angle(inno(3));
  inno(4) = angles::normalize_angle(inno(4));
  inno(5) = angles::normalize_angle(inno(5));
#else
  inno(6) = angles::normalize_angle(inno(6));
  inno(7) = angles::normalize_angle(inno(7));
  inno(8) = angles::normalize_angle(inno(8));
#endif
#if 0
  const Vec<3> rpy_pred = z_pred.segment<3>(6);
  const Vec<3> rpy_meas = z.segment<3>(6);
  const auto R_pred = rpy_to_R(rpy_pred);
  const auto R_meas = rpy_to_R(rpy_meas);
  const auto delta_R = R_pred.transpose() * R_meas;
  const auto delta_rpy = R_to_rpy(delta_R);
  inno.segment<3>(6) = delta_rpy;
  // std::cout << "Inno: " << inno.segment<3>(6).transpose() << std::endl;
#endif

  // Posterior Mean
  xa_ += K * inno;
  // Posterior Covariance
  Pa_ -= K * H * Pa_;

#endif

  return true;
}

#if ENABLE_VIO_YAW_OFFSET
FLAUKF::MeasVioVec FLAUKF::MeasurementModelVio(const StateVec &x) {
  MeasVioVec z;

  auto const vio_yaw_offset = x(15);
  auto const R_vio_yaw =
      Eigen::AngleAxis<Scalar_t>(vio_yaw_offset, Vec<3>::UnitZ());

#if VIO_NO_POSITION
  z.segment<3>(0) = R_vio_yaw * x.segment<3>(3);
  z.segment<3>(3) = x.segment<3>(6);
  z(5) += vio_yaw_offset;
#else
  z.segment<3>(0) = R_vio_yaw * x.segment<3>(0);
  z.segment<3>(3) = R_vio_yaw * x.segment<3>(3);
  z.segment<3>(6) = x.segment<3>(6);
  z(8) += vio_yaw_offset;
#endif

  return z;
}
#endif
