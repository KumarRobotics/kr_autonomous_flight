// Copyright 2016 KumarRobotics - Kartik Mohta
#ifndef ESTIMATION_PKGS_FLA_UKF_INCLUDE_FLA_UKF_FLA_UKF_H_
#define ESTIMATION_PKGS_FLA_UKF_INCLUDE_FLA_UKF_FLA_UKF_H_

#include <Eigen/Core>
#include <ros/ros.h>
#include <array>
#include <deque>
#include <utility>
#include <vector>

class FLAUKF {
 public:
  using Scalar_t = double;

  template <int N>
  using Vec = Eigen::Matrix<Scalar_t, N, 1>;

  template <int N>
  using RowVec = Eigen::Matrix<Scalar_t, 1, N>;

  template <int N, int M>
  using Mat = Eigen::Matrix<Scalar_t, N, M>;

  // Dimensions
  // State: [pos, vel, rot_rpy, bias_acc, bias_gyro, vio_yaw_offset]
  static constexpr unsigned int state_count_ = 16;
  static constexpr unsigned int proc_noise_count_ = 13;
  static constexpr unsigned int input_count_ = 6;

  static constexpr std::array<unsigned int, 6> meas_cam_idx_{
      {0, 1, 2, 6, 7, 8}};
  static constexpr unsigned int meas_cam_count_ = meas_cam_idx_.size();

  static constexpr std::array<unsigned int, 3> meas_laser_idx_{{0, 1, 8}};
  static constexpr unsigned int meas_laser_count_ = meas_laser_idx_.size();

  static constexpr std::array<unsigned int, 1> meas_height_idx_{{2}};
  static constexpr unsigned int meas_height_count_ = meas_height_idx_.size();

  static constexpr std::array<unsigned int, 6> meas_gps_idx_{
      {0, 1, 2, 3, 4, 5}};
  static constexpr unsigned int meas_gps_count_ = meas_gps_idx_.size();

  static constexpr std::array<unsigned int, 1> meas_yaw_idx_{{8}};
  static constexpr unsigned int meas_yaw_count_ = meas_yaw_idx_.size();

#if VIO_NO_POSITION
  static constexpr std::array<unsigned int, 6> meas_vio_idx_{
      {3, 4, 5, 6, 7, 8}};
  static constexpr unsigned int meas_vio_count_ = meas_vio_idx_.size();
#else
  static constexpr std::array<unsigned int, 9> meas_vio_idx_{
      {0, 1, 2, 3, 4, 5, 6, 7, 8}};
  static constexpr unsigned int meas_vio_count_ = meas_vio_idx_.size();
#endif

  using StateVec = Vec<state_count_>;
  using StateCov = Mat<state_count_, state_count_>;
  using InputVec = Vec<input_count_>;
  using ProcNoiseVec = Vec<proc_noise_count_>;
  using ProcNoiseCov = Mat<proc_noise_count_, proc_noise_count_>;
  using MeasCamVec = Vec<meas_cam_count_>;
  using MeasCamCov = Mat<meas_cam_count_, meas_cam_count_>;
  using MeasLaserVec = Vec<meas_laser_count_>;
  using MeasLaserCov = Mat<meas_laser_count_, meas_laser_count_>;
  using MeasHeightVec = Vec<meas_height_count_>;
  using MeasHeightCov = Mat<meas_height_count_, meas_height_count_>;
  using MeasGpsVec = Vec<meas_gps_count_>;
  using MeasGpsCov = Mat<meas_gps_count_, meas_gps_count_>;
  using MeasYawVec = Vec<meas_yaw_count_>;
  using MeasYawCov = Mat<meas_yaw_count_, meas_yaw_count_>;
  using MeasVioVec = Vec<meas_vio_count_>;
  using MeasVioCov = Mat<meas_vio_count_, meas_vio_count_>;

  FLAUKF();

  const StateVec &GetState() const;
  const StateCov &GetStateCovariance() const;

  void SetGravity(Scalar_t _g);
  void SetImuCovariance(const ProcNoiseCov &_Rv);
  void SetParameters(Scalar_t _alpha, Scalar_t _beta, Scalar_t _kappa);
  void SetCurrentFloorHeight(Scalar_t h);
  void SetFloorChangeThreshold(Scalar_t t);
  void SetFloorMergeThreshold(Scalar_t t);

  bool ProcessUpdate(const InputVec &u, const ros::Time &time);

  bool MeasurementUpdateCam(const MeasCamVec &z, const MeasCamCov &RnCam,
                            const ros::Time &time);
  MeasCamVec MeasurementModelCam(const StateVec &x);

  bool MeasurementUpdateLaser(const MeasLaserVec &z,
                              const MeasLaserCov &RnLaser,
                              const ros::Time &time);
  MeasLaserVec MeasurementModelLaser(const StateVec &x);

  bool MeasurementUpdateHeight(const MeasHeightVec &z,
                               const MeasHeightCov &RnHeight,
                               const ros::Time &time);
  MeasHeightVec MeasurementModelHeight(const StateVec &x);

  bool MeasurementUpdateGps(const MeasGpsVec &z, const MeasGpsCov &RnGps,
                            const ros::Time &time);

  bool MeasurementUpdateYaw(const MeasYawVec &z, const MeasYawCov &RnYaw,
                            const ros::Time &time);

  bool MeasurementUpdateVio(const MeasVioVec &z, const MeasVioCov &RnVio,
                            const ros::Time &time);
  MeasVioVec MeasurementModelVio(const StateVec &x);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 private:
  // Private functions
  void GenerateWeights(unsigned int L);

  /* Generate the sigma points given the noise covariance matrix
   * param[in] Rn Noise covariance matrix used for generating the sigma points
   */
  template <typename T>
  Mat<state_count_ + T::RowsAtCompileTime,
      2 * (state_count_ + T::RowsAtCompileTime) + 1>
  GenerateSigmaPoints(const T &Rn) {
    constexpr int noise_count = T::RowsAtCompileTime;
    constexpr int L = state_count_ + noise_count;

    GenerateWeights(L);

    // Expand state
    Vec<L> xaa = Vec<L>::Zero();
    xaa.template topRows<state_count_>() = xa_;
    Mat<L, L> Paa = Mat<L, L>::Zero();
    Paa.template block<state_count_, state_count_>(0, 0) = Pa_;
    if (noise_count > 0)
      Paa.template block<noise_count, noise_count>(state_count_, state_count_) =
        Rn;

    // Matrix square root
    Mat<L, L> sqrtPaa = Paa.llt().matrixL();

    Mat<L, 2 *L + 1> Xaa = xaa.template replicate<1, 2 * L + 1>();
    Xaa.template block<L, L>(0, 1).noalias() += gamma_ * sqrtPaa;
    Xaa.template block<L, L>(0, L + 1).noalias() -= gamma_ * sqrtPaa;
    return Xaa;
  }

  StateVec ProcessModel(const StateVec &x, const InputVec &u,
                        const ProcNoiseVec &w, Scalar_t dt);
  bool PropagateAprioriCovariance(const ros::Time time);

  // State
  StateVec xa_;
  StateCov Pa_;
  ros::Time prev_proc_update_time_;

  // Initial process update indicator
  bool init_process_;
  bool init_meas_;

  // Process Covariance Matrix
  ProcNoiseCov Rv_;

  // Gravity
  Scalar_t g_;

  // UKF Parameters
  Scalar_t alpha_;
  Scalar_t beta_;
  Scalar_t kappa_;
  Scalar_t lambda_;
  Scalar_t gamma_;

  Mat<1, Eigen::Dynamic> wm_;
  Mat<1, Eigen::Dynamic> wc_;

  Scalar_t curr_floor_height_;
  Scalar_t floor_change_threshold_;
  std::vector<Scalar_t> known_floor_heights_;
  Scalar_t floor_merge_threshold_;
  std::deque<std::pair<ros::Time, Scalar_t>> height_hist_;
#if 0
  double height_hist_duration_;
#else
  double height_hist_length_;
#endif
};

#endif // ESTIMATION_PKGS_FLA_UKF_INCLUDE_FLA_UKF_FLA_UKF_H_
