#include <benchmark/benchmark.h>
#include "mapper/voxel_mapper.h"

/**
 * @brief Arguments represent the x-dim, y-dim, and z-dim size respectively. The
 * fourth argument is the resolution multiplied by 100 because arguments are
 * signed long.
 */
static void BM_GetInflatedMap(benchmark::State& state) {
  int x_dim = state.range(0);
  int y_dim = state.range(1);
  int z_dim = state.range(2);
  int x_origin = - x_dim / 2;
  int y_origin = - y_dim / 2;
  int z_origin = - z_dim / 2;
  double resolution = static_cast<double>(state.range(3)) / 100.0;
  int8_t val_default = 0;
  int decay_times = 30;
  Eigen::Vector3d origin(x_origin, y_origin, z_origin);
  Eigen::Vector3d dimensions(x_dim, y_dim, z_dim);
  mapper::VoxelMapper test_mapper(origin, dimensions,
                                  resolution, val_default,
                                  decay_times);

  for (auto _ : state) {
    test_mapper.getInflatedMap();
  }
}


/**
 * @brief Arguments represent the x-dim, y-dim, and z-dim size respectively. The
 * fourth argument is the resolution multiplied by 100 because arguments are
 * signed long.
 */
static void BM_DecayLocalCloud(benchmark::State& state) {
  int x_dim = state.range(0);
  int y_dim = state.range(1);
  int z_dim = state.range(2);
  int x_origin = - x_dim / 2;
  int y_origin = - y_dim / 2;
  int z_origin = - z_dim / 2;
  double resolution = static_cast<double>(state.range(3)) / 100.0;
  int8_t val_default = 0;
  int decay_times = 30;
  Eigen::Vector3d origin(x_origin, y_origin, z_origin);
  Eigen::Vector3d dimensions(x_dim, y_dim, z_dim);
  mapper::VoxelMapper test_mapper(origin, dimensions,
                                  resolution, val_default,
                                  decay_times);

  Eigen::Vector3d position(0, 0, 0);
  double max_range = 9.8;

  for (auto _ : state) {
    test_mapper.decayLocalCloud(position, max_range);
  }
}


/**
 * @brief Arguments represent the x-dim, y-dim, and z-dim size respectively. The
 * fourth argument is the resolution multiplied by 100 because arguments are
 * signed long.
 */
static void BM_AddCloud(benchmark::State& state) {
  int x_dim = state.range(0);
  int y_dim = state.range(1);
  int z_dim = state.range(2);
  int x_origin = - x_dim / 2;
  int y_origin = - y_dim / 2;
  int z_origin = - z_dim / 2;
  double resolution = static_cast<double>(state.range(3)) / 100.0;
  int8_t val_default = 0;
  int decay_times = 30;
  Eigen::Vector3d origin(x_origin, y_origin, z_origin);
  Eigen::Vector3d dimensions(x_dim, y_dim, z_dim);

  // Create the voxel mapper object
  mapper::VoxelMapper test_mapper(origin, dimensions,
                                  resolution, val_default,
                                  decay_times);

  // We will consider that the lidar is at position (-10, -5, -1) in the map
  // frame of reference
  Eigen::Vector3d lidar_pos(-10, -5, -1);
  Eigen::Affine3d t_map_lidar = Eigen::Translation3d(-10, -5, -1) *
                                Eigen::AngleAxisd(0,
                                                  Eigen::Vector3d(0, 0, 0));
  double max_range = 30.0;
  mapper::vec_Vec3d lidar_points;

  // Add 3 points for all voxels in the range -30 to 30 in all 3 axes. Some
  // will be discarded and only a sperical shape of voxels around
  // (-10, -5, -1) should be marked as occupied
  for (int x = 0; x <= 120; x++) {
      for (int y = 0; y <= 120; y++) {
          for (int z = 0; z <= 120; z++) {
              double real_x = -30 + x * 0.5;
              double real_y = -30 + y * 0.5;
              double real_z = -30 + z * 0.5;
              Eigen::Vector3d point(real_x, real_y, real_z);
              lidar_points.push_back(point);
          }
      }
  }

  // Current neighboring voxels are just +-1 in x and y directions
  mapper::vec_Vec3i neighbors;
  neighbors.push_back(Eigen::Vector3i(-1, 0, 0));
  neighbors.push_back(Eigen::Vector3i(1, 0, 0));
  neighbors.push_back(Eigen::Vector3i(0, -1, 0));
  neighbors.push_back(Eigen::Vector3i(0, 1, 0));

  for (auto _ : state) {
    test_mapper.addCloud(lidar_points, t_map_lidar, neighbors, false,
                         max_range);
  }
}


/**
 * @brief Arguments represent the x-dim, y-dim, and z-dim size respectively. The
 * fourth argument is the resolution multiplied by 100 because arguments are
 * signed long.
 */
static void BM_GetInflatedLocalMap(benchmark::State& state) {
  int x_dim = state.range(0);
  int y_dim = state.range(1);
  int z_dim = state.range(2);
  int x_origin = - x_dim / 2;
  int y_origin = - y_dim / 2;
  int z_origin = - z_dim / 2;
  double resolution = static_cast<double>(state.range(3)) / 100.0;
  int8_t val_default = 0;
  int decay_times = 30;
  Eigen::Vector3d origin(x_origin, y_origin, z_origin);
  Eigen::Vector3d dimensions(x_dim, y_dim, z_dim);
  mapper::VoxelMapper test_mapper(origin, dimensions,
                                  resolution, val_default,
                                  decay_times);

  Eigen::Vector3d local_origin(75, 25, 2.5);
  Eigen::Vector3d local_dimensions(100, 100, 10);

  for (auto _ : state) {
    test_mapper.getInflatedLocalMap(local_origin, local_dimensions);
  }
}


BENCHMARK(BM_GetInflatedMap)
  ->Args({200, 200, 10, 50})
  ->Args({200, 200, 10, 25})
  ->Args({200, 200, 10, 10});

BENCHMARK(BM_DecayLocalCloud)
  ->Args({200, 200, 10, 50})
  ->Args({200, 200, 10, 25})
  ->Args({200, 200, 10, 10});

BENCHMARK(BM_AddCloud)
  ->Args({200, 200, 10, 50})
  ->Args({200, 200, 10, 25})
  ->Args({200, 200, 10, 10});

BENCHMARK(BM_GetInflatedLocalMap)
  ->Args({200, 200, 10, 50})
  ->Args({200, 200, 10, 25})
  ->Args({200, 200, 10, 10});


BENCHMARK_MAIN();


