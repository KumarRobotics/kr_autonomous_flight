#include <benchmark/benchmark.h>
#include <mpl_collision/map_util.h>

// Create a fixture to share the same setup among all benchmarking tests
class BenchmarkMapUtil : public benchmark::Fixture {
 public:
  void SetUp(const ::benchmark::State& state) {
    Vec3f origin(-100, -100, -5);
    Vec3i dim(400, 400, 20);
    double resolution = 0.5;

    // Initialize the map with val_free as the default value
    std::vector<signed char> base_map(dim(0) * dim(1) * dim(2),
                                      kr_planning_msgs::VoxelMap::val_free);

    /* Modify the values of some voxels to not have a uniform map */

    // First set a region of voxels to be occupied in the center of the map.
    for (int z = 5; z < 15; z++) {
      for (int y = 25; y < 375; y++) {
        for (int x = 25; x < 375; x++) {
          int idx = x + dim(0) * y + dim(0) * dim(1) * z;
          base_map[idx] = kr_planning_msgs::VoxelMap::val_occ;
        }
      }
    }

    // Then set a smaller region of voxels inside the previous region as unknown
    for (int z = 7; z < 13; z++) {
      for (int y = 75; y < 325; y++) {
        for (int x = 75; x < 325; x++) {
          int idx = x + dim(0) * y + dim(0) * dim(1) * z;
          base_map[idx] = kr_planning_msgs::VoxelMap::val_unknown;
        }
      }
    }

    // Initialize the MapUtil object with the previously created map
    mapper_.setMap(origin, dim, base_map, resolution);

    /** Now populate the vectors of points and voxels. The amount of points
     * should correspond to the amount of voxels. There should be some points
     * that lie outside the bounds of the map as well to make sure that the
     * isOutside function is used.
     */

    for (int x = -50; x < 450; x++) {
      for (int y = -50; y < 450; y++) {
        for (int z = -10; z < 30; z++) {
          test_voxels_.push_back(Vec3i(x, y, z));
          double real_x = x * 0.5 + 0.25;
          double real_y = y * 0.5 + 0.25;
          double real_z = z * 0.5 + 0.25;
          test_cloud_.push_back(Vec3f(real_x, real_y, real_z));
        }
      }
    }
  }

  MPL::VoxelMapUtil mapper_;
  vec_Vec3i test_voxels_;
  vec_Vec3f test_cloud_;
};

// Define benchmarks
BENCHMARK_DEFINE_F(BenchmarkMapUtil, BM_isOutside)(benchmark::State& state) {
  for (auto _ : state) {
    // Counter is used to add some dummy code to avoid the function call from
    // being optimized out
    int counter = 0;

    for (auto &voxel : test_voxels_) {
      bool it_is = mapper_.isOutside(voxel);

      if (it_is) {
        counter++;
      }
    }

    if (counter == 0) {
      std::cout << "This should not have been printed" << std::endl;
    }
  }
}

BENCHMARK_DEFINE_F(BenchmarkMapUtil, BM_isFree)(benchmark::State& state) {
  for (auto _ : state) {
    // Counter is used to add some dummy code to avoid the function call from
    // being optimized out
    int counter = 0;

    for (auto &voxel : test_voxels_) {
      bool it_is = mapper_.isFree(voxel);

      if (it_is) {
        counter++;
      }
    }

    if (counter == 0) {
      std::cout << "This should not have been printed" << std::endl;
    }
  }
}

BENCHMARK_DEFINE_F(BenchmarkMapUtil, BM_isOccupied)(benchmark::State& state) {
  for (auto _ : state) {
    // Counter is used to add some dummy code to avoid the function call from
    // being optimized out
    int counter = 0;

    for (auto &voxel : test_voxels_) {
      bool it_is = mapper_.isOccupied(voxel);

      if (it_is) {
        counter++;
      }
    }

    if (counter == 0) {
      std::cout << "This should not have been printed" << std::endl;
    }
  }
}

BENCHMARK_DEFINE_F(BenchmarkMapUtil, BM_isUnknown)(benchmark::State& state) {
  for (auto _ : state) {
    // Counter is used to add some dummy code to avoid the function call from
    // being optimized out
    int counter = 0;

    for (auto &voxel : test_voxels_) {
      bool it_is = mapper_.isUnknown(voxel);

      if (it_is) {
        counter++;
      }
    }

    if (counter == 0) {
      std::cout << "This should not have been printed" << std::endl;
    }
  }
}

BENCHMARK_DEFINE_F(BenchmarkMapUtil, BM_floatToInt)(benchmark::State& state) {
  for (auto _ : state) {
    for (auto &point : test_cloud_) {
      Vec3i index = mapper_.floatToInt(point);

      // Add some dummy conditional to avoid the function call from being
      // optimized out
      if (index(0) == -100000 && index(1) == -100000 && index(2) == -100000) {
        std::cout << "This should not have been printed" << std::endl;
      }
    }
  }
}

BENCHMARK_DEFINE_F(BenchmarkMapUtil, BM_intToFloat)(benchmark::State& state) {
  for (auto _ : state) {
    for (auto &voxel : test_voxels_) {
      Vec3f point = mapper_.intToFloat(voxel);

      // Add some dummy conditional to avoid the function call from being
      // optimized out
      if (point(0) == -500 && point(1) == -500 && point(2) == -500) {
        std::cout << "This should not have been printed" << std::endl;
      }
    }
  }
}

BENCHMARK_DEFINE_F(BenchmarkMapUtil, BM_rayTrace)(benchmark::State& state) {
  vec_Vec3f start_positions;
  vec_Vec3f end_positions;

  start_positions.push_back(Vec3f(99, 77, 4));
  start_positions.push_back(Vec3f(-80, 20, 1));
  start_positions.push_back(Vec3f(-50, -50, 4));

  end_positions.push_back(Vec3f(50, 50, 4));
  end_positions.push_back(Vec3f(69, 18, -4));
  end_positions.push_back(Vec3f(-10, 99, 3));

  for (auto _ : state) {
    for (auto &start : start_positions) {
      for (auto &end : end_positions) {
        vec_Vec3i voxels = mapper_.rayTrace(start, end);
        if (voxels.size() == -1) {
          std::cout << "This message should not have been printed" << std::endl;
        }
      }
    }
  }
}

BENCHMARK_DEFINE_F(BenchmarkMapUtil, BM_getCloud)(benchmark::State& state) {
  for (auto _ : state) {
    vec_Vec3f cloud = mapper_.getCloud();
  }
}

BENCHMARK_DEFINE_F(BenchmarkMapUtil, BM_getFreeCloud)(benchmark::State& state) {
  for (auto _ : state) {
    vec_Vec3f cloud = mapper_.getFreeCloud();
  }
}

BENCHMARK_DEFINE_F(BenchmarkMapUtil, BM_getUnknown)(benchmark::State& state) {
  for (auto _ : state) {
    vec_Vec3f cloud = mapper_.getUnknownCloud();
  }
}

// Register Benchmarks
BENCHMARK_REGISTER_F(BenchmarkMapUtil, BM_isOutside);
BENCHMARK_REGISTER_F(BenchmarkMapUtil, BM_isFree);
BENCHMARK_REGISTER_F(BenchmarkMapUtil, BM_isOccupied);
BENCHMARK_REGISTER_F(BenchmarkMapUtil, BM_isUnknown);
BENCHMARK_REGISTER_F(BenchmarkMapUtil, BM_floatToInt);
BENCHMARK_REGISTER_F(BenchmarkMapUtil, BM_intToFloat);
BENCHMARK_REGISTER_F(BenchmarkMapUtil, BM_rayTrace);
BENCHMARK_REGISTER_F(BenchmarkMapUtil, BM_getCloud);
BENCHMARK_REGISTER_F(BenchmarkMapUtil, BM_getFreeCloud);
BENCHMARK_REGISTER_F(BenchmarkMapUtil, BM_getUnknown);

BENCHMARK_MAIN();


