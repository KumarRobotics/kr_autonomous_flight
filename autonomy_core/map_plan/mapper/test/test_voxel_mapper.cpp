#include <gtest/gtest.h>
#include <kr_planning_msgs/VoxelMap.h>
#include <iostream>
#include <cmath>
#include <memory>
#include "mapper/voxel_mapper.h"
#include "mpl_collision/map_util.h"

/**
 * @brief This function is passed as a parameter to the sort method so that
 * points in a point cloud can be sorted. Points are first compared by their
 * first axis and consequently compared through their next axis if the previous
 * are equal
 * @param first : first element to compare
 * @param second : second element to compare
 * @return returns true if the first parameter is smaller than the second
 * and false otherwise
 */
bool comparePoints(Eigen::Vector3d first, Eigen::Vector3d second) {
  const double epsilon = 0.000001;

  if (fabs(first[0] - second[0]) < epsilon) {
    // Index 0 is the same so check next index
    if (fabs(first[1] - second[1]) < epsilon) {
      // Index 1 is the same so check the last index
      if (fabs(first[2] - second[2]) < epsilon) {
        // All elements are equal
        return false;
      } else {
        // Points differ at index 2 so compare them normally
        return first[2] < second[2];
      }
    } else {
      // Points differ at index 1 so compare them normally
      return first[1] < second[1];
    }
  } else {
    // They are not equal so compare them normally
    return first[0] < second[0];
  }
}


// Test class needs to be in the same namespace as VoxelMapper to be able to
// make it a friend class
namespace mapper {

/** Declaration of fixture class to use the same configuration across tests **/
class VoxelMapperTest : public testing::Test {
 protected:
  void SetUp() override;
  Eigen::Vector3i getVoxel(int index);

  // The origin is the most negative corner
  double x_origin_;
  double y_origin_;
  double z_origin_;
  // Dimensions along each axis (NOT amount of voxels in each axis)
  int x_dim_;
  int y_dim_;
  int z_dim_;
  double resolution_;
  int8_t val_default_;
  int8_t val_occ_;
  int8_t val_even_;
  int8_t val_add_;
  int8_t val_unknown_;
  int8_t val_free_;
  int8_t val_decay_;
  int decay_times_;

  vec_Vec3d gt_cloud_;     // Ground truth point cloud for some tests

  // This is the VoxelMapper object that is used for all the tests
  std::unique_ptr<mapper::VoxelMapper> p_test_mapper_;
};

/**
 * @brief Used for debugging purposes. When a test fails this function can be
 * used to retrieve the corresponding voxel index from a linear indexing value
 * @param index Index of voxel stored in contiguous memory
 * @return Eigen::Vector3i : Voxel index
 */
Eigen::Vector3i VoxelMapperTest::getVoxel(int index) {
  Eigen::Vector3i voxel;
  int x_voxels = x_dim_ / resolution_;
  int y_voxels = y_dim_ / resolution_;
  voxel[2] = index / (x_voxels * y_voxels);
  index = index % (x_voxels * y_voxels);
  voxel[1] = index / x_voxels;
  voxel[0] = index % x_voxels;

  return voxel;
}

void VoxelMapperTest::SetUp() {
  x_origin_ = -100;
  y_origin_ = -100;
  z_origin_ = -5;
  x_dim_ = 200;
  y_dim_ = 200;
  z_dim_ = 10;
  resolution_ = 0.5;
  val_default_ = 0;
  decay_times_ = 30;
  Eigen::Vector3d origin(x_origin_, y_origin_, z_origin_);
  Eigen::Vector3d dimensions(x_dim_, y_dim_, z_dim_);
  p_test_mapper_.reset( new mapper::VoxelMapper(origin, dimensions,
                                                resolution_, val_default_,
                                                decay_times_));
  val_occ_        = p_test_mapper_->val_occ_;
  val_even_       = p_test_mapper_->val_even_;
  val_add_        = p_test_mapper_->val_add_;
  val_unknown_    = p_test_mapper_->val_unknown_;
  val_free_       = p_test_mapper_->val_free_;
  val_decay_      = p_test_mapper_->val_decay_;

  // Fill the cloud with some random known points
  gt_cloud_.push_back(Eigen::Vector3d(0.25, 0.25, 0.25));
  gt_cloud_.push_back(Eigen::Vector3d(0.75, 0.75, 1.75));
  gt_cloud_.push_back(Eigen::Vector3d(-28.25, -12.25, 3.25));
  gt_cloud_.push_back(Eigen::Vector3d(20.25, 66.25, 4.25));
  gt_cloud_.push_back(Eigen::Vector3d(20.25, -49.25, 4.25));
  gt_cloud_.push_back(Eigen::Vector3d(85.25, -61.25, 2.25));
  gt_cloud_.push_back(Eigen::Vector3d(94.25, -76.25, -3.25));
  gt_cloud_.push_back(Eigen::Vector3d(-64.25, 58.25, -4.75));
  gt_cloud_.push_back(Eigen::Vector3d(34.25, 34.25, -3.25));
  gt_cloud_.push_back(Eigen::Vector3d(79.25, 12.25, -2.25));

  // Cloud is sorted because it should not matter in what order the points
  // in point clouds returned by class methods
  std::sort(gt_cloud_.begin(), gt_cloud_.end(), comparePoints);
}


/*** Defining Tests ***/

/**
 * @brief The allocate() method is able to relocate the current map into another
 * part of the world, but this functionality is currently NOT being used since
 * allocate() is only called in the constructor of the VoxelMapper class. The
 * map is stored in the map_ data member and we can access it directly via
 * friendship to verify the correct initialization of allocate(), but this test
 * should not depend on the data type of the map_ variable; for this reason, we
 * use the getMap() method to check the status of the map. In the following
 * assertions, we check that the map is created with the correct dimensions.
 */
TEST_F(VoxelMapperTest, TestAllocateDimensions) {
  // Creating voxel mapper object with same parameters as global_voxel_mapper
  // in source file: local_global_mapper.cpp
  Eigen::Vector3d origin(-100, -100, -5);
  Eigen::Vector3d dimensions(200, 200, 10);
  p_test_mapper_.reset(new mapper::VoxelMapper(origin, dimensions, 0.5, 0, 30));
  EXPECT_EQ(p_test_mapper_->getMap().data.size(), 3200000);

  // Creating voxel mapper object with same parameters as storage_voxel_mapper
  // in source file: local_global_mapper.cpp
  origin = Eigen::Vector3d(-100, -100, -5);
  dimensions = Eigen::Vector3d(200, 200, 12);
  p_test_mapper_.reset(new mapper::VoxelMapper(origin, dimensions,
                                                0.25, 0, 30));
  EXPECT_EQ(p_test_mapper_->getMap().data.size(), 30720000);

  // Creating voxel mapper object with custom parameters
  origin = Eigen::Vector3d(-5, -5, -5);
  dimensions = Eigen::Vector3d(10, 10, 10);
  p_test_mapper_.reset(new mapper::VoxelMapper(origin, dimensions, 0.1, 0, 30));
  EXPECT_EQ(p_test_mapper_->getMap().data.size(), 1000000);
}

/**
 * @brief Similar to TestAllocateDimensions but here we test that the relocating
 * of the map is working properly. When relocating the map to a new area in
 * the world, the new voxels are initialized to the default value, but the
 * overlapping voxels retain their previous values. So we first create a map and
 * mark all of its voxels as occupied. Then we relocate the map so that it
 * partially overlaps with its orginal position. The voxels that are overlapped
 * should be kept as occupied and the new voxels should have the defualt value.
 */
TEST_F(VoxelMapperTest, TestAllocateRelocate) {
  // Relocating should not happen if origin and dimensions are the same as to
  // what they previously were
  Eigen::Vector3d prev_origin(x_origin_, y_origin_, z_origin_);
  Eigen::Vector3d prev_dimensions(x_dim_, y_dim_, z_dim_);
  ASSERT_FALSE(p_test_mapper_->allocate(prev_origin, prev_dimensions));

  // Mark all voxels as occupied. This original map, in world coordinates,
  // ranges from:
  // x: -100 to 100
  // y: -100 to 100
  // z: -5 to 5
  // In voxels this translates to:
  // x: 200 voxels from -100 to 0 and 200 voxels from 0 to 100
  // y: 200 voxels from -100 to 0 and 200 voxels from 0 to 100
  // z: 10 voxels from -5 to 0 and 10 voxels from 0 to 5
  int num_voxels = 3200000;
  std::vector<signed char> base_map(num_voxels, val_occ_);
  Eigen::Vector3d origin(x_origin_, y_origin_, z_origin_);
  Eigen::Vector3i dimensions(x_dim_ / resolution_,
                             y_dim_ / resolution_,
                             z_dim_ / resolution_);
  p_test_mapper_->setMap(origin, dimensions, base_map, resolution_);

  // Relocating map in the positive direction in all three axes with its most
  // negative corner at (0, 0, 0) and reducing x-dim and y-dim by 50. With the
  // same resolution this means 100 less voxels in each dimension.
  // z stays the same
  Eigen::Vector3d new_origin(0, 0, 0);
  Eigen::Vector3d new_dimensions(150, 150, 10);
  // True means that the relocating actually happened
  EXPECT_TRUE(p_test_mapper_->allocate(new_origin, new_dimensions));

  kr_planning_msgs::VoxelMap relocated_map = p_test_mapper_->getMap();

  // Create the ground truth voxel map to compare it to the relocated map
  kr_planning_msgs::VoxelMap gt_voxel_map;

  // new map should have 300 voxels in x, 300 voxels in the y, and 20 in z
  int dim_x = 150 / resolution_;
  int dim_y = 150 / resolution_;
  int dim_z = 10 / resolution_;
  gt_voxel_map.data.resize(dim_x * dim_y * dim_z, val_default_);

  // Sections with overlap between the previous origin-dimensions and the new
  // origin-dimensions should be marked as obstacles while all other voxels
  // should have the default value. In the relocated map, the dimensions range
  // from
  // x: 0 to 150
  // y: 0 to 150
  // z: 0 to 10
  // Given the previous location and that all the voxels there were occupied,
  // the new map has
  // x: The first 200 voxels marked as occupied and the last 100 as default
  // y: The first 200 voxels marked as occupied and the last 100 as default
  // z: The first 10 voxels marked as occupied and the last 10 as default
  // so set that for the ground truth voxel map
  for (int x = 0 ; x < 200; x++) {
    for (int y = 0; y < 200; y++) {
      for (int z = 0; z < 10; z++) {
        int idx = x + dim_x * y + dim_x * dim_y * z;
        gt_voxel_map.data[idx] = val_occ_;
      }
    }
  }

  num_voxels = gt_voxel_map.data.size();
  ASSERT_EQ(relocated_map.data.size(), gt_voxel_map.data.size());

  for (int idx = 0; idx < num_voxels; idx++) {
    auto voxel = getVoxel(idx);
    EXPECT_EQ(relocated_map.data[idx], gt_voxel_map.data[idx])
      << "IDX:" << idx << "\t"
      << "Voxel: [" << voxel[0] << ", " << voxel[1] << ", " << voxel[2]
      << "]" << std::endl;
  }
}

/**
 * @brief In this test we make sure that after calling setMapUnkown, all voxels
 * in the map and the inflated map are marked as unknown
 */
TEST_F(VoxelMapperTest, TestSetMapUknown) {
  p_test_mapper_->setMapUnknown();
  kr_planning_msgs::VoxelMap vox_map = p_test_mapper_->getMap();
  kr_planning_msgs::VoxelMap inflated_vox_map =
                                  p_test_mapper_->getInflatedMap();
  for (auto &voxel : vox_map.data) {
    EXPECT_EQ(voxel, val_unknown_);
  }
  for (auto &voxel : inflated_vox_map.data) {
    EXPECT_EQ(voxel, val_unknown_);
  }
}

/**
 * @brief In this test we make sure that after calling setMapFree, all voxels
 * in the map and the inflated map are marked as free
 */
TEST_F(VoxelMapperTest, TestSetMapFree) {
  p_test_mapper_->setMapFree();
  kr_planning_msgs::VoxelMap vox_map = p_test_mapper_->getMap();
  kr_planning_msgs::VoxelMap inflated_vox_map =
                                  p_test_mapper_->getInflatedMap();
  for (auto &voxel : vox_map.data) {
    EXPECT_EQ(voxel, val_free_);
  }
  for (auto &voxel : inflated_vox_map.data) {
    EXPECT_EQ(voxel, val_free_);
  }
}

/**
 * @brief The decayLocalCloud method decays all the occupied voxels within a
 * local range. So first we mark all the voxels in the map as occupied and call
 * decayLocalCloud multiple times. Since getMap doesn't return the exact value
 * of the voxel, we will call decayLocalCloud until the in-range voxels are
 * equal to val_even (voxels equal to or below this value are considered free).
 * This corresponds to calling decayLocalCloud a total of
 * (val_occ - val_even)/decay_times_ times. Only the voxels that are within
 * range should be decayed and marked as free, while the rest should remain as
 * occupied 
 */
TEST_F(VoxelMapperTest, TestDecayLocalCloud) {
  // Compute how many times decayLocalCloud must be called to free the voxels
  int num_calls = (static_cast<float>(val_occ_)
                   - static_cast<float>(val_even_))
                   / static_cast<float>(val_decay_);

  // Mark all voxels in the map as occupied
  int num_voxels = 3200000;
  std::vector<signed char> base_map(num_voxels, val_occ_);
  Eigen::Vector3d origin(x_origin_, y_origin_, z_origin_);
  Eigen::Vector3i dimensions(x_dim_ / resolution_,
                             y_dim_ / resolution_,
                             z_dim_ / resolution_);
  p_test_mapper_->setMap(origin, dimensions, base_map, resolution_);

  // Now decay the voxels that are in the range of 9.1 in all three axes
  // around (0, 0, 0) in world coordinates. This corresponds to the lidar's
  // position in the map's frame of reference
  Eigen::Vector3d position(0, 0, 0);
  double max_range = 9.1;
  for (int i = 0; i < num_calls; i++) {
    p_test_mapper_->decayLocalCloud(position, max_range);
  }

  kr_planning_msgs::VoxelMap decayed_map = p_test_mapper_->getMap();

  // Create the ground truth voxel map to compare it to the decayed map
  kr_planning_msgs::VoxelMap gt_voxel_map;
  int dim_x = x_dim_ / resolution_;
  int dim_y = y_dim_ / resolution_;
  int dim_z = z_dim_ / resolution_;
  // All voxels should be marked occupied except the decayed voxel locations
  gt_voxel_map.data.resize(dim_x * dim_y * dim_z, val_occ_);

  // Free the voxels that should be decayed
  for (int x = 181; x <= 218; x++) {
    for (int y = 181; y <= 218; y++) {
      for (int z = 0; z <= 19; z++) {
        int idx = x + 400 * y + 400 * 400 * z;
        gt_voxel_map.data[idx] = val_free_;
      }
    }
  }

  // Finally make the comparison
  ASSERT_EQ(decayed_map.data.size(), gt_voxel_map.data.size());
  for (int idx = 0; idx < num_voxels; idx++) {
    auto voxel = getVoxel(idx);
    EXPECT_EQ(decayed_map.data[idx], gt_voxel_map.data[idx])
      << "IDX:" << idx << "\t"
      << "Voxel: [" << voxel[0] << ", " << voxel[1] << ", " << voxel[2]
      << "]" << std::endl;
  }
}

/**
 * @brief This method is called to process the point cloud from the lidar scans.
 * Currently, raytracing is always set to false, so it is never called. The
 * decayLocalCloud method is called inside this method, so we take this into
 * consideration for the test. Points are provided with respect to the lidar
 * frame and the transformation that is passed as an argument is the pose of the
 * lidar in the map frame. Any points outside of max_range are discarded. In
 * this test, we simulate the lidar to be at a world coordinate different than
 * (0, 0, 0) and make a single call to addCloud. This represents a single lidar
 * scan, but to make sure that the voxels corresponding to the points are marked
 * as occupied in this single scan, we multiply the points by the necessary
 * amount to condsider a voxel occupied. After making the call we compare the
 * normal map and the inflated map to their ground truths.
 */
TEST_F(VoxelMapperTest, TestAddCloud) {
  // We will consider that the lidar is at position (-10, -5, -1) without any
  // rotation in the map frame of reference
  Eigen::Vector3d lidar_pos(-10, -5, -1);
  Eigen::Affine3d t_map_lidar = Eigen::Translation3d(lidar_pos) *
                                Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 0));
  double max_range = 3.0;
  vec_Vec3d lidar_points;

  // There needs to be more than num_points points per voxel to have a single
  // point cloud set the respective voxels occupied
  int num_points = (val_occ_ - val_even_) / val_add_;

  // Add num_points+1 points for 20 voxels in the positive and negative
  // direction along all 3 axes. Note that these points are with respect to
  // the lidar frame which is at position (-10, -5, -1) in the world frame
  for (int k = -20; k < 20; k++) {
    double real_k = k * 0.5 + 0.25;
    Eigen::Vector3d point_x(real_k, 0, 0);
    Eigen::Vector3d point_y(0, real_k, 0);
    Eigen::Vector3d point_z(0, 0, real_k);
    for (int n = 0; n < num_points + 1; n++) {
      lidar_points.push_back(point_x);
      lidar_points.push_back(point_y);
      lidar_points.push_back(point_z);
    }
  }

  // Current neighboring voxels for global map are just +-1 in x and y
  // directions, so replicate this for tests
  vec_Vec3i neighbors;
  neighbors.push_back(Eigen::Vector3i(-1, 0, 0));
  neighbors.push_back(Eigen::Vector3i(1, 0, 0));
  neighbors.push_back(Eigen::Vector3i(0, -1, 0));
  neighbors.push_back(Eigen::Vector3i(0, 1, 0));

  p_test_mapper_->addCloud(lidar_points, t_map_lidar, neighbors, false,
                            max_range);
  kr_planning_msgs::VoxelMap processed_map = p_test_mapper_->getMap();
  kr_planning_msgs::VoxelMap processed_inflated_map;
  processed_inflated_map = p_test_mapper_->getInflatedMap();

  // Create the ground truth voxel maps to compare to the normal map and the
  // inflated map
  kr_planning_msgs::VoxelMap gt_voxel_map;
  kr_planning_msgs::VoxelMap gt_inflated_voxel_map;
  int x_dim = x_dim_ / resolution_;
  int y_dim = y_dim_ / resolution_;
  int z_dim = z_dim_ / resolution_;
  gt_voxel_map.data.resize(x_dim * y_dim * z_dim, val_default_);
  gt_inflated_voxel_map.data.resize(x_dim * y_dim * z_dim, val_default_);

  // Fill the voxels that should be occupied. To know which voxels should be
  // occupied, consider the pose of the lidar and the max range of the points
  // meaning some points should be filtered out. The lidar position
  // (-10, -5, -1) lies at the most negative corners of voxels
  // X: 180, Y: 190, Z: 8 and with a range of 3 for the points, that is 6
  // voxels towards each direction
  Eigen::Vector3i voxel_start(180, 190, 8);
  std::vector<std::vector<int>> ranges{{174, 185}, {184, 195}, {2, 13}};

  for (int i = 0; i < 3; i++) {
    // While iterating through each range, keep the other two axes static
    for (int k = ranges[i][0]; k <= ranges[i][1]; k++) {
      int x = i == 0 ? k : voxel_start[0];
      int y = i == 1 ? k : voxel_start[1];
      int z = i == 2 ? k : voxel_start[2];

      int idx = x + x_dim * y + x_dim * y_dim * z;
      gt_voxel_map.data[idx] = val_occ_;
      gt_inflated_voxel_map.data[idx] = val_occ_;

      // Include neighbors for the inflated map
      Eigen::Vector3i current_voxel(x, y, z);
      for (auto &neighbor : neighbors) {
        Eigen::Vector3i neighbor_voxel = current_voxel + neighbor;
        int idx = neighbor_voxel(0) + x_dim * neighbor_voxel(1)
                    + x_dim * y_dim * neighbor_voxel(2);
        gt_inflated_voxel_map.data[idx] = val_occ_;
      }
    }
  }

  // Compare normal map
  int num_voxels = x_dim * y_dim * z_dim;
  ASSERT_EQ(processed_map.data.size(), gt_voxel_map.data.size());
  for (int idx = 0; idx < num_voxels; idx++) {
    auto voxel = getVoxel(idx);
    EXPECT_EQ(processed_map.data[idx], gt_voxel_map.data[idx])
      << "IDX:" << idx << "\t"
      << "Voxel: [" << voxel[0] << ", " << voxel[1] << ", " << voxel[2]
      << "]" << std::endl;
  }

  // Compare the inflated map
  ASSERT_EQ(processed_inflated_map.data.size(),
            gt_inflated_voxel_map.data.size());
  for (int idx = 0; idx < num_voxels; idx++) {
    auto voxel = getVoxel(idx);
    EXPECT_EQ(processed_inflated_map.data[idx], gt_inflated_voxel_map.data[idx])
      << "IDX:" << idx << "\t"
      << "Voxel: [" << voxel[0] << ", " << voxel[1] << ", " << voxel[2]
      << "]" << std::endl;
  }
}

/**
 * @brief If no point cloud has been processed, then the inflated map and the
 * normal map are identical. The processing of the cloud is tested separately so
 * here we are just going to make sure that the two maps are indeed identical
 * upon initialization. Initialization is done in the constructor so here we
 * will just compare the maps to each other after being created.
 */
TEST_F(VoxelMapperTest, TestGetInflatedMap) {
  kr_planning_msgs::VoxelMap normal_map = p_test_mapper_->getMap();
  kr_planning_msgs::VoxelMap inflated_map = p_test_mapper_->getInflatedMap();

  int num_voxels = 3200000;
  ASSERT_EQ(normal_map.data.size(), num_voxels);
  ASSERT_EQ(normal_map.data.size(), inflated_map.data.size());
  for (int idx = 0; idx < num_voxels; idx++) {
    auto voxel = getVoxel(idx);
    EXPECT_EQ(normal_map.data[idx], inflated_map.data[idx])
      << "IDX:" << idx << "\t"
      << "Voxel: [" << voxel[0] << ", " << voxel[1] << ", " << voxel[2]
      << "]" << std::endl;
  }
}

/**
 * @brief This method extracts a portion of the world. It will retrieve a map
 * that is cropped out of the inflated map. The function takes as arguments an
 * origin and dimensions for the cropped map, but the resolution is the same as
 * the inflated map. One key difference is that any voxels that are outside
 * the bounds of the cropped inflated map are considered as occupied. In this
 * test, given the default map that is created with all voxels set to default
 * value, we extract a local map that is partially outside the bounds of the
 * intial map. So the extracted local map should have the overlapping voxels set
 * to the default value while all other voxels set to occupied.
 */
TEST_F(VoxelMapperTest, TestGetInflatedLocalMap) {
  // The origin of the local map is moved in the positive direction in all
  // three axes by different amounts. This will cause a portion of the local
  // map to overlap with the original map.
  Eigen::Vector3d origin(75, 25, 2.5);    // Prev origin was (-100, -100, -5)
  Eigen::Vector3d dimensions(100, 100, 10);   // Prev dimen was (200, 200, 10)

  kr_planning_msgs::VoxelMap local_map;
  local_map = p_test_mapper_->getInflatedLocalMap(origin, dimensions);

  // Create the ground truth voxel map to compare it to the local map
  kr_planning_msgs::VoxelMap gt_voxel_map;
  int dim_x = dimensions(0) / resolution_;
  int dim_y = dimensions(1) / resolution_;
  int dim_z = dimensions(2) / resolution_;
  gt_voxel_map.data.resize(dim_x * dim_y * dim_z, val_occ_);

  // Set the overlapping voxels to the default value
  for (int x = 0; x < 50; x++) {          // 200 - 150 = 50
    for (int y = 0; y < 150; y++) {       // 200 - 50 = 150
      for (int z = 0; z < 5; z++) {       // 10 - 5 = 5
        int idx = x + dim_x * y + dim_x * dim_y * z;
        gt_voxel_map.data[idx] = val_default_;
      }
    }
  }

  // Finally make the comparison
  int num_voxels = dim_x * dim_y * dim_z;
  ASSERT_EQ(local_map.data.size(), gt_voxel_map.data.size());
  for (int idx = 0; idx < num_voxels; idx++) {
    auto voxel = getVoxel(idx);
    EXPECT_EQ(local_map.data[idx], gt_voxel_map.data[idx])
      << "IDX:" << idx << "\t"
      << "Voxel: [" << voxel[0] << ", " << voxel[1] << ", " << voxel[2]
      << "]" << std::endl;
  }
}

/**
 * @brief In this test, we check that the cloud is retrieved correctly.
 * For every voxel that is occupied in the map, a point lying at the
 * center of the voxel should be included in the returned cloud.
 */
TEST_F(VoxelMapperTest, TestGetCloud) {
  Eigen::Affine3d t_map_lidar = Eigen::Translation3d(0, 0, 0) *
                              Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 0));

  // There needs to be more than num_points points per voxel to have a single
  // point cloud set the respective voxels occupied
  int num_points = (val_occ_ - val_even_) / val_add_;

  // Add num_points+1 points for all points in gt_cloud
  vec_Vec3d scan_points;
  for (auto &point : gt_cloud_) {
    for (int n = 0; n < num_points + 1; n++) {
      scan_points.push_back(point);
    }
  }

  // No neighboring voxels for inflated map, lidar pose is at the origin,
  // no raytracing and the max range is large enough to avoid filtering out
  // any point within the bounds of the map
  p_test_mapper_->addCloud(scan_points, t_map_lidar, vec_Vec3i(), false, 180);

  vec_Vec3d point_cloud = p_test_mapper_->getCloud();

  // Sort the received point cloud to compare it to the ground truth point
  // cloud. We sort because it should not matter in what order the points
  // are received
  std::sort(point_cloud.begin(), point_cloud.end(), comparePoints);

  ASSERT_EQ(point_cloud.size(), gt_cloud_.size());
  // Iterate over every point
  for (int i = 0; i < gt_cloud_.size(); i++) {
    // iterate over every index in each point
    for (int n = 0; n < 3; n++) {
      EXPECT_DOUBLE_EQ(point_cloud[i][n], gt_cloud_[i][n])
        << "IDX: " << i << ", Axis: " << n << std::endl;
    }
  }
}

/**
 * @brief In this test, we check that the inflated cloud is retrieved correctly.
 * For every voxel that is occupied in the inflated map, a point lying at the
 * center of the voxel should be included in the returned cloud.
 */
TEST_F(VoxelMapperTest, TestGetInflatedCloud) {
  Eigen::Affine3d t_map_lidar = Eigen::Translation3d(0, 0, 0) *
                              Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 0));

  // There needs to be more than num_points points per voxel to have a single
  // point cloud set the respective voxels occupied
  int num_points = (val_occ_ - val_even_) / val_add_;

  // Add num_points+1 points for all points in gt_cloud
  vec_Vec3d scan_points;
  for (auto &point : gt_cloud_) {
    for (int n = 0; n < num_points + 1; n++) {
      scan_points.push_back(point);
    }
  }

  // No neighboring voxels for inflated map, lidar pose is at the origin,
  // no raytracing and the max range is large enough to avoid filtering out
  // any point within the bounds of the map
  p_test_mapper_->addCloud(scan_points, t_map_lidar, vec_Vec3i(), false, 180);

  vec_Vec3d point_cloud = p_test_mapper_->getInflatedCloud();

  // Sort the received point cloud to compare it to the ground truth point
  // cloud. We sort because it should not matter in what order the points
  // are received
  std::sort(point_cloud.begin(), point_cloud.end(), comparePoints);

  ASSERT_EQ(point_cloud.size(), gt_cloud_.size());
  // Iterate over every point
  for (int i = 0; i < gt_cloud_.size(); i++) {
    // iterate over every index in each point
    for (int n = 0; n < 3; n++) {
      EXPECT_DOUBLE_EQ(point_cloud[i][n], gt_cloud_[i][n])
        << "IDX: " << i << ", Axis: " << n << std::endl;
    }
  }
}

/**
 * @brief Similar to the getCloud and getInflatedCloud tests, here we retrieve
 * a cloud in a cropped out section of the map. We choose a position and
 * dimensions making sure that we only encapsulate some of the points in the
 * ground truth cloud. We first add the points from the ground truth cloud by
 * calling addCloud. Then we create the local ground truth since not all points
 * should be returned when calling getLocalCloud.
 */
TEST_F(VoxelMapperTest, TestGetLocalCloud) {
  Eigen::Affine3d t_map_lidar = Eigen::Translation3d(0, 0, 0) *
                              Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 0));

  // There needs to be more than num_points points per voxel to have a single
  // point cloud set the respective voxels occupied
  int num_points = (val_occ_ - val_even_) / val_add_;

  // Add num_points+1 points for all points in gt_cloud
  vec_Vec3d scan_points;
  for (auto &point : gt_cloud_) {
    for (int n = 0; n < num_points + 1; n++) {
      scan_points.push_back(point);
    }
  }

  // No neighboring voxels for inflated map, lidar pose is at the origin,
  // no raytracing and the max range is large enough to avoid filtering out
  // any point within the bounds of the map
  p_test_mapper_->addCloud(scan_points, t_map_lidar, vec_Vec3i(), false, 180);

  // Get points that are within the following bounds:
  // Lower: [-35, -50, 1]
  // Upper: [35, 25, 4.4]
  Eigen::Vector3d origin(-35, -50, 1);
  Eigen::Vector3d dimensions(70, 75, 3.4);
  vec_Vec3d point_cloud = p_test_mapper_->getLocalCloud(origin, dimensions);

  // Create the local ground truth cloud
  vec_Vec3d gt_local_cloud;
  gt_local_cloud.push_back(Eigen::Vector3d(-28.25, -12.25, 3.25));
  gt_local_cloud.push_back(Eigen::Vector3d(0.75, 0.75, 1.75));
  gt_local_cloud.push_back(Eigen::Vector3d(20.25, -49.25, 4.25));

  // Sort the received point cloud to compare it to the ground truth point
  // cloud. We sort because it should not matter in what order the points
  // are received
  std::sort(point_cloud.begin(), point_cloud.end(), comparePoints);

  ASSERT_EQ(point_cloud.size(), gt_local_cloud.size());
  // Iterate over every point
  for (int i = 0; i < gt_local_cloud.size(); i++) {
    // iterate over every index in each point
    for (int n = 0; n < 3; n++) {
      EXPECT_DOUBLE_EQ(point_cloud[i][n], gt_local_cloud[i][n])
        << "IDX: " << i << ", Axis: " << n << std::endl;
    }
  }
}

/**
 * @brief Same as TestGetLocalCloud, but for the inflated map
 */
TEST_F(VoxelMapperTest, TestGetInflatedLocalCloud) {
  Eigen::Affine3d t_map_lidar = Eigen::Translation3d(0, 0, 0) *
                              Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 0));

  // There needs to be more than num_points points per voxel to have a single
  // point cloud set the respective voxels occupied
  int num_points = (val_occ_ - val_even_) / val_add_;

  // Add num_points+1 points for all points in gt_cloud
  vec_Vec3d scan_points;
  for (auto &point : gt_cloud_) {
    for (int n = 0; n < num_points + 1; n++) {
      scan_points.push_back(point);
    }
  }

  // No neighboring voxels for inflated map, lidar pose is at the origin,
  // no raytracing and the max range is large enough to avoid filtering out
  // any point within the bounds of the map
  p_test_mapper_->addCloud(scan_points, t_map_lidar, vec_Vec3i(), false, 180);

  // Get points that are within the following bounds:
  // Lower: [-35, -50, 1]
  // Upper: [35, 25, 4.4]
  Eigen::Vector3d origin(-35, -50, 1);
  Eigen::Vector3d dimensions(70, 75, 3.4);
  vec_Vec3d point_cloud =
    p_test_mapper_->getInflatedLocalCloud(origin, dimensions);

  // Create the local ground truth cloud
  vec_Vec3d gt_local_cloud;
  gt_local_cloud.push_back(Eigen::Vector3d(-28.25, -12.25, 3.25));
  gt_local_cloud.push_back(Eigen::Vector3d(0.75, 0.75, 1.75));
  gt_local_cloud.push_back(Eigen::Vector3d(20.25, -49.25, 4.25));

  // Sort the received point cloud to compare it to the ground truth point
  // cloud. We sort because it should not matter in what order the points
  // are received
  std::sort(point_cloud.begin(), point_cloud.end(), comparePoints);

  ASSERT_EQ(point_cloud.size(), gt_local_cloud.size());
  // Iterate over every point
  for (int i = 0; i < gt_local_cloud.size(); i++) {
    // iterate over every index in each point
    for (int n = 0; n < 3; n++) {
      EXPECT_DOUBLE_EQ(point_cloud[i][n], gt_local_cloud[i][n])
        << "IDX: " << i << ", Axis: " << n << std::endl;
    }
  }
}

/**
 * @brief The freeVoxels function takes in a single point and free its
 * corrresponding voxel along with the voxel's neighbors. So in this test we
 * use the addCloud method to occupy a single voxel and its neighbors.
 * Then we free it with the freeVoxels method and check if it was done correctly
 */
TEST_F(VoxelMapperTest, TestFreeVoxels) {
  Eigen::Affine3d t_map_lidar = Eigen::Translation3d(0, 0, 0) *
                              Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 0));

  // There needs to be more than num_points points per voxel to have a single
  // point cloud set the respective voxels occupied
  int num_points = (val_occ_ - val_even_) / val_add_;

  // Add num_points+1 points for all points in gt_cloud
  vec_Vec3d scan_points;
  for (int n = 0; n < num_points + 1; n++) {
    scan_points.push_back(Eigen::Vector3d(15, 10, 2));
  }

  // Current neighboring voxels for global map are just +-1 in x and y
  // directions, so replicate this for tests
  vec_Vec3i neighbors;
  neighbors.push_back(Eigen::Vector3i(-1, 0, 0));
  neighbors.push_back(Eigen::Vector3i(1, 0, 0));
  neighbors.push_back(Eigen::Vector3i(0, -1, 0));
  neighbors.push_back(Eigen::Vector3i(0, 1, 0));

  p_test_mapper_->addCloud(scan_points, t_map_lidar, neighbors, false, 180);

  // Make sure that 5 voxels were marked as occupied
  ASSERT_EQ(p_test_mapper_->getCloud().size(), 1);
  ASSERT_EQ(p_test_mapper_->getInflatedCloud().size(), 5);

  // Now free the voxels
  p_test_mapper_->freeVoxels(Eigen::Vector3d(15, 10, 2), neighbors);

  // No voxels should be occupied now
  ASSERT_EQ(p_test_mapper_->getCloud().size(), 0);

  // If the neighboring voxels in the map are free, then the same voxels in
  // inflated map are not freed.
  ASSERT_EQ(p_test_mapper_->getInflatedCloud().size(), 4);
}

/**
 * @brief The getInflatedOccMap method returns a 2D slice of the inflated map.
 * You pass in the z height and the thickness of the slice as parameters. The
 * thickness refers to how much vertical space is going to be taken into account
 * to create the 2D slice in either direction. Since it is 2D, there is only one
 * voxel along the z axis. In this test we use the previously initialized ground
 * truth cloud to occupy some voxels in the inflated map. Then we select
 * an arbitrary height and thickness. Finally we create a ground truth 2D map to
 * compare the results to.
 */
TEST_F(VoxelMapperTest, TestgetInflatedOccMap) {
  Eigen::Affine3d t_map_lidar = Eigen::Translation3d(0, 0, 0) *
                              Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 0));

  // There needs to be more than num_points points per voxel to have a single
  // point cloud set the respective voxels occupied
  int num_points = (val_occ_ - val_even_) / val_add_;

  // Add num_points+1 points for all points in gt_cloud
  vec_Vec3d scan_points;
  for (auto &point : gt_cloud_) {
    for (int n = 0; n < num_points + 1; n++) {
      scan_points.push_back(point);
    }
  }

  // No neighboring voxels for inflated map, lidar pose is at the origin,
  // no raytracing and the max range is large enough to avoid filtering out
  // any point within the bounds of the map
  p_test_mapper_->addCloud(scan_points, t_map_lidar, vec_Vec3i(), false, 180);

  // Should consider vertical space between [-1.0, 3.5]
  kr_planning_msgs::VoxelMap sliced_map =
    p_test_mapper_->getInflatedOccMap(1.25, 2.25);

  // Create the ground truth voxel map
  kr_planning_msgs::VoxelMap gt_slice;
  int x_dim = x_dim_ / resolution_;
  int y_dim = y_dim_ / resolution_;
  gt_slice.data.resize(x_dim * y_dim, val_free_);

  // The points occupying a voxel in the previous vertical space are:
  // (0.25, 0.25, 0.25)       -> voxel indices: [200, 200, 10]
  // (0.75, 0.75, 1.75)       -> voxel indices: [201, 201, 13]
  // (-28.25, -12.25, 3.25)   -> voxel indices: [143, 175, 16]
  // (85.25, -61.25, 2.25)    -> voxel indices: [370, 77, 14]
  // The returned voxel map uses column-major storage and we also have to
  // consider axis aligned voxels, so the occupied pixels are computed as
  // follows: x + x_dim * y, where x and y are the integer indices
  std::vector<int> indices {80200, 80601, 70143, 31170};
  for (auto &idx : indices) {
    gt_slice.data[idx] = val_occ_;
  }

  // Finally compare the two slices
  ASSERT_EQ(gt_slice.data.size(), sliced_map.data.size());

  for (int i = 0; i < gt_slice.data.size(); i++) {
    EXPECT_EQ(gt_slice.data[i], sliced_map.data[i]) << std::endl;
  }
}

}   // namespace mapper


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

