#include <gtest/gtest.h>
#include <planning_ros_msgs/VoxelMap.h>
#include <iostream>
#include <cmath>
#include <memory>
#include "mapper/voxel_mapper.h"


// Test class needs to be in the same namespace as VoxelMapper to be able to
// make it a friend class
namespace mapper {

/** Declaration of fixture class to use the same configuration across tests **/
class VoxelMapperTest : public testing::Test {
 protected:
    void SetUp() override;

    double x_origin_;
    double y_origin_;
    double z_origin_;
    int x_dim_;
    int y_dim_;
    int z_dim_;
    double resolution_;
    int8_t val_default_;
    int8_t val_occ_;
    int8_t val_even_;
    int8_t val_add_;
    int decay_times_;

    // This is the VoxelMapper object that is used for all the tests
    std::unique_ptr<mapper::VoxelMapper> p_test_mapper_;
};

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
    val_occ_ = p_test_mapper_->val_occ;
    val_even_ = p_test_mapper_->val_even;
    val_add_ = p_test_mapper_->val_add;
}


/*** Defining Tests ***/

/**
 * @brief The allocate() method is able to relocate the current map into another
 * part of the world, but this functionality is currently NOT being used since
 * allocate() is only called in the constructor of the VoxelMapper class. The
 * map is stored in the map_ data member and we could access it directly via
 * friendship to verify the correct initialization of allocate(), but this test
 * should not depend on the data type of the map_ variable; for this reason, we
 * use the getMap() method to check the status of the map. In the first three
 * assertions, we check that the map is created with the correct dimensions.
 */
TEST_F(VoxelMapperTest, TestAllocateDimensions) {
    // Creating voxel mapper object with same parameters as global_voxel_mapper
    // in source file: local_global_mapper.cpp
    Eigen::Vector3d origin(-100, -100, -5);
    Eigen::Vector3d dimensions(200, 200, 10);
    p_test_mapper_.reset(new mapper::VoxelMapper(origin, dimensions,
                                                 0.5, 0, 30));
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
    p_test_mapper_.reset(new mapper::VoxelMapper(origin, dimensions,
                                                 0.1, 0, 30));
    EXPECT_EQ(p_test_mapper_->getMap().data.size(), 1000000);
}

/**
 * @brief Similar to TestAllocateDimensions but here we test that the relocating
 * of the map is working properly. 
 */
TEST_F(VoxelMapperTest, TestAllocateRelocate) {
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
    std::fill(p_test_mapper_->map_.data(),
              p_test_mapper_->map_.data() + num_voxels, val_occ_);

    // Relocating map in the positive direction in all three axes centered at
    // (0,0,0) and reducing x-dim and y-dim by 50. With the same resolution this
    // means 100 less voxels in each dimension. z stays the same
    Eigen::Vector3d new_origin(0, 0, 0);
    Eigen::Vector3d new_dimensions(150, 150, 10);
    // True means that the relocating actually happened
    EXPECT_TRUE(p_test_mapper_->allocate(new_dimensions, new_origin));

    planning_ros_msgs::VoxelMap relocated_map = p_test_mapper_->getMap();

    // Create the ground truth voxel map to compare the relocated map to
    planning_ros_msgs::VoxelMap gt_voxel_map;
    // new map should have 300 voxels in x dimension
    int dim_x = 150 / resolution_;
    // new map should have 300 voxels in y dimension
    int dim_y = 150 / resolution_;
    // new map should have 20 voxels in z dimension
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
    // x: 200 voxels marked as occupied and 100 as default
    // y: 200 voxels marked as occupied and 100 as default
    // z: 10 voxels marked as occupied and 10 as default
    // so set that for the ground truth voxel map
    for (int x = 0 ; x < 200; x++) {
        for (int y = 0; y < 200; y++) {
            for (int z = 0; z < 10; z++) {
                int idx = x + 300 * y + 300 * 300 * z;
                gt_voxel_map.data[idx] = val_occ_;
            }
        }
    }
    num_voxels = relocated_map.data.size();
    ASSERT_EQ(relocated_map.data.size(), gt_voxel_map.data.size());
    for (int idx = 0; idx < num_voxels; idx++) {
        EXPECT_EQ(relocated_map.data[idx], gt_voxel_map.data[idx]) << "IDX: "
                                                                   << idx;
    }
}

/**
 * @brief The decayLocalCloud method decays all the occupied voxels within a
 * local range. So first we mark all the voxels in the map as occupied and call
 * decayLocalCloud multiple times. Since getMap doesn't return the exact value
 * of the voxel, we will call decayLocalCloud until the in-range voxels are
 * equal to val_even; this corresponds to calling decayLocalCloud a total of
 * (val_occ - val_even)/decay_times_ times. Only the voxels that are within
 * range should be decayed and marked as free, while the rest should remain as
 * occupied 
 */
TEST_F(VoxelMapperTest, TestDecayLocalCloud) {
    // Compute how many times decayLocalCloud must be called to free the voxels
    int num_calls = (static_cast<float>(p_test_mapper_->val_occ)
                     - static_cast<float>(p_test_mapper_->val_even))
                     / static_cast<float>(p_test_mapper_->val_decay);

    // Mark all voxels in the map as occupied
    int num_voxels = 3200000;
    std::fill(p_test_mapper_->map_.data(),
              p_test_mapper_->map_.data() + num_voxels,
              p_test_mapper_->val_occ);

    // Now decay the voxels that are in the range of 9.8 in all three axes
    // around (0, 0, 0)
    Eigen::Vector3d position(0, 0, 0);
    double max_range = 9.8;
    for (int i = 0; i < num_calls; i++) {
        p_test_mapper_->decayLocalCloud(position, max_range);
    }

    planning_ros_msgs::VoxelMap decayed_map = p_test_mapper_->getMap();

    // Create the ground truth voxel map to compare it to the decayed map
    planning_ros_msgs::VoxelMap gt_voxel_map;
    int dim_x = x_dim_ / resolution_;
    int dim_y = y_dim_ / resolution_;
    int dim_z = z_dim_ / resolution_;
    // All voxels should be marked occupied except the decayed voxel locations
    gt_voxel_map.data.resize(dim_x * dim_y * dim_z, p_test_mapper_->val_occ);

    // Free the voxels that should be decayed
    for (int x = 180; x < 219; x++) {
        for (int y = 180; y < 219; y++) {
            for (int z = 0; z < 20; z++) {
                int idx = x + 400 * y + 400 * 400 * z;
                gt_voxel_map.data[idx] = p_test_mapper_->val_free;
            }
        }
    }

    // Finally make the comparison
    ASSERT_EQ(decayed_map.data.size(), gt_voxel_map.data.size());
    for (int idx = 0; idx < num_voxels; idx++) {
        EXPECT_EQ(decayed_map.data[idx], gt_voxel_map.data[idx]) << "IDX: "
                                                                 << idx;
    }
}

/**
 * @brief This method is called to process the point cloud from the lidar scans.
 * Currently, raytracing is always set to false, so it is never called. The
 * decayLocalCloud method is called inside this method, but since the decay
 * value is 2, it does not impede a voxel from being marked as occupied after
 * one scan if we include 3 lidar points in the specified voxel. Points are
 * provided with respect to the lidar frame and the transformation that is
 * passed as an argument is the pose of the lidar in the map frame. Any points
 * outside of max_range are discarded. In this test, we simulate the lidar to be
 * at a world coordinate different than (0, 0, 0) and make a single call to
 * addCloud. This represents a single lidar scan, but to make sure that the
 * voxels corresponding to the points are marked as occupied in this single
 * scan, we triplicate the points. After making the call we compare the normal
 * map and the inflated map to their ground truths.
 */
TEST_F(VoxelMapperTest, TestAddCloud) {
    // We will consider that the lidar is at position (-10, -5, -1) in the map
    // frame of reference
    Eigen::Vector3d lidar_pos(-10, -5, -1);
    Eigen::Affine3d t_map_lidar = Eigen::Translation3d(-10, -5, -1) *
                                  Eigen::AngleAxisd(0,
                                                    Eigen::Vector3d(0, 0, 0));
    double max_range = 30.0;
    vec_Vec3d lidar_points;

    // Number of points need for one scan to mark voxel as occupied
    int num_points = (val_occ_ - val_even_) / val_add_;

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
                for (int n = 0; n < 3; n++) {
                    lidar_points.push_back(point);
                }
            }
        }
    }

    // Current neighboring voxels are just +-1 in x and y directions
    vec_Vec3i neighbors;
    neighbors.push_back(Eigen::Vector3i(-1, 0, 0));
    neighbors.push_back(Eigen::Vector3i(1, 0, 0));
    neighbors.push_back(Eigen::Vector3i(0, -1, 0));
    neighbors.push_back(Eigen::Vector3i(0, 1, 0));

    p_test_mapper_->addCloud(lidar_points, t_map_lidar, neighbors, false,
                             max_range);
    planning_ros_msgs::VoxelMap processed_map = p_test_mapper_->getMap();
    planning_ros_msgs::VoxelMap processed_inflated_map;
    processed_inflated_map = p_test_mapper_->getInflatedMap();

    // Create the ground truth voxel maps to compare to the normal map and the
    // inflated map
    planning_ros_msgs::VoxelMap gt_voxel_map;
    planning_ros_msgs::VoxelMap gt_inflated_voxel_map;
    int x_dim = x_dim_ / resolution_;
    int y_dim = y_dim_ / resolution_;
    int z_dim = z_dim_ / resolution_;
    gt_voxel_map.data.resize(x_dim * y_dim * z_dim, val_default_);
    gt_inflated_voxel_map.data.resize(x_dim * y_dim * z_dim, val_default_);

    // Fill the voxels that should be occupied
    for (int x = 0; x < x_dim; x++) {
        for (int y = 0; y < y_dim; y++) {
            for (int z = 0; z < z_dim; z++) {
                Eigen::Vector3d voxel_pos = Eigen::Vector3d(x, y, z) *
                                            resolution_ +
                                            Eigen::Vector3d(x_origin_,
                                                            y_origin_,
                                                            z_origin_);
                if ((voxel_pos - lidar_pos).norm() <= max_range) {
                    int idx = x + x_dim * y + x_dim * y_dim * z;
                    gt_voxel_map.data[idx] = gt_voxel_map.val_occ;
                    gt_inflated_voxel_map.data[idx] = gt_voxel_map.val_occ;

                    // Include neighbors for the inflated map
                    Eigen::Vector3i current_voxel(x, y, z);
                    for (auto &neighbor : neighbors) {
                        Eigen::Vector3i neigbor_voxel = current_voxel +
                                                        neighbor;
                        int idx = neigbor_voxel(0) + x_dim * neigbor_voxel(1)
                                  + x_dim * y_dim * neigbor_voxel(2);
                        gt_inflated_voxel_map.data[idx] = gt_voxel_map.val_occ;
                    }
                }
            }
        }
    }

    // Compare normal map
    int num_voxels = x_dim * y_dim * z_dim;
    ASSERT_EQ(processed_map.data.size(), gt_voxel_map.data.size());
    for (int idx = 0; idx < num_voxels; idx++) {
        EXPECT_EQ(processed_map.data[idx], gt_voxel_map.data[idx]) << "IDX: "
                                                                   << idx;
    }

    // Compare the inflated map
    ASSERT_EQ(processed_inflated_map.data.size(),
              gt_inflated_voxel_map.data.size());
    for (int idx = 0; idx < num_voxels; idx++) {
        EXPECT_EQ(processed_inflated_map.data[idx],
                  gt_inflated_voxel_map.data[idx]) << "IDX: " << idx;
    }
}

/**
 * @brief If no point cloud has been processed, then the inflated map and the normal map
 * are identical. The processing of the cloud is tested separately so here we are just going to
 * make sure that the two maps are indeed identical upon initialization. Initialization is done
 * in the constructor so here we will just compare the maps.
 */
TEST_F(VoxelMapperTest, TestGetInflatedMap) {
    planning_ros_msgs::VoxelMap normal_map = p_test_mapper_->getMap();
    planning_ros_msgs::VoxelMap inflated_map = p_test_mapper_->getInflatedMap();

    int num_voxels = 3200000;
    ASSERT_EQ(normal_map.data.size(), num_voxels);
    ASSERT_EQ(normal_map.data.size(), inflated_map.data.size());
    for (int idx = 0; idx < num_voxels; idx++) {
        EXPECT_EQ(normal_map.data[idx], inflated_map.data[idx]) << "IDX: "
                                                                << idx;
    }
}

/**
 * @brief This method extracts a portion of the world with a world origin and
 * map dimensions provided as arguments. The resolution is the same as the
 * inflated map and it will retrieve the inflated map, similar to the
 * getInflatedMap method. One key difference is that any voxels that are outside
 * the bounds of the inflated map are considered as occupied. In this test,
 * given the map created with all voxels set to default, we extract a local map
 * that is partially outside the bounds of the intial map. So the extracted
 * local map should have the overlapping voxels set to the default value while
 * all other voxels set to occupied.
 */
TEST_F(VoxelMapperTest, TestGetInflatedLocalMap) {
    // The origin of the local map is moved in the positive direction in all
    // three axes by different amounts. This will cause a portion of the local
    // map to overlap with the original map.
    Eigen::Vector3d origin(75, 25, 2.5);
    Eigen::Vector3d dimensions(100, 100, 10);

    planning_ros_msgs::VoxelMap local_map;
    local_map = p_test_mapper_->getInflatedLocalMap(origin, dimensions);

    // Create the ground truth voxel map to compare it to the local map
    planning_ros_msgs::VoxelMap gt_voxel_map;
    int dim_x = dimensions(0) / resolution_;
    int dim_y = dimensions(1) / resolution_;
    int dim_z = dimensions(2) / resolution_;
    gt_voxel_map.data.resize(dim_x * dim_y * dim_z, val_occ_);

    for (int x = 0; x < 50; x++) {          // 200 - 150 = 50
        for (int y = 0; y < 150; y++) {     // 200 - 50 = 150
            for (int z = 0; z < 5; z++) {   // 10 - 5 = 5
                int idx = x + dim_x * y + dim_x * dim_y * z;
                gt_voxel_map.data[idx] = val_default_;
            }
        }
    }

    // Finally make the comparison
    int num_voxels = dim_x * dim_y * dim_z;
    ASSERT_EQ(local_map.data.size(), gt_voxel_map.data.size());
    for (int idx = 0; idx < num_voxels; idx++) {
        EXPECT_EQ(local_map.data[idx], gt_voxel_map.data[idx]) << "IDX: "
                                                               << idx;
    }
}

}   // namespace mapper


