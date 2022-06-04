#include <iostream>
#include <gtest/gtest.h>
#include <memory>
#include <cmath>
#include <chrono>
#include <planning_ros_msgs/VoxelMap.h>
#include "mapper/voxel_mapper.h"


// Test class needs to be in the same namespace as VoxelMapper to be able to make it a friend class
namespace mapper
{

/*** Declaration of fixture class to use the same configuration across tests ***/
class VoxelMapperTest : public testing::Test
{
protected:
    // Google test needs to create objects of this test fixture and it calls the default constructor
    VoxelMapperTest();
    VoxelMapperTest(double x_orig, double y_orig, double z_orig, int x_dim, int y_dim, int z_dim,
                    double res, int decay_times);

    const double x_origin_; 
    const double y_origin_; 
    const double z_origin_; 
    const int x_dim_;
    const int y_dim_;
    const int z_dim_;
    const double resolution_;
    const int decay_times_;

    // This is the VoxelMapper object that is used for all the tests
    std::unique_ptr<mapper::VoxelMapper> p_test_mapper_;

    // Timing variables to measure method callback performance
    std::chrono::time_point<std::chrono::steady_clock> time_start, time_end;
};


/*** Defining the methods in the VoxelMapperTest class defintion ***/
VoxelMapperTest::VoxelMapperTest()
    :VoxelMapperTest(-100, -100, -5, 200, 200, 10, 0.5, 30) {}

VoxelMapperTest::VoxelMapperTest(double x_orig, double y_orig, double z_orig,
                                 int x_dim, int y_dim, int z_dim, double res, int decay_times)
    : x_origin_(x_orig), y_origin_(y_orig), z_origin_(z_orig),
      x_dim_(x_dim), y_dim_(y_dim), z_dim_(z_dim),
      resolution_(res), decay_times_(decay_times)
{
    const Eigen::Vector3d origin(x_origin_, y_origin_, z_origin_);
    const Eigen::Vector3d dimensions(x_dim_, y_dim_, z_dim_);
    p_test_mapper_.reset(new mapper::VoxelMapper(origin, dimensions, resolution_, 0, decay_times_));
}


/*** Defining Tests ***/

/**
 * @brief The allocate() method is able to relocate the current map into another part of the
 * world, but this functionality is currently NOT being used since allocate() is only called
 * in the constructor of the VoxelMapper class. The map is stored in the map_ data member and
 * we could access it directly via friendship to verify the correct initialization of allocate(),
 * but this test should not depend on the data type of the map_ variable; for this reason, we use
 * the getMap() method to check the status of the map. In the first three assertions, we check that
 * the map is created with the correct dimensions. In the second portion we check that relocating is
 * working properly.
 */
TEST_F(VoxelMapperTest, TestAllocate)
{
    /*****  PART1: Test that map is created with correct dimensions  *****/
    // Creating voxel mapper object with same parameters as global_voxel_mapper in source file:
    // local_global_mapper.cpp
    Eigen::Vector3d origin(-100, -100, -5);
    Eigen::Vector3d dimensions(200, 200, 10);

    time_start = std::chrono::steady_clock::now();
    p_test_mapper_.reset(new mapper::VoxelMapper(origin, dimensions, 0.5, 0, 30));
    time_end = std::chrono::steady_clock::now();
    std::cout << "CALLBACK Duration (Constructor): " <<
        std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count()
        << "us" << std::endl;

    EXPECT_EQ(p_test_mapper_->getMap().data.size(), 3200000);

    // Creating voxel mapper object with same parameters as storage_voxel_mapper in source file:
    // local_global_mapper.cpp
    origin = Eigen::Vector3d(-100, -100, -5);
    dimensions = Eigen::Vector3d(200, 200, 12);
    p_test_mapper_.reset(new mapper::VoxelMapper(origin, dimensions, 0.25, 0, 30));
    EXPECT_EQ(p_test_mapper_->getMap().data.size(), 30720000);

    // Creating voxel mapper object with custom parameters
    origin = Eigen::Vector3d(-5, -5, -5);
    dimensions = Eigen::Vector3d(10, 10, 10);
    p_test_mapper_.reset(new mapper::VoxelMapper(origin, dimensions, 0.1, 0, 30));
    EXPECT_EQ(p_test_mapper_->getMap().data.size(), 1000000);


    /*****  PART2: Now test whether the relocation is working  *****/
    // Create voxel mapper object
    origin = Eigen::Vector3d(-100, -100, -5);
    dimensions = Eigen::Vector3d(200, 200, 10);
    p_test_mapper_.reset(new mapper::VoxelMapper(origin, dimensions, 0.5, 0, 30));

    // Mark all voxels as occupied. This original map, in world coordinates, ranges from:
    // x: -100 to 100
    // y: -100 to 100
    // z: -5 to 5
    // In voxels this translates to:
    // x: 200 voxels from -100 to 0 and 200 voxels from 0 to 100
    // y: 200 voxels from -100 to 0 and 200 voxels from 0 to 100
    // z: 10 voxels from -5 to 0 and 10 voxels from 0 to 5
    int num_voxels = 3200000;
    std::fill(p_test_mapper_->map_.data(), p_test_mapper_->map_.data() + num_voxels,
              p_test_mapper_->val_occ);

    // Relocating map in the positive direction in all three axes centered at (0,0,0) and
    // reducing x-dim and y-dim by 50. With the same resolution this means 100 less voxels in each
    // dimension. z stays the same
    Eigen::Vector3d new_origin(0, 0, 0);
    Eigen::Vector3d new_dimensions(150, 150, 10);
    // True means that the relocating actually happened
    time_start = std::chrono::steady_clock::now();
    bool allocated = p_test_mapper_->allocate(new_dimensions, new_origin);
    time_end = std::chrono::steady_clock::now();
    std::cout << "CALLBACK Duration (allocate): " <<
        std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count()
        << "us" << std::endl;
    
    EXPECT_TRUE(allocated);
    planning_ros_msgs::VoxelMap relocated_map = p_test_mapper_->getMap();

    // Create the ground truth voxel map to compare the relocated map to
    planning_ros_msgs::VoxelMap gt_voxel_map;
    int dim_x = 150 / 0.5;     // new map should have 300 voxels in x dimension
    int dim_y = 150 / 0.5;     // new map should have 300 voxels in y dimension
    int dim_z = 10 / 0.5;      // new map should have 20 voxels in z dimension
    gt_voxel_map.data.resize(dim_x * dim_y * dim_z, p_test_mapper_->val_default);

    // Sections with overlap between the previous origin-dimensions and the new origin-dimensions
    // should be marked as obstacles while all other voxels should have the default value.
    // In the relocated map, the dimensions range from
    // x: 0 to 150
    // y: 0 to 150
    // z: 0 to 10
    // Given the previous location and that all the voxels there were occupied, the new map has
    // x: 200 voxels marked as occupied and 100 as default
    // y: 200 voxels marked as occupied and 100 as default
    // z: 10 voxels marked as occupied and 10 as default
    // so set that for the ground truth voxel map
    for (int x = 0 ; x < 200; x++)
    {
        for (int y = 0; y < 200; y++)
        {
            for (int z = 0; z < 10; z++)
            {
                int idx = x + 300 * y + 300 * 300 * z;
                gt_voxel_map.data[idx] = p_test_mapper_->val_occ;
            }
        }
    }
    num_voxels = relocated_map.data.size(); 
    ASSERT_EQ(relocated_map.data.size(), gt_voxel_map.data.size());
    for (int idx = 0; idx < num_voxels; idx++)
    {
        EXPECT_EQ(relocated_map.data[idx], gt_voxel_map.data[idx]) << "IDX: " << idx;
    }
}

/**
 * @brief The decayLocalCloud method decays all the occupied voxels within a local range. So first
 * we mark all the voxels in the map as occupied and call decayLocalCloud multiple times. Since
 * getMap doesn't return the exact value of the voxel, we will call decayLocalCloud until the
 * in-range voxels are equal to val_even; this corresponds to calling decayLocalCloud a total of
 * (val_occ - val_even)/decay_times_ times. Only the voxels that are within range should be decayed
 * and marked as free, while the rest should remain as occupied 
 */
TEST_F(VoxelMapperTest, TestDecayLocalCloud)
{
    // Compute how many times decayLocalCloud must be called to free the voxels
    int num_calls = (static_cast<float>(p_test_mapper_->val_occ)
                     - static_cast<float>(p_test_mapper_->val_even))
                     / static_cast<float>(p_test_mapper_->val_decay);

    // Mark all voxels in the map as occupied
    int num_voxels = 3200000;
    std::fill(p_test_mapper_->map_.data(), p_test_mapper_->map_.data() + num_voxels,
              p_test_mapper_->val_occ);

    // Now decay the voxels that are in the range of 9.8 in all three axes around (0, 0, 0)
    Eigen::Vector3d position(0, 0, 0);
    double max_range = 9.8;
    time_start = std::chrono::steady_clock::now();
    for (int i = 0; i < num_calls; i++)
    {
        p_test_mapper_->decayLocalCloud(position, max_range);
    }
    time_end = std::chrono::steady_clock::now();
    std::cout << "CALLBACK Duration (decayLocalCloud): " <<
        std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count()
        << "us" << std::endl;

    planning_ros_msgs::VoxelMap decayed_map = p_test_mapper_->getMap();

    // Create the ground truth voxel map to compare it to the decayed map
    planning_ros_msgs::VoxelMap gt_voxel_map;
    int dim_x = x_dim_ / resolution_;
    int dim_y = y_dim_ / resolution_;
    int dim_z = z_dim_ / resolution_;
    // All voxels should be marked as occupied except the decayed voxel locations
    gt_voxel_map.data.resize(dim_x * dim_y * dim_z, p_test_mapper_->val_occ);

    // Free the voxels that should be decayed
    for (int x = 180; x < 219; x++)
    {
        for (int y = 180; y < 219; y++)
        {
            for (int z = 0; z < 20; z++)
            {
                int idx = x + 400 * y + 400 * 400 * z;
                gt_voxel_map.data[idx] = p_test_mapper_->val_free;
            }
        }
    }

    // Finally make the comparison
    ASSERT_EQ(decayed_map.data.size(), gt_voxel_map.data.size());
    for (int idx = 0; idx < num_voxels; idx++)
    {
        EXPECT_EQ(decayed_map.data[idx], gt_voxel_map.data[idx]) << "IDX: " << idx;
    }
}

/**
 * @brief This method is called to process the point cloud from the lidar scans. Currently,
 * raytracing is always set to false, so it is never called. The decayLocalCloud method is called
 * inside this method, but since the decay value is 2, it does not impede a voxel from being marked
 * as occupied after one scan if we include 3 lidar points in the specified voxel. Points are
 * provided with respect to the lidar frame and the transformation that is passed as an argument
 * is the pose of the lidar in the map frame. Any points outside of max_range are discarded.
 * In this test, we simulate the lidar to be at a world coordinate different than (0, 0, 0) and 
 * make a single call to addCloud. This represents a single lidar scan, but to make sure that
 * the voxels corresponding to the points are marked as occupied in this single scan, we triplicate
 * the points. After making the call we compare the normal map and the inflated map to their ground
 * truths.
 */
TEST_F(VoxelMapperTest, TestAddCloud)
{
    // We will consider that the lidar is at position (-10, -5, -1) in the map frame of reference
    Eigen::Vector3d lidar_pos(-10, -5, -1);
    Eigen::Affine3d t_map_lidar = Eigen::Translation3d(-10, -5, -1) *
                                  Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 0));
    double max_range = 30.0;
    vec_Vec3d lidar_points;

    // Add 3 points for all voxels in the range -30 to in all 3 axes. Some will be discarded
    // and only a sperical shape of voxels around (-10, -5, -1) should be marked as occupied
    for (double x = -30; x <= 30; x+=0.5)
    {
        for (double y = -30; y <= 30; y+=0.5)
        {
            for (double z = -30; z <= 30; z+=0.5)
            {
                Eigen::Vector3d point(x, y, z);
                for (int n = 0; n < 3; n++)
                {
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

    time_start = std::chrono::steady_clock::now();
    p_test_mapper_->addCloud(lidar_points, t_map_lidar, neighbors, false, max_range);
    time_end = std::chrono::steady_clock::now();
    std::cout << "CALLBACK Duration (addCloud): " <<
        std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count()
        << "us" << std::endl;
    planning_ros_msgs::VoxelMap processed_map = p_test_mapper_->getMap();
    planning_ros_msgs::VoxelMap processed_inflated_map = p_test_mapper_->getInflatedMap();

    // Create the ground truth voxel maps to compare to the normal map and the inflated map
    planning_ros_msgs::VoxelMap gt_voxel_map;
    planning_ros_msgs::VoxelMap gt_inflated_voxel_map;
    int x_dim = x_dim_ / resolution_;
    int y_dim = y_dim_ / resolution_;
    int z_dim = z_dim_ / resolution_;
    gt_voxel_map.data.resize(x_dim * y_dim * z_dim, gt_voxel_map.val_default);
    gt_inflated_voxel_map.data.resize(x_dim * y_dim * z_dim, gt_voxel_map.val_default);

    // Fill the voxels that should be occupied
    for (int x = 0; x < x_dim; x++)
    {
        for (int y = 0; y < y_dim; y++)
        {
            for (int z = 0; z < z_dim; z++)
            {
                Eigen::Vector3d voxel_pos = Eigen::Vector3d(x, y, z) * 0.5 +
                                            Eigen::Vector3d(-100, -100, -5);  
                if ((voxel_pos - lidar_pos).norm() <= max_range)
                {
                    int idx = x + x_dim * y + x_dim * y_dim * z;
                    gt_voxel_map.data[idx] = gt_voxel_map.val_occ;
                    gt_inflated_voxel_map.data[idx] = gt_voxel_map.val_occ;

                    // Include neighbors for the inflated map
                    Eigen::Vector3i current_voxel(x, y, z);
                    for (auto &neighbor : neighbors)
                    {
                        Eigen::Vector3i neigbor_voxel = current_voxel + neighbor;
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
    for (int idx = 0; idx < num_voxels; idx++)
    {
        EXPECT_EQ(processed_map.data[idx], gt_voxel_map.data[idx]) << "IDX: " << idx;
    }

    // Compare the inflated map
    ASSERT_EQ(processed_inflated_map.data.size(), gt_inflated_voxel_map.data.size());
    for (int idx = 0; idx < num_voxels; idx++)
    {
        EXPECT_EQ(processed_inflated_map.data[idx], gt_inflated_voxel_map.data[idx]) << "IDX: " << idx;
    }
}

/**
 * @brief If no point cloud has been processed, then the inflated map and the normal map
 * are identical. The processing of the cloud is tested separately so here we are just going to
 * make sure that the two maps are indeed identical upon initialization. Initialization is done
 * in the constructor so here we will just compare the maps.
 */
TEST_F(VoxelMapperTest, TestGetInflatedMap)
{
    time_start = std::chrono::steady_clock::now();
    planning_ros_msgs::VoxelMap normal_map = p_test_mapper_->getMap();
    time_end = std::chrono::steady_clock::now();
    std::cout << "CALLBACK Duration (getMap): " <<
        std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count()
        << "us" << std::endl;

    time_start = std::chrono::steady_clock::now();
    planning_ros_msgs::VoxelMap inflated_map = p_test_mapper_->getInflatedMap();
    time_end = std::chrono::steady_clock::now();
    std::cout << "CALLBACK Duration (getInflatedMap): " <<
        std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count()
        << "us" << std::endl;

    int num_voxels = 3200000;
    ASSERT_EQ(normal_map.data.size(), num_voxels);
    ASSERT_EQ(normal_map.data.size(), inflated_map.data.size());
    for (int idx = 0; idx < num_voxels; idx++)
    {
        EXPECT_EQ(normal_map.data[idx], inflated_map.data[idx]) << "IDX: " << idx;
    }
}

/**
 * @brief This method extracts a portion of the world with a world origin and map dimensions
 * provided as arguments. The resolution is the same as the inflated map and it will retrieve the
 * inflated map, similar to the getInflatedMap method. One key difference is that any voxels that
 * are outside the bounds of the inflated map are considered as occupied. In this test, given the
 * map created with all voxels set to default, we extract a local map that is partially outside
 * the bounds of the intial map. So the extracted local map should have the overlapping voxels set
 * to the default value while all other voxels set to occupied.
 */
TEST_F(VoxelMapperTest, TestGetInflatedLocalMap)
{
    // The origin of the local map is moved in the positive direction in all three axes by different
    // amounts. This will cause a portion of the local map to overlap with the original map.
    Eigen::Vector3d origin(75, 25, 2.5);
    Eigen::Vector3d dimensions(100, 100, 10);
    
    time_start = std::chrono::steady_clock::now();
    planning_ros_msgs::VoxelMap local_map = p_test_mapper_->getInflatedLocalMap(origin, dimensions);
    time_end = std::chrono::steady_clock::now();
    std::cout << "CALLBACK Duration (getInflatedLocalMap): " <<
        std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count()
        << "us" << std::endl;

    // Create the ground truth voxel map to compare it to the local map
    planning_ros_msgs::VoxelMap gt_voxel_map;
    int dim_x = dimensions(0) / resolution_;
    int dim_y = dimensions(1) / resolution_;
    int dim_z = dimensions(2) / resolution_;
    gt_voxel_map.data.resize(dim_x * dim_y * dim_z, gt_voxel_map.val_occ);

    for (int x = 0; x < 50; x++)        // 200 - 150 = 50
    {
        for (int y = 0; y < 150; y++)   // 200 - 50 = 150
        {
            for (int z = 0; z < 5; z++) // 10 - 5 = 5
            {
                int idx = x + dim_x * y + dim_x * dim_y * z;
                gt_voxel_map.data[idx] = gt_voxel_map.val_default;
            }
        }
    }

    // Finally make the comparison
    int num_voxels = dim_x * dim_y * dim_z;
    ASSERT_EQ(local_map.data.size(), gt_voxel_map.data.size());
    for (int idx = 0; idx < num_voxels; idx++)
    {
        EXPECT_EQ(local_map.data[idx], gt_voxel_map.data[idx]) << "IDX: " << idx;
    }
}

}//namespace mapper


