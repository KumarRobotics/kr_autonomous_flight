#include <gtest/gtest.h>
#include <eigen_conversions/eigen_msg.h>
#include "mapper/voxel_mapper.h"
#include <memory>
#include <math.h>
#include <planning_ros_msgs/VoxelMap.h>
#include <iostream>


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
    /*****  Part1: Test that map is created with correct dimensions  *****/
    // Creating voxel mapper object with same parameters as global_voxel_mapper in source file:
    // local_global_mapper.cpp
    Eigen::Vector3d origin(-100, -100, -5);
    Eigen::Vector3d dimensions(200, 200, 10);
    p_test_mapper_.reset(new mapper::VoxelMapper(origin, dimensions, 0.5, 0, 30));
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


    /*****  Part2: Now test whether the relocation is working  *****/
    // Create voxel mapper object
    origin = Eigen::Vector3d(-100, -100, -5);
    dimensions = Eigen::Vector3d(200, 200, 10);
    int num_voxels = 3200000;
    p_test_mapper_.reset(new mapper::VoxelMapper(origin, dimensions, 0.5, 0, 30));

    // Mark all voxels as occupied. This original map, in world coordinates, ranges from:
    // x: -100 to 100
    // y: -100 to 100
    // z: -5 to 5
    // In voxels this translates to:
    // x: 200 voxels from -100 to 0 and 200 voxels from 0 to 100
    // y: 200 voxels from -100 to 0 and 200 voxels from 0 to 100
    // z: 10 voxels from -5 to 0 and 10 voxels from 0 to 5
    std::fill(p_test_mapper_->map_.data(), p_test_mapper_->map_.data() + num_voxels,
              p_test_mapper_->val_occ);

    // Relocating map in the positive direction in all three axes centered at (0,0,0) and
    // reducing x-dim and y-dim by 50. With the same resolution this means 100 less voxels in each
    // dimension. z stays the same
    Eigen::Vector3d new_origin(0, 0, 0);
    Eigen::Vector3d new_dimensions(150, 150, 10);
    // True means that the relocating actually happened
    EXPECT_TRUE(p_test_mapper_->allocate(new_dimensions, new_origin));
    planning_ros_msgs::VoxelMap relocated_map = p_test_mapper_->getMap();

    // Create the ground truth voxel map to compare the relocated map to
    planning_ros_msgs::VoxelMap gt_voxel_map;
    gt_voxel_map.origin.x = 0; 
    gt_voxel_map.origin.y = 0; 
    gt_voxel_map.origin.z = 0; 
    gt_voxel_map.dim.x = 150 / 0.5;     // new map should have 300 voxels in x dimension
    gt_voxel_map.dim.y = 150 / 0.5;     // new map should have 300 voxels in y dimension
    gt_voxel_map.dim.z = 10 / 0.5;      // new map should have 20 voxels in z dimension
    gt_voxel_map.resolution = 0.5;
    gt_voxel_map.header.frame_id = "map";
    gt_voxel_map.data.resize(gt_voxel_map.dim.x * gt_voxel_map.dim.y * gt_voxel_map.dim.z,
                             p_test_mapper_->val_default);

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

TEST_F(VoxelMapperTest, TestDecayLocalCloud)
{
}

TEST_F(VoxelMapperTest, TestAddCloud)
{
}

TEST_F(VoxelMapperTest, TestGetInflatedMap)
{
}

TEST_F(VoxelMapperTest, TestGetInflatedLocalMap)
{
}

}//namespace mapper


