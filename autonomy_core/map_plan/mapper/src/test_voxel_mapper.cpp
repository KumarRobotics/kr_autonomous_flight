#include <gtest/gtest.h>
#include <eigen_conversions/eigen_msg.h>
#include "mapper/voxel_mapper.h"
#include <memory>
#include <iostream>

// Defining a fixture class to use the same configuration for a VoxelMapper object
class VoxelMapperTest : public testing::Test
{
protected:
    void SetUp() override
    {
        // Setup the test map here
        const int8_t val_default = 0;
        const int decay_times_to_empty = 0;
        const double resolution = 1.0f;
        // World is 100 x 100 x 100
        const Eigen::Vector3d dimensions(100.0 * resolution,
                                         100.0 * resolution,
                                         100.0 * resolution);
        // Making the origin to be centered around 0.0 in all three axes
        const Eigen::Vector3d origin(-dimensions(0) / 2,
                                     -dimensions(1) / 2,
                                     -dimensions(2) / 2);

        p_test_mapper_.reset(new mapper::VoxelMapper(origin,
                                                     dimensions,
                                                     resolution,
                                                     val_default,
                                                     decay_times_to_empty
                                                     ));
        p_test_mapper_->setMapFree();
        // Add 100 x 100 x 100 points to the vector
        for (int n = 0; n < 1000000; n++)
        {
            neighbors.push_back(Eigen::Vector3i());
        }
    }

    // This is the VoxelMapper object that is used for all the tests
    std::unique_ptr<mapper::VoxelMapper> p_test_mapper_;
    Eigen::Vector3d point{1, 2, 3};
    mapper::vec_Vec3i neighbors;
    std::chrono::time_point<std::chrono::steady_clock> start, end;
    std::chrono::duration<float> duration;
};

TEST_F(VoxelMapperTest, TestGetMap)
{
    // Running for 100 iterations because test clock precision is low
    for (int i = 0; i < 100; i++)
    {
        planning_ros_msgs::VoxelMap vox_map = p_test_mapper_->getMap();
    }
}

TEST_F(VoxelMapperTest, TestFreeVoxels)
{
    // Running for 100 iterations because test clock precision is low
    for (int i = 0; i < 100; i++)
    {
        p_test_mapper_->freeVoxels(point, neighbors);
    }
}


