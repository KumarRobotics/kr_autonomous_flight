#include <gtest/gtest.h>
#include <kr_planning_msgs/VoxelMap.h>
#include <iostream>
#include <cmath>
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


namespace MPL {

/**
 * @brief In this test we verify that voxels outside of the map are identified
 */
TEST(MapUtilTest, TestIsOutside) {
  VoxelMapUtil test_map;  // This is the MapUtil object

  Vec3f origin(-100, -100, -5);
  Vec3i dim(400, 400, 20);    // Amount of voxels along each axis
  double resolution = 0.5;

  // Create a base map for the MapUtil object with val_free as the default value
  std::vector<signed char> base_map(dim(0) * dim(1) * dim(2),
                                    kr_planning_msgs::VoxelMap::val_free);

  // Initialize the MapUtil object with the previously created map
  test_map.setMap(origin, dim, base_map, resolution);

  // Create a set of voxels indices that are outide the bounds of the map
  vec_Vec3i outside_voxels;
  outside_voxels.push_back(Vec3i(-1, 34, 2));
  outside_voxels.push_back(Vec3i(0, 0, 20));
  outside_voxels.push_back(Vec3i(5, 3, -1));
  outside_voxels.push_back(Vec3i(6, -1, 4));
  outside_voxels.push_back(Vec3i(298, 400, 7));

  // Create a set of voxel indices that are inside the bounds of the map
  vec_Vec3i inside_voxels;
  inside_voxels.push_back(Vec3i(2, 5, 7));
  inside_voxels.push_back(Vec3i(0, 0, 0));
  inside_voxels.push_back(Vec3i(399, 399, 19));
  inside_voxels.push_back(Vec3i(288, 179, 0));
  inside_voxels.push_back(Vec3i(367, 118, 8));

  // Check that the isOutside method correctly identifies inside and outside
  // voxels
  for (auto &vox : outside_voxels) {
    EXPECT_TRUE(test_map.isOutside(vox)) << "Voxel: ("
      << vox(0) << ", " << vox(1) << ", " << vox(2) << ")\n";
  }

  for (auto &vox : inside_voxels) {
    EXPECT_FALSE(test_map.isOutside(vox)) << "Voxel: ("
      << vox(0) << ", " << vox(1) << ", " << vox(2) << ")\n";
  }
}

/**
 * @brief The isFree method returns a boolean value. In this test we initialize
 * the map in the MapUtil object to have all of its voxels as free, then we set
 * a few voxels to be non-free and finally verify that isFree can identify them
 */
TEST(MapUtilTest, TestIsFree) {
  VoxelMapUtil test_map;  // This is the MapUtil object

  Vec3f origin(-100, -100, -5);
  Vec3i dim(400, 400, 20);    // Amount of voxels along each axis
  double resolution = 0.5;

  // Create a base map for the MapUtil object with val_free as the default value
  std::vector<signed char> base_map(dim(0) * dim(1) * dim(2),
                                    kr_planning_msgs::VoxelMap::val_free);

  vec_Vec3i non_free_voxels;
  non_free_voxels.push_back(Vec3i(176, 91, 6));
  non_free_voxels.push_back(Vec3i(102, 157, 12));
  non_free_voxels.push_back(Vec3i(18, 261, 3));
  non_free_voxels.push_back(Vec3i(253, 137, 4));
  non_free_voxels.push_back(Vec3i(123, 158, 16));
  for (auto &vox : non_free_voxels) {
    int idx = vox(0) + dim(0) * vox(1) + dim(0) * dim(1) * vox(2);
    base_map[idx] = kr_planning_msgs::VoxelMap::val_occ;
  }

  // Initialize the MapUtil object with the previously created map
  test_map.setMap(origin, dim, base_map, resolution);

  // Create a set of voxels indices that are outide the bounds of the map
  vec_Vec3i outside_voxels;
  outside_voxels.push_back(Vec3i(-1, 34, 2));
  outside_voxels.push_back(Vec3i(0, 0, 20));
  outside_voxels.push_back(Vec3i(5, 3, -1));
  outside_voxels.push_back(Vec3i(6, -1, 4));
  outside_voxels.push_back(Vec3i(298, 400, 7));

  // Create a set of voxel indices that are free
  vec_Vec3i free_voxels;
  free_voxels.push_back(Vec3i(2, 5, 7));
  free_voxels.push_back(Vec3i(0, 0, 0));
  free_voxels.push_back(Vec3i(399, 399, 19));
  free_voxels.push_back(Vec3i(288, 179, 0));
  free_voxels.push_back(Vec3i(367, 118, 8));

  // Check that the isFree method correctly identifies free and non-free voxels.
  // It should return false for any out of bounds voxels
  for (auto &vox : outside_voxels) {
    EXPECT_FALSE(test_map.isFree(vox)) << "Voxel: ("
      << vox(0) << ", " << vox(1) << ", " << vox(2) << ")\n";
  }

  for (auto &vox : free_voxels) {
    EXPECT_TRUE(test_map.isFree(vox)) << "Voxel: ("
      << vox(0) << ", " << vox(1) << ", " << vox(2) << ")\n";
  }

  for (auto &vox : non_free_voxels) {
    EXPECT_FALSE(test_map.isFree(vox)) << "Voxel: ("
      << vox(0) << ", " << vox(1) << ", " << vox(2) << ")\n";
  }
}

/**
 * @brief The isOccupied method returns a boolean value. In this test we
 * initialize the map in the MapUtil object to have all of its voxels as
 * occupied, then we set a few voxels to be non-occupied and finally verify that
 * isOccupied can identify them
 */
TEST(MapUtilTest, TestIsOccupied) {
  VoxelMapUtil test_map;  // This is the MapUtil object

  Vec3f origin(-100, -100, -5);
  Vec3i dim(400, 400, 20);    // Amount of voxels along each axis
  double resolution = 0.5;

  // Create a base map for the MapUtil object with val_occ as the default value
  std::vector<signed char> base_map(dim(0) * dim(1) * dim(2),
                                    kr_planning_msgs::VoxelMap::val_occ);

  vec_Vec3i non_occ_voxels;
  non_occ_voxels.push_back(Vec3i(176, 91, 6));
  non_occ_voxels.push_back(Vec3i(102, 157, 12));
  non_occ_voxels.push_back(Vec3i(18, 261, 3));
  non_occ_voxels.push_back(Vec3i(253, 137, 4));
  non_occ_voxels.push_back(Vec3i(123, 158, 16));
  for (auto &vox : non_occ_voxels) {
    int idx = vox(0) + dim(0) * vox(1) + dim(0) * dim(1) * vox(2);
    base_map[idx] = kr_planning_msgs::VoxelMap::val_free;
  }

  // Initialize the MapUtil object with the previously created map
  test_map.setMap(origin, dim, base_map, resolution);

  // Create a set of voxels indices that are outide the bounds of the map
  vec_Vec3i outside_voxels;
  outside_voxels.push_back(Vec3i(-1, 34, 2));
  outside_voxels.push_back(Vec3i(0, 0, 20));
  outside_voxels.push_back(Vec3i(5, 3, -1));
  outside_voxels.push_back(Vec3i(6, -1, 4));
  outside_voxels.push_back(Vec3i(298, 400, 7));

  // Create a set of voxel indices that are occupied
  vec_Vec3i occ_voxels;
  occ_voxels.push_back(Vec3i(2, 5, 7));
  occ_voxels.push_back(Vec3i(0, 0, 0));
  occ_voxels.push_back(Vec3i(399, 399, 19));
  occ_voxels.push_back(Vec3i(288, 179, 0));
  occ_voxels.push_back(Vec3i(367, 118, 8));

  // Check that the isOccupied method correctly identifies occupied and
  // non-occupied voxels. It should return false for any out of bounds voxels
  for (auto &vox : outside_voxels) {
    EXPECT_FALSE(test_map.isOccupied(vox)) << "Voxel: ("
      << vox(0) << ", " << vox(1) << ", " << vox(2) << ")\n";
  }

  for (auto &vox : occ_voxels) {
    EXPECT_TRUE(test_map.isOccupied(vox)) << "Voxel: ("
      << vox(0) << ", " << vox(1) << ", " << vox(2) << ")\n";
  }

  for (auto &vox : non_occ_voxels) {
    EXPECT_FALSE(test_map.isOccupied(vox)) << "Voxel: ("
      << vox(0) << ", " << vox(1) << ", " << vox(2) << ")\n";
  }
}

/**
 * @brief The isUnknown method returns a boolean value. In this test we
 * initialize the map in the MapUtil object to have all of its voxels as
 * unknown, then we set a few voxels to be non-unknown and finally verify that
 * isUnknown can identify them.
 */
TEST(MapUtilTest, TestIsUnknown) {
  VoxelMapUtil test_map;  // This is the MapUtil object

  Vec3f origin(-100, -100, -5);
  Vec3i dim(400, 400, 20);    // Amount of voxels along each axis
  double resolution = 0.5;

  // Create a base map for the MapUtil object with val_unknown as the
  // default value
  std::vector<signed char> base_map(dim(0) * dim(1) * dim(2),
                                    kr_planning_msgs::VoxelMap::val_unknown);

  vec_Vec3i non_unknown_voxels;
  non_unknown_voxels.push_back(Vec3i(176, 91, 6));
  non_unknown_voxels.push_back(Vec3i(102, 157, 12));
  non_unknown_voxels.push_back(Vec3i(18, 261, 3));
  non_unknown_voxels.push_back(Vec3i(253, 137, 4));
  non_unknown_voxels.push_back(Vec3i(123, 158, 16));
  for (auto &vox : non_unknown_voxels) {
    int idx = vox(0) + dim(0) * vox(1) + dim(0) * dim(1) * vox(2);
    base_map[idx] = kr_planning_msgs::VoxelMap::val_occ;
  }

  // Initialize the MapUtil object with the previously created map
  test_map.setMap(origin, dim, base_map, resolution);

  // Create a set of voxels indices that are outide the bounds of the map
  vec_Vec3i outside_voxels;
  outside_voxels.push_back(Vec3i(-1, 34, 2));
  outside_voxels.push_back(Vec3i(0, 0, 20));
  outside_voxels.push_back(Vec3i(5, 3, -1));
  outside_voxels.push_back(Vec3i(6, -1, 4));
  outside_voxels.push_back(Vec3i(298, 400, 7));

  // Create a set of voxel indices that are unknown
  vec_Vec3i unknown_voxels;
  unknown_voxels.push_back(Vec3i(2, 5, 7));
  unknown_voxels.push_back(Vec3i(0, 0, 0));
  unknown_voxels.push_back(Vec3i(399, 399, 19));
  unknown_voxels.push_back(Vec3i(288, 179, 0));
  unknown_voxels.push_back(Vec3i(367, 118, 8));

  // Check that the isUnknown method correctly identifies unknown and
  // non-unknown voxels. It should return false for any out of bounds voxels
  for (auto &vox : outside_voxels) {
    EXPECT_FALSE(test_map.isUnknown(vox)) << "Voxel: ("
      << vox(0) << ", " << vox(1) << ", " << vox(2) << ")\n";
  }

  for (auto &vox : unknown_voxels) {
    EXPECT_TRUE(test_map.isUnknown(vox)) << "Voxel: ("
      << vox(0) << ", " << vox(1) << ", " << vox(2) << ")\n";
  }

  for (auto &vox : non_unknown_voxels) {
    EXPECT_FALSE(test_map.isUnknown(vox)) << "Voxel: ("
      << vox(0) << ", " << vox(1) << ", " << vox(2) << ")\n";
  }
}

/**
 * @brief MapUtil considers voxels to be axis-aligned. In this test we make
 * sure that the conversion from world coordinates to voxel indices and
 * vice-versa are reciprocal and consistent. We check the use of the
 * floatToInt and intToFloat functions.
 */
TEST(MapUtilTest, TestVoxelIndexing) {
  // Create map_util object
  VoxelMapUtil test_map;
  // Dimensions are the amount of voxels in each axis
  test_map.setMap(Vec3f(-100, -100, -5), Vec3i(400, 400, 20), {}, 0.5);

  // Initialize set of points for converstion
  vec_Vec3f point_cloud;      // Test point cloud
  vec_Vec3f gt_point_cloud;   // Values that should be returned by intToFloat
  vec_Vec3i voxel_indices;    // Values that should be returned by floatToInt

  // Test most negative limit of the world
  point_cloud.push_back(Vec3f(-100, -100, -5));
  gt_point_cloud.push_back(Vec3f(-99.75, -99.75, -4.75));
  voxel_indices.push_back(Vec3i(0, 0, 0));

  // Test most positive limit of the world
  point_cloud.push_back(Vec3f(100, 100, 5));
  gt_point_cloud.push_back(Vec3f(100.25, 100.25, 5.25));
  voxel_indices.push_back(Vec3i(400, 400, 20));

  // Test outside the bounds of the world in the positive direction
  point_cloud.push_back(Vec3f(105, 105, 10));
  gt_point_cloud.push_back(Vec3f(105.25, 105.25, 10.25));
  voxel_indices.push_back(Vec3i(410, 410, 30));

  // Test outside the bounds of the world in the most negative direction
  point_cloud.push_back(Vec3f(-105, -105, -10));
  gt_point_cloud.push_back(Vec3f(-104.75, -104.75, -9.75));
  voxel_indices.push_back(Vec3i(-10, -10, -10));

  point_cloud.push_back(Vec3f(-100.1, -100.6, -4.9));
  gt_point_cloud.push_back(Vec3f(-100.25, -100.75, -4.75));
  voxel_indices.push_back(Vec3i(-1, -2, 0));

  // Check points near the voxel boundaries to make sure that voxels are
  // actually axis-aligned and not origin-centered
  point_cloud.push_back(Vec3f(10.1, 10.1, 2));
  gt_point_cloud.push_back(Vec3f(10.25, 10.25, 2.25));
  voxel_indices.push_back(Vec3i(220, 220, 14));

  point_cloud.push_back(Vec3f(10.9, 10.9, 2));
  gt_point_cloud.push_back(Vec3f(10.75, 10.75, 2.25));
  voxel_indices.push_back(Vec3i(221, 221, 14));

  point_cloud.push_back(Vec3f(5.6, 5.6, 3));
  gt_point_cloud.push_back(Vec3f(5.75, 5.75, 3.25));
  voxel_indices.push_back(Vec3i(211, 211, 16));

  point_cloud.push_back(Vec3f(5.4, 5.4, 3));
  gt_point_cloud.push_back(Vec3f(5.25, 5.25, 3.25));
  voxel_indices.push_back(Vec3i(210, 210, 16));

  for (int i = 0; i < point_cloud.size(); i++) {
    auto &point = point_cloud[i];
    auto &gt_point = gt_point_cloud[i];
    auto &voxel = voxel_indices[i];
    Vec3i ret_indices = test_map.floatToInt(point);
    Vec3f ret_point = test_map.intToFloat(ret_indices);

    // Compare the returned indices to the actual voxel indices
    for (int n = 0; n < 3; n++) {
      EXPECT_EQ(voxel(n), ret_indices(n)) << "Point idx: " << i
        << "  Voxel: (" << voxel(0) << ", " << voxel(1) << ", "
        << voxel(2) << ")" << std::endl;
    }

    // Compare the original point to the returned point
    for (int n = 0; n < 3; n++) {
      EXPECT_DOUBLE_EQ(gt_point(n), ret_point(n)) << "Point idx: " << i
        << "  Point: (" << point(0) << ", " << point(1) << ", "
        << point(2) << ")" << std::endl;
    }
  }
}

/**
 * @brief The getCloud method will return a vector of points (centered at the
 * origin of voxels) corresponding to occupied voxels. In this test we initalize
 * a map to have non-occupied voxels, then we set a particular set of voxels as
 * occupied and verify that the corresponding point cloud is returned
 */
TEST(MapUtilTest, TestGetCloud) {
  VoxelMapUtil test_map;  // This is the MapUtil object

  Vec3f origin(-100, -100, -5);
  Vec3i dim(400, 400, 20);    // Amount of voxels along each axis
  double resolution = 0.5;

  // Create a base map for the MapUtil object with val_free as the default value
  std::vector<signed char> base_map(dim(0) * dim(1) * dim(2),
                                    kr_planning_msgs::VoxelMap::val_free);

  vec_Vec3i occ_voxels;
  vec_Vec3f occ_cloud;
  occ_voxels.push_back(Vec3i(18, 261, 3));
  occ_cloud.push_back(Vec3f(-90.75, 30.75, -3.25));

  occ_voxels.push_back(Vec3i(102, 157, 12));
  occ_cloud.push_back(Vec3f(-48.75, -21.25, 1.25));

  occ_voxels.push_back(Vec3i(123, 158, 16));
  occ_cloud.push_back(Vec3f(-38.25, -20.75, 3.25));

  occ_voxels.push_back(Vec3i(176, 91, 6));
  occ_cloud.push_back(Vec3f(-11.75, -54.25, -1.75));

  occ_voxels.push_back(Vec3i(253, 137, 4));
  occ_cloud.push_back(Vec3f(26.75, -31.25, -2.75));
  for (auto &vox : occ_voxels) {
    int idx = vox(0) + dim(0) * vox(1) + dim(0) * dim(1) * vox(2);
    base_map[idx] = kr_planning_msgs::VoxelMap::val_occ;
  }

  // Initialize the MapUtil object with the previously created map
  test_map.setMap(origin, dim, base_map, resolution);

  vec_Vec3f ret_cloud = test_map.getCloud();
  std::sort(ret_cloud.begin(), ret_cloud.end(), comparePoints);

  ASSERT_EQ(occ_cloud.size(), ret_cloud.size());
  // Iterate over every point
  for (int i = 0; i < occ_cloud.size(); i++) {
    // Iterate over every index in each point
    for (int n = 0; n < 3; n++) {
      EXPECT_DOUBLE_EQ(occ_cloud[i][n], ret_cloud[i][n])
        << "IDX: " << i << ", Axis: " << n << std::endl;
    }
  }
}

/**
 * @brief The getFreeCloud method will return a vector of points (centered at
 * the origin of voxels) corresponding to free voxels. In this test we initalize
 * a map to have non-free voxels, then we set a particular set of voxels as
 * free and verify that the corresponding point cloud is returned
 */
TEST(MapUtilTest, TestGetFreeCloud) {
  VoxelMapUtil test_map;  // This is the MapUtil object

  Vec3f origin(-100, -100, -5);
  Vec3i dim(400, 400, 20);    // Amount of voxels along each axis
  double resolution = 0.5;

  // Create a base map for the MapUtil object with val_occ as the default value
  std::vector<signed char> base_map(dim(0) * dim(1) * dim(2),
                                    kr_planning_msgs::VoxelMap::val_occ);

  vec_Vec3i free_voxels;
  vec_Vec3f free_cloud;
  free_voxels.push_back(Vec3i(18, 261, 3));
  free_cloud.push_back(Vec3f(-90.75, 30.75, -3.25));

  free_voxels.push_back(Vec3i(102, 157, 12));
  free_cloud.push_back(Vec3f(-48.75, -21.25, 1.25));

  free_voxels.push_back(Vec3i(123, 158, 16));
  free_cloud.push_back(Vec3f(-38.25, -20.75, 3.25));

  free_voxels.push_back(Vec3i(176, 91, 6));
  free_cloud.push_back(Vec3f(-11.75, -54.25, -1.75));

  free_voxels.push_back(Vec3i(253, 137, 4));
  free_cloud.push_back(Vec3f(26.75, -31.25, -2.75));
  for (auto &vox : free_voxels) {
    int idx = vox(0) + dim(0) * vox(1) + dim(0) * dim(1) * vox(2);
    base_map[idx] = kr_planning_msgs::VoxelMap::val_free;
  }

  // Initialize the MapUtil object with the previously created map
  test_map.setMap(origin, dim, base_map, resolution);

  vec_Vec3f ret_cloud = test_map.getFreeCloud();
  std::sort(ret_cloud.begin(), ret_cloud.end(), comparePoints);

  ASSERT_EQ(free_cloud.size(), ret_cloud.size());
  // Iterate over every point
  for (int i = 0; i < free_cloud.size(); i++) {
    // Iterate over every index in each point
    for (int n = 0; n < 3; n++) {
      EXPECT_DOUBLE_EQ(free_cloud[i][n], ret_cloud[i][n])
        << "IDX: " << i << ", Axis: " << n << std::endl;
    }
  }
}

/**
 * @brief The getUnknownCloud method will return a vector of points (centered at
 * the origin of voxels) corresponding to unknown voxels. In this test we
 * initalize a map to have non-unknown voxels, then we set a particular set of
 * voxels as unknown and verify that the corresponding point cloud is returned
 */
TEST(MapUtilTest, TestGetUnkwownCloud) {
  VoxelMapUtil test_map;  // This is the MapUtil object

  Vec3f origin(-100, -100, -5);
  Vec3i dim(400, 400, 20);    // Amount of voxels along each axis
  double resolution = 0.5;

  // Create a base map for the MapUtil object with val_free as the default value
  std::vector<signed char> base_map(dim(0) * dim(1) * dim(2),
                                    kr_planning_msgs::VoxelMap::val_free);

  vec_Vec3i unknown_voxels;
  vec_Vec3f unknown_cloud;
  unknown_voxels.push_back(Vec3i(18, 261, 3));
  unknown_cloud.push_back(Vec3f(-90.75, 30.75, -3.25));

  unknown_voxels.push_back(Vec3i(102, 157, 12));
  unknown_cloud.push_back(Vec3f(-48.75, -21.25, 1.25));

  unknown_voxels.push_back(Vec3i(123, 158, 16));
  unknown_cloud.push_back(Vec3f(-38.25, -20.75, 3.25));

  unknown_voxels.push_back(Vec3i(176, 91, 6));
  unknown_cloud.push_back(Vec3f(-11.75, -54.25, -1.75));

  unknown_voxels.push_back(Vec3i(253, 137, 4));
  unknown_cloud.push_back(Vec3f(26.75, -31.25, -2.75));
  for (auto &vox : unknown_voxels) {
    int idx = vox(0) + dim(0) * vox(1) + dim(0) * dim(1) * vox(2);
    base_map[idx] = kr_planning_msgs::VoxelMap::val_unknown;
  }

  // Initialize the MapUtil object with the previously created map
  test_map.setMap(origin, dim, base_map, resolution);

  vec_Vec3f ret_cloud = test_map.getUnknownCloud();
  std::sort(ret_cloud.begin(), ret_cloud.end(), comparePoints);

  ASSERT_EQ(unknown_cloud.size(), ret_cloud.size());
  // Iterate over every point
  for (int i = 0; i < unknown_cloud.size(); i++) {
    // Iterate over every index in each point
    for (int n = 0; n < 3; n++) {
      EXPECT_DOUBLE_EQ(unknown_cloud[i][n], ret_cloud[i][n])
        << "IDX: " << i << ", Axis: " << n << std::endl;
    }
  }
}

}   // namespace MPL


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


