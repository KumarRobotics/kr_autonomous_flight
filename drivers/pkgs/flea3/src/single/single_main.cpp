#include "flea3/single_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "flea3_single");
  ros::NodeHandle pnh("~");

  try {
    flea3::SingleNode single_node(pnh);
    single_node.Run();
    ros::spin();
    single_node.End();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
