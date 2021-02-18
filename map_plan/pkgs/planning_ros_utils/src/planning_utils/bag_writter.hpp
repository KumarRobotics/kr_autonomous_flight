#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

template <typename T>
void write_bag(std::string file_name, std::string topic_name,
               const std::vector<T>& msgs) {
  rosbag::Bag bag;
  bag.open(file_name, rosbag::bagmode::Write);

  for (const auto& it : msgs) bag.write(topic_name, it.header.stamp, it);
  ROS_INFO("Write %zu messages as %s in file %s", msgs.size(),
           topic_name.c_str(), file_name.c_str());
  bag.close();
}
