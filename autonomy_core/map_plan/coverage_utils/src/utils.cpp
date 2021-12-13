
#include "utils.h"

#include <fstream>
#include <iostream>
#include <vector>

using namespace std;

// Functions for convex_hull calculation are from
// https://www.topcoder.com/blog/problem-of-the-week-save-the-trees/
bool cmp(pt a, pt b) { return a.x < b.x || (a.x == b.x && a.y < b.y); }

bool cw(pt a, pt b, pt c) {
  return a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y) < 0;
}

bool ccw(pt a, pt b, pt c) {
  return a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y) > 0;
}

void convex_hull(vector<pt>& a) {
  if (a.size() == 1) return;

  sort(a.begin(), a.end(), &cmp);
  pt p1 = a[0], p2 = a.back();
  vector<pt> up, down;
  up.push_back(p1);
  down.push_back(p1);
  for (int i = 1; i < (int)a.size(); i++) {
    if (i == a.size() - 1 || cw(p1, a[i], p2)) {
      while (up.size() >= 2 && !cw(up[up.size() - 2], up[up.size() - 1], a[i]))
        up.pop_back();
      up.push_back(a[i]);
    }
    if (i == a.size() - 1 || ccw(p1, a[i], p2)) {
      while (down.size() >= 2 &&
             !ccw(down[down.size() - 2], down[down.size() - 1], a[i]))
        down.pop_back();
      down.push_back(a[i]);
    }
  }

  a.clear();
  for (int i = 0; i < (int)up.size(); i++) a.push_back(up[i]);
  for (int i = down.size() - 2; i > 0; i--) a.push_back(down[i]);
}

vector<pt> PreprocessData() {
  ifstream in(
      "/home/sam/autonomy_stack/autonomy_core/map_plan/coverage_utils/config/"
      "input.txt");

  cout << "TODO: data path is hardcoded" << endl;
  double data1, data2;
  char delimiter_symbol;
  double min_x, min_y;
  double average_x, average_y;
  int num_data = 0;
  int idx = 0;

  vector<pt> pt_vec;
  while (in >> data1 >> delimiter_symbol >> data2) {
    idx += 1;
    if (idx == 1) {
      // skip first line which is the header
      cout << "skipping the first line which is header..."
           << "/n";
      continue;
    }

    if (idx == 2) {
      // first data point, intialize min_x and min_y
      min_x = data1;
      min_y = data2;
    }

    average_x += data1;
    average_y += data2;
    if (data1 < min_x) {
      min_x = data1;
    }
    if (data2 < min_y) {
      min_y = data2;
    }
    num_data++;

    pt cur_pt;
    cur_pt.x = data1;
    cur_pt.y = data2;
    pt_vec.push_back(cur_pt);
  }
  if (num_data == 0) {
    cout << "No tree position data found in the file!!!!!!" << '\n';
  }
  cout << "total tree positions extracted: " << num_data << '\n';
  in.close();

  // average_x = average_x / double(num_data);
  // average_y = average_y / double(num_data);
  // cout << "data point average x and y: " << average_x << " and " << average_y
  //      << endl;

  // vector<pt> pt_vec;
  // ifstream in2(
  //     "/home/sam/autonomy_stack/autonomy_core/map_plan/coverage_utils/config/"
  //     "input.txt");
  // while (in2 >> data1 && in2 >> data2) {
  //   pt cur_pt;
  //   // cur_pt.x = data1 - average_x;
  //   // cur_pt.y = data2 - average_y;
  //   cur_pt.x = data1 - average_x;  // move one axis to 0
  //   cur_pt.y = data2 - min_y;      // move y to center
  //   pt_vec.push_back(cur_pt);
  // }
  // in2.close();

  return pt_vec;
}

planning_ros_msgs::Path path_to_ros(const vec_Vec3f& path, double h) {
  planning_ros_msgs::Path msg;
  for (const auto& itt : path) {
    geometry_msgs::Point this_pt;
    this_pt.x = itt(0);
    this_pt.y = itt(1);
    this_pt.z = h;
    msg.waypoints.push_back(this_pt);
  }
  return msg;
}
