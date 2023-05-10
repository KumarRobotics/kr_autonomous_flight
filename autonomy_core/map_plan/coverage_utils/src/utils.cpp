#include <utils.h>

#include <fstream>
#include <iostream>
#include <vector>

// Functions for convex_hull calculation are from
// https://www.topcoder.com/blog/problem-of-the-week-save-the-trees/

bool cmp(pt a, pt b) { return a.x < b.x || (a.x == b.x && a.y < b.y); }

bool cw(pt a, pt b, pt c) {
  return a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y) < 0;
}

bool ccw(pt a, pt b, pt c) {
  return a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y) > 0;
}

void convex_hull(std::vector<pt>* pts) {
  if ((*pts).size() == 1) return;

  sort((*pts).begin(), (*pts).end(), &cmp);
  pt p1 = (*pts)[0], p2 = (*pts).back();
  std::vector<pt> up, down;
  up.push_back(p1);
  down.push_back(p1);
  for (int i = 1; i < static_cast<int>((*pts).size()); i++) {
    if (i == (*pts).size() - 1 || cw(p1, (*pts)[i], p2)) {
      while (up.size() >= 2 &&
             !cw(up[up.size() - 2], up[up.size() - 1], (*pts)[i]))
        up.pop_back();
      up.push_back((*pts)[i]);
    }
    if (i == (*pts).size() - 1 || ccw(p1, (*pts)[i], p2)) {
      while (down.size() >= 2 &&
             !ccw(down[down.size() - 2], down[down.size() - 1], (*pts)[i]))
        down.pop_back();
      down.push_back((*pts)[i]);
    }
  }

  (*pts).clear();
  for (int i = 0; i < static_cast<int>(up.size()); i++) (*pts).push_back(up[i]);
  for (int i = down.size() - 2; i > 0; i--) (*pts).push_back(down[i]);
}

std::vector<pt> PreprocessData(const std::string& fname) {
  std::ifstream in(fname);

  std::cout << "reading data from:" << fname << '\n';
  double data1, data2;
  char delimiter_symbol;
  double min_x, min_y;
  double average_x, average_y;
  int num_data = 0;
  int idx = 0;

  std::vector<pt> pt_vec;
  while (in >> data1 >> delimiter_symbol >> data2) {
    idx += 1;
    if (idx == 1) {
      // skip first line which is the header
      std::cout << "skipping the first line which is header..."
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
    std::cout << "No tree position data found in the file!!!!!!" << '\n';
    std::cout << "No tree position data found in the file!!!!!!" << '\n';
    std::cout << "No tree position data found in the file!!!!!!" << '\n';
  }
  std::cout << "total tree positions extracted: " << num_data << '\n';
  in.close();

  return pt_vec;
}

kr_planning_msgs::Path path_to_ros(const vec_Vec3f& path, double h) {
  kr_planning_msgs::Path msg;
  for (const auto& itt : path) {
    geometry_msgs::Point this_pt;
    this_pt.x = itt(0);
    this_pt.y = itt(1);
    this_pt.z = h;
    msg.waypoints.push_back(this_pt);
  }
  return msg;
}
