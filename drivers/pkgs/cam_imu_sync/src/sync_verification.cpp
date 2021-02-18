/*
 * Copyright [2015] [Ke Sun  sunke.polyu@gmail.com]
 *                  [Chao Qu quchao@seas.upenn.edu]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <feature_detector/ShiFeatureDetector.h>
#include <feature_tracker/KltFeatureTracker.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_utils/VisionUtils.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/foreach.hpp>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace fd = feature_detector;
namespace ft = feature_tracker;
namespace vu = vision_utils;

// Names of bagfiles for input and output
std::string read_bagname;
std::string write_bagname;
// Topics of interest
std::string imu_topic;
std::string img_topic;
std::string imu_angular_vel_topic;
std::string cam_angular_vel_topic;
// Calibration paramters
cv::Mat cam_intrinsic;
Eigen::Matrix3d offset_R;
// Related to visual rotation estimation
cv_bridge::CvImagePtr prev_img_ptr;
cv_bridge::CvImagePtr curr_img_ptr;
std::vector<cv::Point2f> prev_features;
std::vector<cv::Point2f> curr_features;
std::vector<unsigned char> tracked_flags;
fd::ShiFeatureDetector f_detector;
ft::KltFeatureTracker f_tracker;
double ransac_th;
double ransac_confidence;

bool loadParameters() {
  std::string ns("/sync_test/");

  if (!ros::param::get(ns + "input_bagfile_name", read_bagname)) return false;
  if (!ros::param::get(ns + "output_bagfile_name", write_bagname)) return false;
  if (!ros::param::get(ns + "input_imu_topic", imu_topic)) return false;
  if (!ros::param::get(ns + "input_img_topic", img_topic)) return false;
  if (!ros::param::get(ns + "output_imu_angular_vel_topic",
                       imu_angular_vel_topic))
    return false;
  if (!ros::param::get(ns + "output_cam_angular_vel_topic",
                       cam_angular_vel_topic))
    return false;

  if (!ros::param::get(ns + "ransac_threshold", ransac_th)) return false;
  if (!ros::param::get(ns + "ransac_confidence", ransac_confidence))
    return false;

  double fx, fy, cx, cy, skew;
  if (!ros::param::get(ns + "fx", fx)) return false;
  if (!ros::param::get(ns + "fy", fy)) return false;
  if (!ros::param::get(ns + "cx", cx)) return false;
  if (!ros::param::get(ns + "cy", cy)) return false;
  if (!ros::param::get(ns + "skew", skew)) return false;
  cam_intrinsic = cv::Mat::zeros(3, 3, CV_64F);
  cam_intrinsic.at<double>(0, 0) = fx;
  cam_intrinsic.at<double>(1, 1) = fy;
  cam_intrinsic.at<double>(0, 2) = cx;
  cam_intrinsic.at<double>(1, 2) = cy;
  cam_intrinsic.at<double>(0, 1) = skew;
  cam_intrinsic.at<double>(2, 2) = 1.0f;

  std::cout << "fx: " << fx << std::endl;
  std::cout << "fy: " << fy << std::endl;
  std::cout << "cx: " << cx << std::endl;
  std::cout << "cy: " << cy << std::endl;
  std::cout << "sk: " << skew << std::endl;

  double roll, pitch, yaw;
  if (!ros::param::get(ns + "ic_offset_roll", roll)) return false;
  if (!ros::param::get(ns + "ic_offset_pitch", pitch)) return false;
  if (!ros::param::get(ns + "ic_offset_yaw", yaw)) return false;
  offset_R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ());

  if (!f_detector.populate(ns)) return false;
  if (!f_tracker.populate(ns)) return false;

  return true;
}

void showImgs() {
  cv::Mat drawing_pad;
  cv::cvtColor(curr_img_ptr->image, drawing_pad, CV_GRAY2BGR);

  // Draw current features on the current frame
  for (int i = 0; i < curr_features.size(); ++i) {
    cv::circle(drawing_pad, curr_features[i], 3, cv::Scalar(0, 255, 0), -1);
  }

  // Draw lines between previous features and
  // current features
  for (int i = 0; i < curr_features.size(); ++i) {
    cv::line(drawing_pad, prev_features[i], curr_features[i],
             cv::Scalar(255, 0, 0));
  }

  // Show the drawing results
  cv::imshow("Tracking Results", drawing_pad);
  cv::waitKey(10);
  return;
}

bool computeAngularVelocity(const sensor_msgs::Image::ConstPtr& mptr,
                            Eigen::Vector3d& ang_vel) {
  // See if the previous image is set
  if (prev_img_ptr) {
    prev_features.clear();
    curr_features.clear();
    tracked_flags.clear();
    // Copy the current image
    curr_img_ptr =
        cv_bridge::toCvCopy(mptr, sensor_msgs::image_encodings::MONO8);
    // Detect features on the previous image
    f_detector.detect(prev_img_ptr->image, prev_features);
    // Tracked the detected features on the current image
    f_tracker.track(prev_img_ptr->image, curr_img_ptr->image, prev_features,
                    curr_features, tracked_flags);
    // Remove the untracked features
    std::vector<cv::Point2f>::iterator prev_iter = prev_features.begin();
    for (int i = 0; i < tracked_flags.size(); ++i) {
      if (tracked_flags[i] == 0) {
        prev_features.erase(prev_iter);
        continue;
      }
      ++prev_iter;
    }
    showImgs();
    // Compute the relative rotation between the two
    // frames by epipolar geometry.
    cv::Mat opencv_R(3, 3, CV_64F);
    cv::Mat opencv_t(3, 1, CV_64F);
    vu::findRTfromE(prev_features, curr_features, ransac_th, ransac_confidence,
                    cam_intrinsic, cam_intrinsic, opencv_R, opencv_t);

    // Computer time interval between two frames
    double dt =
        curr_img_ptr->header.stamp.toSec() - prev_img_ptr->header.stamp.toSec();

    // Conver the opencv type to eigen
    Eigen::Matrix3d dR;
    cv::cv2eigen(opencv_R, dR);
    dR.transposeInPlace();

    // Compute angular velocity
    Eigen::AngleAxisd angle_axis(dR);
    Eigen::Vector3d axis = angle_axis.axis();
    double angle = angle_axis.angle();
    double dot_angle = angle / dt;

    Eigen::Matrix3d interim_R =
        Eigen::AngleAxisd(angle / 2.0, axis).toRotationMatrix();
    Eigen::Vector3d new_axis = offset_R * interim_R.transpose() * axis;
    ang_vel = dot_angle * new_axis;

    // Update previous image
    prev_img_ptr = curr_img_ptr;
    // ROS_INFO(" ");
    // std::cout << "dt:\t" << dt << std::endl;
    // std::cout << "dR:\n" << dR << std::endl;
    // std::cout << "angle:\t" << angle << std::endl;
    // std::cout << "axis:\t" << axis.transpose() << std::endl;
    return true;
  } else {
    // Set the first image
    prev_img_ptr =
        cv_bridge::toCvCopy(mptr, sensor_msgs::image_encodings::MONO8);
    imshow("Tracking Results", prev_img_ptr->image);
    cv::waitKey(0);
    return false;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sync_text");
  ros::NodeHandle nh;

  ROS_INFO("Loading ROS parameters.");
  if (!loadParameters()) {
    ROS_ERROR("Cannot load parameters.");
    return -1;
  }

  cv::namedWindow("Tracking Results");
  cv::waitKey(10);

  // Topics we need
  std::vector<std::string> topics(0);
  topics.push_back(imu_topic);
  topics.push_back(img_topic);

  // Store the output data
  std::vector<sensor_msgs::Imu> imu_imu(0);
  std::vector<sensor_msgs::Imu> cam_imu(0);

  // Open the bagfile to be read
  ROS_INFO("Loading bag file.");
  rosbag::Bag read_bag(read_bagname, rosbag::bagmode::Read);
  rosbag::View view(read_bag, rosbag::TopicQuery(topics));

  // Process the recorded msgs
  ROS_INFO("Process data.");
  BOOST_FOREACH (rosbag::MessageInstance const m, view) {
    if (!m.getTopic().compare(imu_topic)) {
      sensor_msgs::Imu::ConstPtr mptr = m.instantiate<sensor_msgs::Imu>();

      if (mptr != NULL) {
        imu_imu.push_back(*mptr);
      }

    } else if (!m.getTopic().compare(img_topic)) {
      sensor_msgs::Image::ConstPtr mptr = m.instantiate<sensor_msgs::Image>();

      // TODO: process the images
      if (mptr != NULL) {
        Eigen::Vector3d ang_vel;
        bool ready = computeAngularVelocity(mptr, ang_vel);
        if (ready) {
          double dt =
              (prev_img_ptr->header.stamp - curr_img_ptr->header.stamp).toSec();
          sensor_msgs::Imu new_imu;
          new_imu.header.stamp =
              prev_img_ptr->header.stamp + ros::Duration(dt / 2.0);
          tf::vectorEigenToMsg(ang_vel, new_imu.angular_velocity);
          cam_imu.push_back(new_imu);
        }
      }
    }
  }

  // Close the bag for reading
  read_bag.close();

  // Write the results to a new bagfile
  ROS_INFO("Write results into a new bagfile.");
  rosbag::Bag write_bag(write_bagname, rosbag::bagmode::Write);
  for (int i = 0; i < imu_imu.size(); ++i) {
    write_bag.write(imu_angular_vel_topic, imu_imu[i].header.stamp, imu_imu[i]);
  }
  for (int i = 0; i < cam_imu.size(); ++i) {
    write_bag.write(cam_angular_vel_topic, cam_imu[i].header.stamp, cam_imu[i]);
  }

  write_bag.close();

  return 0;
}
