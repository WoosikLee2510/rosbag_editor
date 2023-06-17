/*
 * ROSBAG EDITOR
 * Copyright (C) 2022 Woosik Lee
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// SYSTEM
#include <Eigen/Eigen>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>

using namespace std;

int main(int argc, char **argv) {
  // init the system
  ros::init(argc, argv, "rosbag_editor");
  auto nh = make_shared<ros::NodeHandle>("~");

  // initialize the source bag file and the target bag file
  string source_bag_path, target_bag_path;
  nh->param<string>("source_bag_path", source_bag_path, source_bag_path);
  nh->param<string>("target_bag_path", target_bag_path, target_bag_path);
  cout << "[ROSBAG EDIT]::source_bag_path: " << source_bag_path << endl;
  cout << "[ROSBAG EDIT]::target_bag_path: " << target_bag_path << endl;

  rosbag::Bag source_bag, target_bag;
  source_bag.open(source_bag_path, rosbag::bagmode::Read);
  target_bag.open(target_bag_path, rosbag::bagmode::Write);
  rosbag::View msgs(source_bag);

  // Load edit settings
  double time_start, time_end;
  nh->param<double>("time_start", time_start, time_start);
  nh->param<double>("time_end", time_end, time_end);
  time_start = (time_start < 0 ? msgs.getBeginTime().toSec(): time_start);
  time_end = (time_end < 0 ? msgs.getBeginTime().toSec(): time_end);
  cout << fixed << "[ROSBAG EDIT]::time_start: " << time_start << endl;
  cout << fixed << "[ROSBAG EDIT]::time_end: " << time_end << endl;
  bool add_imu, add_camera, add_lidar, add_vicon, add_wheel1, add_wheel2, add_gps, add_rtk;
  nh->param<bool>("add_imu", add_imu, false);
  nh->param<bool>("add_camera", add_camera, false);
  nh->param<bool>("add_lidar", add_lidar, false);
  nh->param<bool>("add_vicon", add_vicon, false);
  nh->param<bool>("add_wheel1", add_wheel1, false);
  nh->param<bool>("add_wheel2", add_wheel2, false);
  nh->param<bool>("add_gps", add_gps, false);
  nh->param<bool>("add_rtk", add_rtk, false);
  cout << "[ROSBAG EDIT]::Saving: ";
  cout << (add_imu ? "IMU, " : "");
  cout << (add_camera ? "CAM, " : "");
  cout << (add_lidar ? "LIDAR, " : "");
  cout << (add_vicon ? "VICON, " : "");
  cout << (add_wheel1 ? "WHEEL(Joint), " : "");
  cout << (add_wheel2 ? "WHEEL(Control), " : "");
  cout << (add_gps ? "GPS, " : "");
  cout << (add_rtk ? "RTK, " : "");
  cout << endl;

  // Edit the messages in the source file and save it to the target file
  printf("Saving ros messages to %s...   ", target_bag_path.c_str());
  for (const rosbag::MessageInstance &msg : msgs) {
    if (!ros::ok() || msg.getTime().toSec() > time_end)
      break;

    if(msg.getTime().toSec() < time_start)
      continue;

    printf("\b\b\b%02d%%", (int)(((msg.getTime().toSec() - time_start) * 100)/(time_end - time_start)));

    // VICON
    if (add_vicon && msg.getTopic() == "/Husky/world") {
      geometry_msgs::PoseStamped::ConstPtr vicon = msg.instantiate<geometry_msgs::PoseStamped>();
      assert(vicon != nullptr);
      target_bag.write("/optitrack", msg.getTime(), msg);
    }
    // GPS
    if (add_gps && msg.getTopic() == "/gps/fix") {
      sensor_msgs::NavSatFix::ConstPtr gps = msg.instantiate<sensor_msgs::NavSatFix>();
      assert(gps != nullptr);
      target_bag.write(msg.getTopic(), msg.getTime(), msg);
    }
    // RTK
    if (add_rtk && msg.getTopic() == "/fix") {
      sensor_msgs::NavSatFix::ConstPtr rtk = msg.instantiate<sensor_msgs::NavSatFix>();
      assert(rtk != nullptr);
      target_bag.write("/rtk/fix", msg.getTime(), msg);
    }
    // LiDAR
    if (add_lidar && msg.getTopic() == "/ouster/points") {
      sensor_msgs::PointCloud2::ConstPtr ldr = msg.instantiate<sensor_msgs::PointCloud2>();
      assert(ldr != nullptr);
      target_bag.write(msg.getTopic(), msg.getTime(), msg);
    }
    // Wheel - Jointstatus
    if (add_wheel1 && msg.getTopic() == "/joint_status") {
      sensor_msgs::JointState::ConstPtr whl = msg.instantiate<sensor_msgs::JointState>();
      assert(whl != nullptr);
      target_bag.write(msg.getTopic(), msg.getTime(), msg);
    }
//    // Wheel - Control
//    if (add_wheel1 && msg.getTopic() == "/joint_status") {
//      sensor_msgs::JointState::ConstPtr whl = msg.instantiate<sensor_msgs::JointState>();
//      assert(whl != nullptr);
//      target_bag.write("/joint_status", msg.getTime(), msg);
//    }

    // Camera
    if (add_camera && (msg.getTopic() == "/t265/fisheye1/image_raw/compressed" || msg.getTopic() == "/t265/fisheye2/image_raw/compressed")) {
      sensor_msgs::CompressedImage::ConstPtr img = msg.instantiate<sensor_msgs::CompressedImage>();
      assert(img != nullptr);
      target_bag.write(msg.getTopic(), msg.getTime(), msg);
    }

    // IMU
    if (add_camera && msg.getTopic() == "/t265/imu") {
      sensor_msgs::Imu::ConstPtr imu = msg.instantiate<sensor_msgs::Imu>();
      assert(imu != nullptr);
      target_bag.write(msg.getTopic(), msg.getTime(), msg);
    }
  }
  printf("\n");
  // Close the bag files and finish
  source_bag.close();
  target_bag.close();
  return true;
}

