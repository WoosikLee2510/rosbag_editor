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

// ROS & SYSTEM
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <Eigen/Eigen>
// MESSAGES
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>

using namespace std;

vector<string> parse_topics(string s);
void print_sensor(string name, vector<string> topics);

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
  // =======================
  // Load edit settings
  // =======================
  // Bag start & end time
  double time_start, time_end;
  nh->param<double>("time_start", time_start, time_start);
  nh->param<double>("time_end", time_end, time_end);
  time_start = (time_start < 0 ? msgs.getBeginTime().toSec(): time_start);
  time_end = (time_end < 0 ? msgs.getBeginTime().toSec(): time_end);
  cout << fixed << "[ROSBAG EDIT]::time_start: " << time_start << endl;
  cout << fixed << "[ROSBAG EDIT]::time_end: " << time_end << endl;
  // sensor types to add
  bool imu_add, camera_add, lidar_add, vicon_add, wheel1_add, wheel2_add, gps_add, rtk_add;
  nh->param<bool>("imu_add", imu_add, false);
  nh->param<bool>("camera_add", camera_add, false);
  nh->param<bool>("lidar_add", lidar_add, false);
  nh->param<bool>("vicon_add", vicon_add, false);
  nh->param<bool>("wheel1_add", wheel1_add, false);
  nh->param<bool>("wheel2_add", wheel2_add, false);
  nh->param<bool>("gps_add", gps_add, false);
  nh->param<bool>("rtk_add", rtk_add, false);
  // sensor topics
  string imu_topic, camera_topic, lidar_topic, vicon_topic, wheel1_topic, wheel2_topic, gps_topic, rtk_topic;
  nh->param<string>("imu_topic", imu_topic, string());
  nh->param<string>("camera_topic", camera_topic, string());
  nh->param<string>("lidar_topic", lidar_topic, string());
  nh->param<string>("vicon_topic", vicon_topic, string());
  nh->param<string>("wheel1_topic", wheel1_topic, string());
  nh->param<string>("wheel2_topic", wheel2_topic, string());
  nh->param<string>("gps_topic", gps_topic, string());
  nh->param<string>("rtk_topic", rtk_topic, string());
  vector<string> imu_topics = parse_topics(imu_topic);
  vector<string> camera_topics = parse_topics(camera_topic);
  vector<string> lidar_topics = parse_topics(lidar_topic);
  vector<string> vicon_topics = parse_topics(vicon_topic);
  vector<string> wheel1_topics = parse_topics(wheel1_topic);
  vector<string> wheel2_topics = parse_topics(wheel2_topic);
  vector<string> gps_topics = parse_topics(gps_topic);
  vector<string> rtk_topics = parse_topics(rtk_topic);


  // =======================
  // Print the setting readings
  // =======================
  cout << "[ROSBAG EDIT]::Saving sensors:" << endl;
  imu_add ? print_sensor("IMU", imu_topics) : void();
  camera_add ? print_sensor("CAMERA", camera_topics) : void();
  lidar_add ? print_sensor("LIDAR", lidar_topics) : void();
  vicon_add ? print_sensor("VICON", vicon_topics) : void();
  gps_add ? print_sensor("GPS", gps_topics) : void();
  rtk_add ? print_sensor("RTK", rtk_topics) : void();
  wheel1_add ? print_sensor("WHEEL1 - Jointstates", wheel1_topics) : void();
  wheel2_add ? print_sensor("WHEEL2 - Odometry", wheel2_topics) : void();

  // Edit the messages in the source file and save it to the target file
  printf("[ROSBAG EDIT]::Saving bag to %s...   ", target_bag_path.c_str());
  for (const rosbag::MessageInstance &msg : msgs) {
    if (!ros::ok() || msg.getTime().toSec() > time_end)
      break;

    if(msg.getTime().toSec() < time_start)
      continue;

    printf("\b\b\b%02d%%", (int)(((msg.getTime().toSec() - time_start) * 100)/(time_end - time_start)));

    // VICON
    if (vicon_add && find(vicon_topics.begin(), vicon_topics.end(), msg.getTopic()) != vicon_topics.end()) {
      geometry_msgs::PoseStamped::ConstPtr vicon = msg.instantiate<geometry_msgs::PoseStamped>();
      assert(vicon != nullptr);
      target_bag.write("/optitrack", msg.getTime(), msg);
    }
    // GPS
    if (gps_add &&  find(gps_topics.begin(), gps_topics.end(), msg.getTopic()) != gps_topics.end()) {
      sensor_msgs::NavSatFix::ConstPtr gps = msg.instantiate<sensor_msgs::NavSatFix>();
      assert(gps != nullptr);
      target_bag.write(msg.getTopic(), msg.getTime(), msg);
    }
    // RTK
    if (rtk_add &&  find(rtk_topics.begin(), rtk_topics.end(), msg.getTopic()) != rtk_topics.end()) {
      sensor_msgs::NavSatFix::ConstPtr rtk = msg.instantiate<sensor_msgs::NavSatFix>();
      assert(rtk != nullptr);
      target_bag.write("/rtk/fix", msg.getTime(), msg);
    }
    // LiDAR
    if (lidar_add &&  find(lidar_topics.begin(), lidar_topics.end(), msg.getTopic()) != lidar_topics.end()) {
      sensor_msgs::PointCloud2::ConstPtr ldr = msg.instantiate<sensor_msgs::PointCloud2>();
      assert(ldr != nullptr);
      target_bag.write(msg.getTopic(), msg.getTime(), msg);
    }
    // Wheel - Jointstatus
    if (wheel1_add &&  find(wheel1_topics.begin(), wheel1_topics.end(), msg.getTopic()) != wheel1_topics.end()) {
      sensor_msgs::JointState::ConstPtr whl = msg.instantiate<sensor_msgs::JointState>();
      assert(whl != nullptr);
      target_bag.write(msg.getTopic(), msg.getTime(), msg);
    }
    // Wheel - Control
    if (wheel2_add &&  find(wheel2_topics.begin(), wheel2_topics.end(), msg.getTopic()) != wheel2_topics.end()) {
      nav_msgs::Odometry::ConstPtr whl = msg.instantiate<nav_msgs::Odometry>();
      assert(whl != nullptr);
      target_bag.write("/velocity_control", msg.getTime(), msg);
    }
    // Camera
    if (camera_add &&  find(camera_topics.begin(), camera_topics.end(), msg.getTopic()) != camera_topics.end()) {
      sensor_msgs::CompressedImage::ConstPtr img = msg.instantiate<sensor_msgs::CompressedImage>();
      assert(img != nullptr);
      target_bag.write(msg.getTopic(), msg.getTime(), msg);
    }
    // IMU
    if (camera_add &&  find(imu_topics.begin(), imu_topics.end(), msg.getTopic()) != imu_topics.end()) {
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


vector<string> parse_topics(string s) {
  vector<string> topics;
  while (true) {
    size_t pos = 0;
    std::string token;
    if ((pos = s.find(", ")) != std::string::npos) {
      token = s.substr(0, pos);
      topics.push_back(token);
      s.erase(0, pos + 2);
    } else {
      assert(!s.empty());
      topics.push_back(s);
      break;
    }
  }
  assert(!topics.empty());
  return topics;
}


void print_sensor(string name, vector<string> topics) {
  cout << "[ROSBAG EDIT]::\t" << name << "(";
  for (int i = 0; i < topics.size(); i++) {
    cout << topics.at(i);
    if (i != topics.size() - 1)
      cout << ", ";
  }
  cout << ")" << endl;
}