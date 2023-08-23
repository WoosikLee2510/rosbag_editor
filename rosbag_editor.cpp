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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>


using namespace std;

vector<string> parse_topics(string s);
void print_sensor(string name, vector<string> topics_in, vector<string> topics_out);

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
  if (source_bag_path == target_bag_path) {
    cout << "[ROSBAG EDIT]::Should set different source and target path!" << endl;
    std::exit(EXIT_FAILURE);
  }

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
  time_end = (time_end < 0 ? msgs.getEndTime().toSec(): time_end);
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
  string imu_topic_in, camera_topic_in, lidar_topic_in, vicon_topic_in, wheel1_topic_in, wheel2_topic_in, gps_topic_in, rtk_topic_in;
  nh->param<string>("imu_topic_in", imu_topic_in, string());
  nh->param<string>("camera_topic_in", camera_topic_in, string());
  nh->param<string>("lidar_topic_in", lidar_topic_in, string());
  nh->param<string>("vicon_topic_in", vicon_topic_in, string());
  nh->param<string>("wheel1_topic_in", wheel1_topic_in, string());
  nh->param<string>("wheel2_topic_in", wheel2_topic_in, string());
  nh->param<string>("gps_topic_in", gps_topic_in, string());
  nh->param<string>("rtk_topic_in", rtk_topic_in, string());
  vector<string> imu_topics_in = parse_topics(imu_topic_in);
  vector<string> camera_topics_in = parse_topics(camera_topic_in);
  vector<string> lidar_topics_in = parse_topics(lidar_topic_in);
  vector<string> vicon_topics_in = parse_topics(vicon_topic_in);
  vector<string> wheel1_topics_in = parse_topics(wheel1_topic_in);
  vector<string> wheel2_topics_in = parse_topics(wheel2_topic_in);
  vector<string> gps_topics_in = parse_topics(gps_topic_in);
  vector<string> rtk_topics_in = parse_topics(rtk_topic_in);
  string imu_topic_out, camera_topic_out, lidar_topic_out, vicon_topic_out, wheel1_topic_out, wheel2_topic_out, gps_topic_out, rtk_topic_out;
  nh->param<string>("imu_topic_out", imu_topic_out, string());
  nh->param<string>("camera_topic_out", camera_topic_out, string());
  nh->param<string>("lidar_topic_out", lidar_topic_out, string());
  nh->param<string>("vicon_topic_out", vicon_topic_out, string());
  nh->param<string>("wheel1_topic_out", wheel1_topic_out, string());
  nh->param<string>("wheel2_topic_out", wheel2_topic_out, string());
  nh->param<string>("gps_topic_out", gps_topic_out, string());
  nh->param<string>("rtk_topic_out", rtk_topic_out, string());
  vector<string> imu_topics_out = parse_topics(imu_topic_out);
  vector<string> camera_topics_out = parse_topics(camera_topic_out);
  vector<string> lidar_topics_out = parse_topics(lidar_topic_out);
  vector<string> vicon_topics_out = parse_topics(vicon_topic_out);
  vector<string> wheel1_topics_out = parse_topics(wheel1_topic_out);
  vector<string> wheel2_topics_out = parse_topics(wheel2_topic_out);
  vector<string> gps_topics_out = parse_topics(gps_topic_out);
  vector<string> rtk_topics_out = parse_topics(rtk_topic_out);


  // =======================
  // Print the setting readings
  // =======================
  cout << "[ROSBAG EDIT]::Saving sensors:" << endl;
  imu_add ? print_sensor("IMU", imu_topics_in, imu_topics_out) : void();
  camera_add ? print_sensor("CAMERA", camera_topics_in, camera_topics_out) : void();
  lidar_add ? print_sensor("LIDAR", lidar_topics_in, lidar_topics_out) : void();
  vicon_add ? print_sensor("VICON", vicon_topics_in, vicon_topics_out) : void();
  gps_add ? print_sensor("GPS", gps_topics_in, gps_topics_out) : void();
  rtk_add ? print_sensor("RTK", rtk_topics_in, rtk_topics_out) : void();
  wheel1_add ? print_sensor("WHEEL1 - Jointstates", wheel1_topics_in, wheel1_topics_out) : void();
  wheel2_add ? print_sensor("WHEEL2 - Odometry", wheel2_topics_in, wheel2_topics_out) : void();

  // Edit the messages in the source file and save it to the target file
  printf("[ROSBAG EDIT]::Saving bag to %s...   ", target_bag_path.c_str());
  for (const rosbag::MessageInstance &msg : msgs) {
    if (!ros::ok() || msg.getTime().toSec() > time_end)
      break;

    if(msg.getTime().toSec() < time_start)
      continue;

    printf("\b\b\b%02d%%", (int)(((msg.getTime().toSec() - time_start) * 100)/(time_end - time_start)));

    // VICON
    if (vicon_add) {
      for (int i = 0; i < vicon_topics_in.size(); i++) {
        if (vicon_topics_in.at(i) == msg.getTopic()) {
          assert(msg.instantiate<geometry_msgs::PoseStamped>() != nullptr);
          target_bag.write(vicon_topics_out.at(i), msg.getTime(), msg);
          continue;
        }
      }
    }
    // GPS
    if (gps_add) {
      for (int i = 0; i < gps_topics_in.size(); i++) {
        if (gps_topics_in.at(i) == msg.getTopic()) {
          assert(msg.instantiate<sensor_msgs::NavSatFix>() != nullptr);
          target_bag.write(gps_topics_out.at(i), msg.getTime(), msg);
          continue;
        }
      }
    }
    // RTK
    if (rtk_add) {
      for (int i = 0; i < rtk_topics_in.size(); i++) {
        if (rtk_topics_in.at(i) == msg.getTopic()) {
          assert(msg.instantiate<sensor_msgs::NavSatFix>() != nullptr);
          target_bag.write(rtk_topics_out.at(i), msg.getTime(), msg);
          continue;
        }
      }
    }
    // LiDAR
    if (lidar_add) {
      for (int i = 0; i < lidar_topics_in.size(); i++) {
        if (lidar_topics_in.at(i) == msg.getTopic()) {
          assert(msg.instantiate<sensor_msgs::PointCloud2>() != nullptr);
          target_bag.write(lidar_topics_out.at(i), msg.getTime(), msg);
          continue;
        }
      }
    }
    // IMU
    if (imu_add) {
      for (int i = 0; i < imu_topics_in.size(); i++) {
        if (imu_topics_in.at(i) == msg.getTopic()) {
          assert(msg.instantiate<sensor_msgs::Imu>() != nullptr);
          target_bag.write(imu_topics_out.at(i), msg.getTime(), msg);
          continue;
        }
      }
    }
    // Wheel - Jointstatus
    if (wheel1_add) {
      for (int i = 0; i < wheel1_topics_in.size(); i++) {
        if (wheel1_topics_in.at(i) == msg.getTopic()) {
          assert(msg.instantiate<sensor_msgs::JointState>() != nullptr);
          target_bag.write(wheel1_topics_out.at(i), msg.getTime(), msg);
          continue;
        }
      }
    }

    // Wheel - Jointstatus
    if (wheel2_add) {
      for (int i = 0; i < wheel2_topics_in.size(); i++) {
        if (wheel2_topics_in.at(i) == msg.getTopic()) {
          assert(msg.instantiate<nav_msgs::Odometry>() != nullptr);
          target_bag.write(wheel2_topics_out.at(i), msg.getTime(), msg);
          continue;
        }
      }
    }

    // camera
    if (camera_add) {
      for (int i = 0; i < camera_topics_in.size(); i++) {
        if (camera_topics_in.at(i) == msg.getTopic()) {
          sensor_msgs::CompressedImage::ConstPtr img_c_ptr = msg.instantiate<sensor_msgs::CompressedImage>();
          sensor_msgs::Image::ConstPtr img_i_ptr = msg.instantiate<sensor_msgs::Image>();
          assert(img_c_ptr != nullptr || img_i_ptr != nullptr);
          if(img_i_ptr != nullptr) { // this is decompressed image so save it as it is
            target_bag.write(camera_topics_out.at(i), msg.getTime(), msg);
          } else {
            assert(img_c_ptr != nullptr);
            std_msgs::Header header;
            header.stamp = img_c_ptr->header.stamp;
            cv::Mat img = cv::imdecode(cv::Mat(img_c_ptr->data), 0);
            auto img_msg = cv_bridge::CvImage(header, "mono8", img).toImageMsg();
            target_bag.write(camera_topics_out.at(i), msg.getTime(), img_msg);
          }
          continue;
        }
      }
    }
  }
  printf("\n");
  // Close the bag files and finish
  source_bag.close();
  target_bag.close();
  return 0;
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


void print_sensor(string name, vector<string> topics_in, vector<string> topics_out) {
  cout << "[ROSBAG EDIT]::" << name << "(";
  for (int i = 0; i < topics_in.size(); i++) {
    cout << topics_in.at(i) << " -> " << topics_out.at(i);
    if (i != topics_in.size() - 1)
      cout << ", ";
  }
  cout << ")" << endl;
}