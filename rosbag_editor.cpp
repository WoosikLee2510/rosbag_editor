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

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

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

  // Load edit settings
  double time_start, time_end;
  nh->param<double>("time_start", time_start, time_start);
  nh->param<double>("time_end", time_end, time_end);
  cout << "[ROSBAG EDIT]::time_start: " << time_start << endl;
  cout << "[ROSBAG EDIT]::time_end: " << time_end << endl;
  if (time_end < 0) // make end time inf if not set
    time_end = FLT_MAX;

  // Edit the messages in the source file and save it to the target file
  rosbag::View view;
  view.addQuery(source_bag);
  for (const rosbag::MessageInstance &msg : view) {
    if (!ros::ok() || msg.getTime().toSec() > time_end)
      break;

    if(msg.getTime().toSec() < time_start)
      continue;





    target_bag.write(msg.getTopic(), msg.getTime(), msg);

  }

  return true;
}