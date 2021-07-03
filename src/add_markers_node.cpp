#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "marker_manager.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "add_markers");
  ROS_WARN("Setting AMCL update_min_{d,a} parameters to zero");
  ros::param::set("/amcl/update_min_d", 0);
  ros::param::set("/amcl/update_min_a", 0);
  ros::NodeHandle node_handle;
  geometry_msgs::Point position;
  geometry_msgs::Quaternion orientation;
  position.x = -3.0;
  position.y = -1.2;
  orientation.w = 1.0;
  MarkerManager manager{node_handle, position, orientation};
  ros::spin();
}
