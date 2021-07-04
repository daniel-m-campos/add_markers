#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "marker_manager.h"

unsigned int GetNotMovingThreshold(const ros::NodeHandle &node_handle) {
  int i = 100;
  if (node_handle.getParam("~not_moving_threshold", i))
    ROS_INFO_STREAM("Parameter not_moving_threshold set to" << i);
  unsigned int not_moving_threshold = i;
  return not_moving_threshold;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle node_handle;
  geometry_msgs::Point position;
  geometry_msgs::Quaternion orientation;
  position.x = -3.0;
  position.y = -1.2;
  orientation.w = 1.0;
  MarkerManager
      manager{node_handle, position, orientation, 5,
              GetNotMovingThreshold(node_handle)};
  ros::spin();
}
