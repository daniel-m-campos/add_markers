#ifndef MARKER_MANAGER_H_
#define MARKER_MANAGER_H_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class MarkerManager {
 public:
  MarkerManager(ros::NodeHandle& node_handle,
                geometry_msgs::Point initial_position,
                geometry_msgs::Quaternion initial_orientation,
                double wait_seconds = 5.0,
                unsigned int not_moving_threshold = 10);
  const nav_msgs::Odometry& GetLastMessage();
  bool IsPickedUp() const;

 private:
  ros::Publisher marker_pub_;
  ros::Subscriber robot_pose_sub_;
  visualization_msgs::Marker marker_;
  nav_msgs::Odometry last_odom_;
  unsigned int not_moving_count_;
  unsigned int not_moving_threshold_;
  bool is_picked_up_;
  double wait_seconds_;

 private:
  void OdmCallback(const nav_msgs::Odometry& odom);
  static visualization_msgs::Marker MakeMarker(
      geometry_msgs::Point position, geometry_msgs::Quaternion orientation);
  void Update();
  void Hide();
  void DropOff();
  bool InRange() const;
  bool IsNotMoving() const;
  static bool Equal(const nav_msgs::Odometry& left,
                    const nav_msgs::Odometry& right);
};

#endif  // MARKER_MANAGER_H_
