#ifndef MARKER_MANAGER_H_
#define MARKER_MANAGER_H_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class MarkerManager {
 public:
  MarkerManager(ros::NodeHandle& node_handle,
                geometry_msgs::Point initial_position,
                geometry_msgs::Quaternion initial_orientation,
                double wait_seconds = 5.0,
                unsigned int not_moving_threshold = 10);
  const geometry_msgs::PoseWithCovarianceStamped& GetLastMessage();
  bool IsPickedUp() const;
  bool IsDroppedOff() const;

 private:
  ros::Publisher marker_pub_;
  ros::Subscriber robot_pose_sub_;
  visualization_msgs::Marker marker_;
  geometry_msgs::PoseWithCovarianceStamped last_odom_;
  unsigned int not_moving_count_;
  unsigned int not_moving_threshold_;
  bool is_picked_up_;
  bool is_dropped_off_;
  double wait_seconds_;
  static constexpr double EPSILON = 0.25;

 private:
  void OdmCallback(const geometry_msgs::PoseWithCovarianceStamped& odom);
  static visualization_msgs::Marker MakeMarker(
      geometry_msgs::Point position, geometry_msgs::Quaternion orientation);
  void Update();
  void Hide();
  void DropOff();
  bool InRange() const;
  bool IsNotMoving() const;
  static bool Equal(const geometry_msgs::PoseWithCovarianceStamped& left,
                    const geometry_msgs::PoseWithCovarianceStamped& right);
  static double Distance(double dx, double dy);
  bool CanPickUp() const;
};

#endif  // MARKER_MANAGER_H_
