#include "marker_manager.h"

#include <thread>

MarkerManager::MarkerManager(ros::NodeHandle& node_handle,
                             geometry_msgs::Point initial_position,
                             geometry_msgs::Quaternion initial_orientation,
                             double wait_seconds,
                             unsigned int not_moving_threshold)
    : robot_pose_sub_{node_handle.subscribe("/odom", 1,
                                            &MarkerManager::OdmCallback, this)},
      marker_pub_{node_handle.advertise<visualization_msgs::Marker>(
          "visualization_marker", 1)},
      marker_{MakeMarker(initial_position, initial_orientation)},
      last_odom_{},
      not_moving_count_{0},
      not_moving_threshold_{not_moving_threshold},
      is_picked_up_{false},
      wait_seconds_{wait_seconds} {
  DropOff();
  ROS_INFO_STREAM("MarkerManager constructed @ "
                  << this << ", on thread : " << std::this_thread::get_id());
}

const nav_msgs::Odometry& MarkerManager::GetLastMessage() { return last_odom_; }

void MarkerManager::OdmCallback(const nav_msgs::Odometry& odom) {
  ROS_INFO_STREAM_ONCE("Received first odom message @ " << odom.header.stamp);
  if (last_odom_.pose == odom.pose) {
    ++not_moving_count_;
  } else {
    not_moving_count_ = 0;
  }
  last_odom_ = odom;
  Update();
}

visualization_msgs::Marker MarkerManager::MakeMarker(
    geometry_msgs::Point position, geometry_msgs::Quaternion orientation) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = position;
  marker.pose.orientation = orientation;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  return marker;
}

bool MarkerManager::IsPickedUp() const { return is_picked_up_; }

void MarkerManager::DropOff() {
  marker_pub_.publish(marker_);
  ros::Duration(wait_seconds_).sleep();
}

void MarkerManager::Hide() {
  marker_.action = visualization_msgs::Marker::DELETE;
  marker_pub_.publish(marker_);
  ros::Duration(wait_seconds_).sleep();
}

void MarkerManager::Update() {
  if (not IsPickedUp() && InRange()) {
    ROS_INFO_STREAM("Marker has been picked up @ " << this);
    is_picked_up_ = true;
    Hide();
  } else if (IsNotMoving()) {
    ROS_INFO("Marker has been dropped off");
    marker_ = MakeMarker(last_odom_.pose.pose.position,
                         last_odom_.pose.pose.orientation);
    DropOff();
    is_picked_up_ = false;
  }
}

bool MarkerManager::InRange() const {
  auto dx = last_odom_.pose.pose.position.x - marker_.pose.position.x;
  auto dy = last_odom_.pose.pose.position.y - marker_.pose.position.y;
  return sqrt(pow(dx, 2) + pow(dy, 2)) < 0.1;
}

bool MarkerManager::IsNotMoving() const {
  return not_moving_count_ >= not_moving_threshold_;
}