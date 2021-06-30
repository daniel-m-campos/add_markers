#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker
MakeMarker(const geometry_msgs::Point &position,
           const geometry_msgs::Quaternion &orientation);

void PlaceMarkerAtPickup(const ros::Publisher &marker_pub,
                         geometry_msgs::Point &position,
                         geometry_msgs::Quaternion &orientation,
                         visualization_msgs::Marker &marker);
void HideMarker(const ros::Publisher &marker_pub,
                visualization_msgs::Marker &marker);

void PlaceMarkerAtDropOff(const ros::Publisher &marker_pub,
                          geometry_msgs::Point &position,
                          geometry_msgs::Quaternion &orientation,
                          visualization_msgs::Marker &marker);

int main(int argc, char **argv) {
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub =
      n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  while (marker_pub.getNumSubscribers() < 1) {
    if (!ros::ok()) {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  geometry_msgs::Point position;
  geometry_msgs::Quaternion orientation;
  visualization_msgs::Marker marker;

  PlaceMarkerAtPickup(marker_pub, position, orientation, marker);
  HideMarker(marker_pub, marker);
  PlaceMarkerAtDropOff(marker_pub, position, orientation, marker);
}

void PlaceMarkerAtPickup(const ros::Publisher &marker_pub,
                         geometry_msgs::Point &position,
                         geometry_msgs::Quaternion &orientation,
                         visualization_msgs::Marker &marker) {
  ROS_INFO("Placing marker at pickup zone");
  position.x = -3.0;
  position.y = -1.2;
  orientation.w = 1.0;
  marker = MakeMarker(position, orientation);
  marker_pub.publish(marker);
  ros::Duration(5).sleep();
}

void HideMarker(const ros::Publisher &marker_pub,
                visualization_msgs::Marker &marker) {
  ROS_INFO("Hiding marker");
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
  ros::Duration(5).sleep();
}

void PlaceMarkerAtDropOff(const ros::Publisher &marker_pub,
                          geometry_msgs::Point &position,
                          geometry_msgs::Quaternion &orientation,
                          visualization_msgs::Marker &marker) {
  ROS_INFO("Placing marker at drop off zone");
  position.x = -2.5;
  position.y = 3.5;
  orientation.z = 0.7;
  orientation.w = 0.7;
  marker.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(marker);
  ros::Duration(5).sleep();
}

visualization_msgs::Marker
MakeMarker(const geometry_msgs::Point &position,
           const geometry_msgs::Quaternion &orientation) {
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
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  return marker;
}
