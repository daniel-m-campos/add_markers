#include <gtest/gtest.h>

#include "marker_manager.h"

class MarkerManagerFixture : public testing::Test {
 public:
  ros::NodeHandle node_handle;
  geometry_msgs::Quaternion orientation;
  geometry_msgs::Point position;
  ros::Subscriber marker_sub{node_handle.subscribe(
      "visualization_marker", 1, &MarkerManagerFixture::MarkerCallBack)};
  static void MarkerCallBack(const visualization_msgs::Marker& msg) {
    ROS_INFO_STREAM("Test subscribed to /visualization_marker");
  }
};

TEST_F(MarkerManagerFixture, testConstructor) {
  MarkerManager manager{node_handle, position, orientation, 0.1};
  ASSERT_FALSE(manager.IsPickedUp());
}

TEST_F(MarkerManagerFixture, testOdomCallBackTriggered) {
  MarkerManager manager{node_handle, position, orientation, 0.1};
  ros::Publisher publisher =
      node_handle.advertise<nav_msgs::Odometry>("/odom", 1);
  nav_msgs::Odometry expected;
  expected.header.stamp = ros::Time::now();
  publisher.publish(expected);
  ros::Duration(0.1).sleep();
  ros::spinOnce();
  auto actual = manager.GetLastMessage();
  ASSERT_EQ(expected.header.stamp, actual.header.stamp);
}

TEST_F(MarkerManagerFixture, testPickedUp) {
  position.x = -3.0;
  position.y = -1.2;
  MarkerManager manager{node_handle, position, orientation, 0.1};
  ros::Publisher publisher =
      node_handle.advertise<nav_msgs::Odometry>("/odom", 1);

  nav_msgs::Odometry msg;
  msg.header.stamp = ros::Time::now();
  msg.pose.pose.position = position;
  publisher.publish(msg);
  ros::Duration(0.1).sleep();
  ros::spinOnce();
  ASSERT_TRUE(manager.IsPickedUp());
}

TEST_F(MarkerManagerFixture, testDroppedOff) {
  position.x = -3.0;
  position.y = -1.2;
  const auto not_moving_threshold = 2;
  MarkerManager manager{node_handle, position, orientation, 0.1,
                        not_moving_threshold};
  ros::Publisher publisher =
      node_handle.advertise<nav_msgs::Odometry>("/odom", 1);

  nav_msgs::Odometry msg;
  msg.header.stamp = ros::Time::now();
  msg.pose.pose.position = position;
  publisher.publish(msg);
  ros::Duration(0.1).sleep();
  ros::spinOnce();
  ASSERT_TRUE(manager.IsPickedUp());

  position.x = -2.5;
  position.y = 3.5;
  for (int i = 0; i <= not_moving_threshold; ++i) {
    msg.header.stamp = ros::Time::now();
    msg.pose.pose.position = position;
    publisher.publish(msg);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  ASSERT_FALSE(manager.IsPickedUp());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_marker_manager");
  return RUN_ALL_TESTS();
}