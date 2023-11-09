#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"

double maze_marker_pos[6][3] = {0};

void maze_boundary_callback(const geometry_msgs::PoseStamped &msg) {
  ROS_INFO("Maze Boundary Pose: [%f, %f, %f]", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
}

void maze_marker0_callback(const geometry_msgs::PointStamped &msg) {
  // ROS_INFO("Maze Boundary Marker 0: [%f, %f, %f]", msg.point.x, msg.point.y, msg.point.z);
  maze_marker_pos[0][0] = msg.point.x;
  maze_marker_pos[0][1] = msg.point.y;
  maze_marker_pos[0][2] = msg.point.z;
}

void maze_marker1_callback(const geometry_msgs::PointStamped &msg) {
  // ROS_INFO("Maze Boundary Marker 1: [%f, %f, %f]", msg.point.x, msg.point.y, msg.point.z);
  maze_marker_pos[1][0] = msg.point.x;
  maze_marker_pos[1][1] = msg.point.y;
  maze_marker_pos[1][2] = msg.point.z;
}

void maze_marker2_callback(const geometry_msgs::PointStamped &msg) {
  // ROS_INFO("Maze Boundary Marker 2: [%f, %f, %f]", msg.point.x, msg.point.y, msg.point.z);
  maze_marker_pos[2][0] = msg.point.x;
  maze_marker_pos[2][1] = msg.point.y;
  maze_marker_pos[2][2] = msg.point.z;
}

void maze_marker3_callback(const geometry_msgs::PointStamped &msg) {
  // ROS_INFO("Maze Boundary Marker 3: [%f, %f, %f]", msg.point.x, msg.point.y, msg.point.z);
  maze_marker_pos[3][0] = msg.point.x;
  maze_marker_pos[3][1] = msg.point.y;
  maze_marker_pos[3][2] = msg.point.z;
}

void maze_marker4_callback(const geometry_msgs::PointStamped &msg) {
  // ROS_INFO("Maze Boundary Marker 4: [%f, %f, %f]", msg.point.x, msg.point.y, msg.point.z);
  maze_marker_pos[4][0] = msg.point.x;
  maze_marker_pos[4][1] = msg.point.y;
  maze_marker_pos[4][2] = msg.point.z;
}

void maze_marker5_callback(const geometry_msgs::PointStamped &msg) {
  // ROS_INFO("Maze Boundary Marker 5: [%f, %f, %f]", msg.point.x, msg.point.y, msg.point.z);
  maze_marker_pos[5][0] = msg.point.x;
  maze_marker_pos[5][1] = msg.point.y;
  maze_marker_pos[5][2] = msg.point.z;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "optitrack_stream_test");
  ros::NodeHandle nh;

  ros::Rate loop_rate(10);

  // ROS Subscribers
  // ros::Subscriber maze_boundary_sub = nh.subscribe("/natnet_ros/MazeBoundary/pose", 1, maze_boundary_callback);
  
  ros::Subscriber marker0_sub = nh.subscribe("/natnet_ros/MazeBoundary/marker0/pose", 1, maze_marker0_callback);
  ros::Subscriber marker1_sub = nh.subscribe("/natnet_ros/MazeBoundary/marker1/pose", 1, maze_marker1_callback);
  ros::Subscriber marker2_sub = nh.subscribe("/natnet_ros/MazeBoundary/marker2/pose", 1, maze_marker2_callback);
  ros::Subscriber marker3_sub = nh.subscribe("/natnet_ros/MazeBoundary/marker3/pose", 1, maze_marker3_callback);
  ros::Subscriber marker4_sub = nh.subscribe("/natnet_ros/MazeBoundary/marker4/pose", 1, maze_marker4_callback);
  ros::Subscriber marker5_sub = nh.subscribe("/natnet_ros/MazeBoundary/marker5/pose", 1, maze_marker5_callback);

  while (ros::ok())
  {
    ros::spinOnce();

    ROS_INFO("Maze Boundary Marker 0: [%f, %f, %f]", maze_marker_pos[0][0], maze_marker_pos[0][1], maze_marker_pos[0][2]);
    ROS_INFO("Maze Boundary Marker 1: [%f, %f, %f]", maze_marker_pos[1][0], maze_marker_pos[1][1], maze_marker_pos[1][2]);
    ROS_INFO("Maze Boundary Marker 2: [%f, %f, %f]", maze_marker_pos[2][0], maze_marker_pos[2][1], maze_marker_pos[2][2]);
    ROS_INFO("Maze Boundary Marker 3: [%f, %f, %f]", maze_marker_pos[3][0], maze_marker_pos[3][1], maze_marker_pos[3][2]);
    ROS_INFO("Maze Boundary Marker 4: [%f, %f, %f]", maze_marker_pos[4][0], maze_marker_pos[4][1], maze_marker_pos[4][2]);
    ROS_INFO("Maze Boundary Marker 5: [%f, %f, %f]", maze_marker_pos[5][0], maze_marker_pos[5][1], maze_marker_pos[5][2]);

    loop_rate.sleep();
  }

  return 0;
}