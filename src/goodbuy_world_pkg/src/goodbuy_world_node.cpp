#include <ros/ros.h>
int main(int argc, char **argv) {
  ros::init(argc, argv, "goodbuy_world_node");
  ros::NodeHandle nh;
  ROS_INFO("GOODBYE CRUEL WORLD ):");
  ros::spin();
  return 0;
}