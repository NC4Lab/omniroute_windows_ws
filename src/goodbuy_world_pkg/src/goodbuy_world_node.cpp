int main(int argc, char **argv) {
  ros::init(argc, argv, "goodbuy_world_node");
  ros::NodeHandle nh;
  ROS_INFO("Goodbye world ):");
  ros::spin();
  return 0;
}