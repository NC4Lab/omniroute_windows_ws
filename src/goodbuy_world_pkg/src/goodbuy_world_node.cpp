#include <ros/ros.h>
#include "std_msgs/String.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "goodbuy_world_node");
  ros::NodeHandle nh;
  ROS_INFO("GOODBYE CRUEL WORLD ):");

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("test_topic", 1000);
  ros::Rate loop_rate(10);
  
  int count = 0;

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "GOODBYE CRUEL WORLD ): " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}