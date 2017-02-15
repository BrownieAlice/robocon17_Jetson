#include "ros/ros.h"
#include "std_msgs/String.h"
#include "detect_cercle/Joutput.h"
#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_pub_data");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<detect_cercle::Joutput>("Jdata", 1);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    detect_cercle::Joutput msg;

    ROS_INFO("pub test data");
    msg.MB_pole=1;
    msg.x=30;
    msg.y=-30;
    msg.stamp=ros::Time::now();
    ROS_INFO("Jdata::MB_pole:%d,x:%f,y:%f",msg.MB_pole,msg.x,msg.y);

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
