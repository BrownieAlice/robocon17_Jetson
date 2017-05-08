#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "../include/write_circle_p.hpp"

namespace
{
  visualization_msgs::Marker marker;
  bool flag = false;
} // namespace

void write_circle(const double x, const double y, const double rad, ros::Publisher &marker_pub)
{
  if (false == flag)
  {
    init_publish();
    flag = true;
  }

  marker.header.stamp = ros::Time::now();
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.scale.x = rad * 2;
  marker.scale.y = rad * 2;

  marker_pub.publish(marker);
}

static void init_publish()
{
  marker.header.frame_id = "/laser_eth";
  marker.ns = "circle_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.0f;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5f;
  marker.lifetime = ros::Duration(1.0);
}
