#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "../include/write_line_p.h"

namespace
{
  visualization_msgs::Marker marker;
  bool flag = false;
} // namespace

void write_line(const double x1, const double y1, const double x2, const double y2, int n, ros::Publisher &marker_pub)
{
  if (false == flag)
  {
    init_publish();
    flag = true;
  }

  marker.header.stamp = ros::Time::now();

  marker.points[0].x = x1;
  marker.points[0].y = y1;
  marker.points[0].z = 0;
  marker.points[1].x = x2;
  marker.points[1].y = y2;
  marker.points[1].z = 0;
  marker.id = n;

  marker_pub.publish(marker);
}

static void init_publish()
{
  marker.header.frame_id = "/laser_usb";
  marker.ns = "line_marker";
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 0;
  marker.scale.x = 0.01;
  marker.scale.y = 0;
  marker.scale.z = 0;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5f;
  marker.lifetime = ros::Duration(1.0);

  marker.points.resize(2);
}
