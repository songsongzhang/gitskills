/*
 * Copyright (C) 2016, BeiJing ZhiXingZhe, Inc.
 *
 * Author Information:
 * songsong 
 * zhangsongsong@idriverplus.com, 
 * 
 * Node Information:
 * This node is used to receive, parse and publish gps data.
 */

#include "control.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
//#include "std_ms"
#include<sstream>
ros::Publisher angle_pub;
control* node;
void chatterCallback(const ivcontrol::pathmsg::ConstPtr &msg)
{

  //ROS_INFO("I heard: [%d]", msg->number);
 // std_msgs::Float32 targetangle;
  ivcontrol::pathmsg pathgrid;
  pathgrid.number = msg->number;
  pathgrid.x = msg->x;
  pathgrid.y = msg->y;
  pathgrid.value = msg->value;
  pathgrid.U = msg->U;
  node->beroad(pathgrid);
  ROS_INFO("path grid Num of control :   [%d]", node->road.Num);
  for(int i=0; i<node->road.Num; i++)
  	{
		ROS_INFO("%d grid Num: [%d,%d]",i,node->road.point[i].x,node->road.point[i].y);
 	}
  node->resultcontrol = node->followroad(node->road);
  // ssMatrix gridmap = node->sortgrid(node->localmap);
  //ROS_INFO("path grid Num: [%d]", pathgrid.number);
  // ROS_INFO("grid after sortgrid is num: %d",gridmap.Num);
  // ROS_INFO("path grid Num: [%d,%d]",gridmap.point[2].x,gridmap.point[2].y);
  // ROS_INFO("path grid Num: [%d,%d]",gridmap.point[3].x,gridmap.point[3].y);
  node->controlmsg.data[0] = node->resultcontrol.angle;
  node->controlmsg.data[1] = node->resultcontrol.torque;
  node->controlmsg.data[2] = node->resultcontrol.speed;
  node->controlmsg.data[3] = node->resultcontrol.state;
  angle_pub.publish(node->controlmsg);
 
}
int main(int argc, char *argv[])
{
  ros::init(argc, argv,"ivcontrol");
  ros::NodeHandle mh;
  ros::Rate loop_rate(10);
  node = new control(mh);
  ros::Subscriber sub = mh.subscribe("ivpathplanner", 1000, chatterCallback);
  angle_pub = mh.advertise<std_msgs::Int16MultiArray>("control", 1000);
  ros::spin();
  loop_rate.sleep();
  return 0;
}

