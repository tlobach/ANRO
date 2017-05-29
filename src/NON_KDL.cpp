#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <visualization_msgs/Marker.h>
#include <signal.h>
#include <stdio.h>
#include <iostream>
#include <math.h>

double const PI = M_PI;

double theta1,theta2,d3;


ros::Publisher publisher;

void callback(const sensor_msgs::JointStateConstPtr &msg) 
{
	theta1 = msg->position[0];
	theta2 = msg->position[1];
	d3 = msg->position[2];
}

int main(int argc, char **argv)
{
	
	ros::init(argc,argv,"NON_KDL");
	ros::NodeHandle node_h;
	publisher = node_h.advertise<geometry_msgs::PoseStamped>("NONKDL", 1);
	ros::Subscriber subscriber=node_h.subscribe<sensor_msgs::JointState>("/joint_states", 1000, boost::bind(callback,_1));
	ros::Rate rate(30);
	
	double a1=0.9,a2=0.6,d1=0.5; 
	double joint2_lower=-3.0,joint2_upper=3.0,joint3_lower=-0.15,joint3_upper=0.05; 
	
	if(!node_h.getParam("/arm1",a1))
		{
			ROS_WARN("Error.Can't get parameters");	
		}
	if(!node_h.getParam("/arm2",a2))
		{
			ROS_WARN("Error.Can't get parameters");	
		}

	if(!node_h.getParam("/d1",d1))
		{
			ROS_WARN("Error.Can't get parameters");	}
	if(!node_h.getParam("/joint2_lower",joint2_lower))
		{
			ROS_WARN("Error.Can't get parameters");	
		}
	if(!node_h.getParam("/joint2_upper",joint2_upper))
		{
			ROS_WARN("Error.Can't get parameters");	
		}
	if(!node_h.getParam("/joint3_lower",joint3_lower))
		{
			ROS_WARN("Error.Can't get parameters");
		}
	if(!node_h.getParam("/joint3_upper",joint3_upper))
		{
			ROS_WARN("Error.Can't get parameters");
}

	while(ros::ok())
	{
		ros::spinOnce(); //pobieramy info od joint state publisher

//		if(theta2 > joint2_upper || theta2 < joint2_lower)
//		{
//			ROS_ERROR("Error.Can't find position.");
//			continue;
//		}
//		
//		if(d3 > joint3_upper || d3 < joint3_lower)
//		{
//			ROS_ERROR("Error.Can't find position.");
//			continue;
//		}
		
		
		double x= a2*cos(theta1+theta2)+a1*cos(theta1);
		double y =a2*sin(theta1+theta2)+a1*sin(theta1);
		double z = d1+d3;
		
		geometry_msgs::PoseStamped pose_stamped;
		pose_stamped.header.frame_id="base_link";
		pose_stamped.header.stamp = ros::Time::now();	
		pose_stamped.pose.position.x=x;
		pose_stamped.pose.position.y=y;
		pose_stamped.pose.position.z=z;
		pose_stamped.pose.orientation.w=cos((theta1+theta2)/2);
		pose_stamped.pose.orientation.x=0;
		pose_stamped.pose.orientation.y=0;
		pose_stamped.pose.orientation.z=sin((theta1+theta2)/2);
		
		
		publisher.publish(pose_stamped);
		rate.sleep();
	}

	return 0;
}
