#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include "anro4/jint_control_srv.h"
#include <math.h>
#include <kdl/chain.hpp>
#include <iostream>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <csignal>
#include <cstring>


#define LOOP_RATE 10

volatile int exitFlag = 0;

using namespace std; 

ros::Publisher joint;
ros::Publisher pub;

double th1,th2,d3,t;
double ox , oy, oz; 
double dx, dy, dz;
double a1,a2,d1,joint2_lower,joint2_upper, joint3_lower,joint3_upper;
int steps; 
string type; 

uint path_no = 0;


void sigintHandler(int) {
	exitFlag = 1;
}

double calculateSpline(double x,double ox, double ta, double t){
	
	double result; 
	result  = ((-2*(x-ox))/(t*t*t))*(ta*ta*ta) + ((3*(x-ox))/(t*t))*(ta*ta) + ox; 
	return result; 
}


bool interoplateJoints(anro4::jint_control_srv::Request& request, anro4::jint_control_srv::Response& response){

	  	ROS_INFO("Params: x=%f, y=%f, z=%f, time=%f ", (double)request.th1, (double)request.th2, (double)request.d3, (double)request.time);
		ROS_INFO("interpolation type: %s", request.type.c_str());

		ros::Rate loop_rate(LOOP_RATE);

		sensor_msgs::JointState robotState; 
		robotState.name.push_back("joint1");
		robotState.name.push_back("joint2");
		robotState.name.push_back("joint3");
		robotState.position.resize(robotState.name.size());

		geometry_msgs::PoseStamped pose_stamped;
		pose_stamped.header.frame_id="base_link";

		nav_msgs::Path path;
		path.header.frame_id="base_link";

		if(request.th1>joint2_upper || request.th1<joint2_lower || request.th2>joint2_upper || request.th2<joint2_lower|| request.d3>0.15 || request.d3<-0.15){
			response.status = "Incorrect joints";
			return true;		
		}

	  	if(request.time <= 0){
		    response.status = "Incorrect time";
		    return true;
		}

		th1 = request.th1;
		th2 = request.th2; 
		d3 = request.d3;
		t = request.time; 

		steps = (int)(t*10); 

		if(request.type == "linear"){

			robotState.position[0] = ox; 
			robotState.position[1] = oy; 
			robotState.position[2] = oz; 

			for(int i=0; i<steps; i++){
				robotState.header.stamp = ros::Time::now();
				path.header.stamp = ros::Time::now(); 
		
				robotState.position[0] += (th1-ox)/(10*t);
				robotState.position[1] += (th2-oy)/(10*t);
				robotState.position[2] += (d3-oz)/(10*t);

				pose_stamped.header.stamp = ros::Time::now();
				pose_stamped.pose.position.x = a2*cos(robotState.position[0]+robotState.position[1])+a1*cos(robotState.position[0]);
  				pose_stamped.pose.position.y = a2*sin(robotState.position[0]+robotState.position[1])+a1*sin(robotState.position[0]);
				pose_stamped.pose.position.z = d1-robotState.position[2];
				path.poses.push_back(pose_stamped);

				joint.publish(robotState);
				pub.publish(path);
				loop_rate.sleep();
			}
		
			ox = robotState.position[0]; 
			oy = robotState.position[1];
			oz = robotState.position[2];

			response.status = "Sucess";

			ROS_INFO("Done");

			return true; 

		}else if(request.type == "spline"){

			
			double dt = t/steps; 
			double t_act = 0; 

			for(int i=0; i<steps; i++){ 
				robotState.header.stamp = ros::Time::now();
				path.header.stamp = ros::Time::now(); 
		
				robotState.position[0] =calculateSpline(th1, ox, t_act, t);
				robotState.position[1] =calculateSpline(th2, oy, t_act, t);
				robotState.position[2] =calculateSpline(d3, oz, t_act, t);
				t_act+=dt; 

				pose_stamped.header.stamp = ros::Time::now();
				pose_stamped.pose.position.x = 0.6*cos(robotState.position[0]+robotState.position[1])+0.9*cos(robotState.position[0]);
  				pose_stamped.pose.position.y = 0.6*sin(robotState.position[0]+robotState.position[1])+0.9*sin(robotState.position[0]);
				pose_stamped.pose.position.z = 0.5-robotState.position[2];
				path.poses.push_back(pose_stamped);


				joint.publish(robotState); 
				pub.publish(path);
				loop_rate.sleep();
			}

			response.status = "Sucess";
		
			ox = robotState.position[0]; 
			oy = robotState.position[1];
			oz = robotState.position[2];

			ROS_INFO("Done");

			return true; 
		}
		else{

			response.status = "Incorrect type";
			return true;
		}
	
}
 

int main(int argc, char **argv){
	

	ros::init(argc, argv, "jint");
	ros::NodeHandle node_h; 
	pub = node_h.advertise<nav_msgs::Path>("path",1);
	joint = node_h.advertise<sensor_msgs::JointState>("joint_states",1);  


	sensor_msgs::JointState robotState; 
	robotState.name.push_back("joint1");
	robotState.name.push_back("joint2");
	robotState.name.push_back("joint3");
	robotState.position.resize(robotState.name.size());
	robotState.header.stamp = ros::Time::now(); 
	robotState.position[0] = 0;
	robotState.position[1] = 0;
	robotState.position[2] = 0;
	joint.publish(robotState);
		
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


	ros::ServiceServer service = node_h.advertiseService("jint_control_srv", interoplateJoints);

	ros::spin();	

	return 0;
}
