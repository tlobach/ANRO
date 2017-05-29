#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

ros::Publisher pub; 


using namespace std;
using namespace KDL;


KDL::Chain	chain;

void msgReceived(const sensor_msgs::JointState & msg)
{
	geometry_msgs::PoseStamped 	poseStamped;
	ChainFkSolverPos_recursive 	solver(chain);
	JntArray 			jntArray(chain.getNrOfJoints());
	Frame 				frame;	

	double 				x, y, z, w;


	double teta1=msg.position[0];
	double teta2=msg.position[1];
	double d3=msg.position[2];

	jntArray(0)=teta1;
	jntArray(1)=teta2;
	jntArray(2)=d3;

	solver.JntToCart(jntArray,frame);

	poseStamped.header.frame_id="base_link";
	poseStamped.header.stamp = ros::Time::now();

	poseStamped.pose.position.x=frame.p.data[0];
	poseStamped.pose.position.y=frame.p.data[1];
	poseStamped.pose.position.z=frame.p.data[2];

	frame.M.GetQuaternion(x, y, z, w);

	poseStamped.pose.orientation.x=x;
	poseStamped.pose.orientation.y=y;
	poseStamped.pose.orientation.z=z;
	poseStamped.pose.orientation.w=w; 

	pub.publish(poseStamped);
	
	
	
}

int main(int argc, char **argv)
{
	

	ros::init(argc, argv, "KDL");
	ros::NodeHandle node_h; 
	ros::Rate  rate(30);
	double a1=0.9,a2=0.6,d1=0.5; 
	double joint2_lower=-3.0,joint2_upper=3.0,joint3_lower=-0.15,joint3_upper=0.05; 
//	double 	   a1,a2,d1;
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
	
	pub = node_h.advertise<geometry_msgs::PoseStamped>("KDL",1);
	ros::Subscriber sub = node_h.subscribe("/joint_states", 1000, msgReceived);

	node_h.param<double>("arm1",a1,1);
    	node_h.param<double>("arm2",a2,1);
	node_h.param<double>("d1",d1,1);

	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(a1, 0, d1, 0)));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(a2, 0, 0, 0)));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ),KDL::Frame::DH(0, 0, 0, 0)));
	 

	ros::spin();

	return 0;
}
