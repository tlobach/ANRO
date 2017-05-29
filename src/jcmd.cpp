#include "ros/ros.h"
#include "anro4/jint_control_srv.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
  double th1, th2, d3, t;
  string type;
  ros::init(argc, argv, "jint_server");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<anro4::jint_control_srv>("jint_control_srv");
  while(ros::ok())
  {
    anro4::jint_control_srv srv;

    ROS_INFO("Enter values: [theta1 theta2 d3 time type(linear or spline)] and press enter");

    cin>>th1>>th2>>d3>>t>>type;
    srv.request.th1 = th1;
    srv.request.th2 = th2;
    srv.request.d3 = d3;
    srv.request.time = t;
    srv.request.type = type;
    if(client.call(srv)){
      ROS_INFO("Service called, %s", srv.response.status.c_str());
    }else{
      ROS_ERROR("Failed to call service.");
    }
    ros::spinOnce();
  }
  return 0;
}
