#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include "anro4/oint_control_srv.h"
#include <math.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <iostream>

#define LOOP_RATE 10

using namespace std;

double x, y, z, ox, oy, oz, t;
double rx, ry, rz, rox, roy, roz;
double arx, ary, arz; 

ros::Publisher oint_pub;
ros::Publisher path_pub;

void calculateQuat(geometry_msgs::PoseStamped & msg, double roll, double pitch, double yaw){
  
  tf::Matrix3x3 obs_mat;
  obs_mat.setEulerYPR(yaw,pitch,roll);

  tf::Quaternion q_tf;
  obs_mat.getRotation(q_tf);
  msg.pose.orientation.x = q_tf.getX();
  msg.pose.orientation.y = q_tf.getY();
  msg.pose.orientation.z = q_tf.getZ();
  msg.pose.orientation.w = q_tf.getW();
}

double calculateSpline(double x,double ox, double ta, double t){
  
  double result; 
  result  = ((-2*(x-ox))/(t*t*t))*(ta*ta*ta) + ((3*(x-ox))/(t*t))*(ta*ta) + ox; 
  return result; 
}

bool interpolatePose(anro4::oint_control_srv::Request& request, anro4::oint_control_srv::Response& response){

    ROS_INFO("Params position: x=%f, y=%f, z=%f, time=%f ", (double)request.x, (double)request.y, (double)request.z, (double)request.time);
    ROS_INFO("Params orientetion: rx=%f, ry=%f, rz=%f", (double)request.rx, (double)request.ry, (double)request.rz);
    ROS_INFO("interpolation type: %s", request.type.c_str());

    ros::Rate loop_rate(LOOP_RATE);

    int steps; 


    geometry_msgs::PoseStamped oint_pose;
    geometry_msgs::PoseStamped pose_stamped;
    oint_pose.header.frame_id="base_link";

    nav_msgs::Path path; 
    path.header.frame_id="base_link";


    if(request.x>3 || request.x<0 || request.y>3 || request.y<0 || request.z>3 || request.z<0){
      response.status = "Incorrect joints";
      return true;    
    }

      if(request.time <= 0){
        response.status = "Incorrect time";
        return true;
    }

    x = request.x;
    y = request.y; 
    z = request.z;
    rx = request.rx;
    ry = request.ry; 
    rz = request.rz;
    t = request.time; 

    steps = (int)(t*10); 

    if(request.type == "linear"){

      oint_pose.pose.position.x=ox;
      oint_pose.pose.position.y=oy;
      oint_pose.pose.position.z=oz;
      arx = rox;
      ary = roy;
      arz = roz;

      for(int i=0; i<steps; i++){

        oint_pose.header.stamp = ros::Time::now();
        path.header.stamp = ros::Time::now(); 
    
        oint_pose.pose.position.x += (x-ox)/(10.0*t);
        oint_pose.pose.position.y += (y-oy)/(10.0*t);
        oint_pose.pose.position.z += (z-oz)/(10.0*t);

        arx += (rx-rox)/(10.0*t);
        ary += (ry-roy)/(10.0*t);
        arz += (rz-roz)/(10.0*t);

        calculateQuat(oint_pose,arx, ary, arz); 


        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.pose.position.x = oint_pose.pose.position.x;
        pose_stamped.pose.position.y = oint_pose.pose.position.y;
        pose_stamped.pose.position.z = oint_pose.pose.position.z;
        path.poses.push_back(pose_stamped);

        oint_pub.publish(oint_pose);
        path_pub.publish(path);
        loop_rate.sleep();

      }
    
      ox = oint_pose.pose.position.x; 
      oy = oint_pose.pose.position.y;
      oz = oint_pose.pose.position.z;

      rox = arx; 
      roy = ary;
      roz = arz;
      


      response.status = "Sucess";

      return true; 

    }else if(request.type == "spline"){

      double dt = t/steps; 
      double t_act = 0; 

      for(int i=0; i<steps; i++){ 

        oint_pose.header.stamp = ros::Time::now();
        path.header.stamp = ros::Time::now(); 
    
        oint_pose.pose.position.x =calculateSpline(x, ox, t_act, t);
        oint_pose.pose.position.y =calculateSpline(y, oy, t_act, t);
        oint_pose.pose.position.z =calculateSpline(z, oz, t_act, t);

        arx = calculateSpline(rx, rox, t_act, t);
        ary = calculateSpline(ry, roy, t_act, t);
        arz = calculateSpline(rz, roz, t_act, t);

        calculateQuat(oint_pose,arx, ary, arz); 

        t_act+=dt; 

        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.pose.position.x = oint_pose.pose.position.x;
        pose_stamped.pose.position.y = oint_pose.pose.position.y;
        pose_stamped.pose.position.z = oint_pose.pose.position.z;
        path.poses.push_back(pose_stamped);


        oint_pub.publish(oint_pose);
        path_pub.publish(path);
        loop_rate.sleep();
      }

      response.status = "Sucess";
    
      ox = oint_pose.pose.position.x; 
      oy = oint_pose.pose.position.y;
      oz = oint_pose.pose.position.z;

      rox = arx; 
      roy = ary;
      roz = arz;

      return true; 
    }
    else{

      response.status = "Incorrect type";
      return true;
    }
  
}

int main(int argc, char **argv)
{
  x = 1; 
  y = 1; 
  z = 1;

  ros::init(argc, argv, "oint");
  ros::NodeHandle n;
  
  oint_pub=n.advertise<geometry_msgs::PoseStamped>("pose_stamped", 1);
  path_pub=n.advertise<nav_msgs::Path>("path",1);
  
  geometry_msgs::PoseStamped oint_pose;
  oint_pose.header.frame_id="base_link";
  oint_pose.header.stamp = ros::Time::now();
  oint_pose.pose.position.x = 1;
  oint_pose.pose.position.y = 1;
  oint_pose.pose.position.z = 1;

  oint_pose.pose.orientation.x = 0;
  oint_pose.pose.orientation.y = 0;
  oint_pose.pose.orientation.z = 0;
  oint_pub.publish(oint_pose);

  ros::ServiceServer service = n.advertiseService("oint_control_srv", interpolatePose);


  ros::spin();
  return 0;
}
