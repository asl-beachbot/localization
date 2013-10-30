#include "ros/ros.h"
#include "std_msgs/String.h"
#include "laser_loc/scan_vector.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
#include <cmath>

#define PI 3.141592653589

/*
Node takes Laser scan data of two poles and converts it into bot pose
*/

class bot_loc {
  //class variables
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;

  void correctAngle(double& angle) {
    while(angle > PI) angle -= 2*PI;
    while(angle < -PI) angle += 2*PI;
  }

  void calcPose(double a_dist, double a_ang, double b_dist, double b_ang) {
    //start timer
    ros::Time begin = ros::Time::now();

    //pole coordinates
    double xp1 = 0;
    double yp1 = 0;
    double xp2 = 10;
    double yp2 = 0;
    
    //calculate possible points
    const double D = pow((xp2-xp1)*(xp2-xp1)+(yp2-yp1)*(yp2-yp1),0.5);
    const double delta = 1.0/4*pow((D+a_dist+b_dist)*(D+a_dist-b_dist)*(D-a_dist+b_dist)*(-D+a_dist+b_dist),0.5);
    double x1 = (xp1+xp2)/2+(xp2-xp1)*(a_dist*a_dist-b_dist*b_dist)/(2*D*D) + 2*(yp1-yp2)/(D*D)*delta;
    double x2 = (xp1+xp2)/2+(xp2-xp1)*(a_dist*a_dist-b_dist*b_dist)/(2*D*D) - 2*(yp1-yp2)/(D*D)*delta;
    double y1 = (yp1+yp2)/2+(yp2-yp1)*(a_dist*a_dist-b_dist*b_dist)/(2*D*D) - 2*(xp1-xp2)/(D*D)*delta;
    double y2 = (yp1+yp2)/2+(yp2-yp1)*(a_dist*a_dist-b_dist*b_dist)/(2*D*D) + 2*(xp1-xp2)/(D*D)*delta;
    
    //correct bot orientation for possible points
    double theta1 = PI - a_ang + atan2(y1-yp1,x1-xp1);
    double theta2 = PI - a_ang + atan2(y2-yp1,x2-xp1);
    //check which pose is the correct one
    bool first = (PI + atan2(y1-yp2,x1-xp2)-theta1-b_ang < 0.1);    //0.1 is possibly unreliable; needed for testing, should be adjusted
    bool second = (PI + atan2(y1-yp2,x1-xp2)-theta2-b_ang < 0.1);
    
    //input correct pose
    double theta, x, y;
    if (first) {theta = theta1; x = x1; y = y1;}
    else {theta = theta2; x = x2; y = y2;}

    correctAngle(theta);

    geometry_msgs::PoseStamped pose_stamp;
    pose_stamp.pose.position.x = x;
    pose_stamp.pose.position.y = y;
    pose_stamp.pose.position.z = 0;
    pose_stamp.pose.orientation.x = 0;
    pose_stamp.pose.orientation.y = 0;
    pose_stamp.pose.orientation.z = sin(theta/2);
    pose_stamp.pose.orientation.w = cos(theta/2);
    pose_stamp.header.frame_id = "laser_frame";
    pose_stamp.header.stamp = ros::Time::now();
    pose_stamp.header.seq = 1;
    pub.publish(pose_stamp); //*/
     

    //end timer
    ros::Time end = ros::Time::now(); 
    ros::Duration duration = end - begin;
    double delta_t = (double)duration.sec + ((double)duration.nsec)/1000000000;  

    //print pose and calc time
    ROS_INFO("[%f %f] %f rad t: %fs\n",x,y,theta,delta_t);
  }

  void chatterCallback(const laser_loc::scan_vector& pole_scan)
  {
    double a_distance = pole_scan.scans[0].distance;
    double b_distance = pole_scan.scans[1].distance;
    double a_angle = pole_scan.scans[0].angle;
    double b_angle = pole_scan.scans[1].angle;
    ROS_INFO("First pole:\td [%f] ang [%f]", a_distance, a_angle);
    ROS_INFO("Second pole:\td [%f] ang [%f]", b_distance, b_angle);
    calcPose(a_distance, a_angle, b_distance, b_angle);
  }

public:
  bot_loc() {
    
    pub = n.advertise<geometry_msgs::PoseStamped>("bot_pose",1000);
    sub = n.subscribe("pole_scan", 1000, &bot_loc::chatterCallback, this);
    ros::spin();
  }


};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "bot_loc");
  bot_loc *locate = new bot_loc();

  return 0;
}
