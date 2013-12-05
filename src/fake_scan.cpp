#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include <cmath>

struct DirectionalTilt {
	double tilt;
	double direction;
};

void NormalizeAngle(double& angle) {	//keeps angle in [-M_PI, M_PI]
    while(angle > M_PI) angle -= 2*M_PI;
    while(angle < -M_PI) angle += 2*M_PI;
 }

class FakeScan {
 private:
 	//start pose
	double x;		
	double y;
	double theta;
 	//pole coordinates
	double xp1;
	double yp1;
	double xp2;
	double yp2;
	double xp3;
	double yp3;
	double xp4;
	double yp4;
	//velocities
	double v;
	double w;
	double max_pole_tilt;	//[rad]
	double max_robot_tilt;	//[rad]
	double sensor_height;	//[m]
	double max_dist_error;	//[m]
	double b;	//Wheel distance [m]
	double pole_radius;	//[m]

	bool use_testing_path;

	ros::NodeHandle n;
	ros::Publisher pub_scan;
	ros::Publisher pub_ref_pose;
	ros::Publisher pub_odom;
	ros::Subscriber sub_vel;

	void Callback(const geometry_msgs::Twist &vel) {	//update velocity
		v = vel.linear.x;
		w = vel.angular.z;
	}

	void TestingPath() {
		if (x < 3.5 && theta <= 0) {v = 1; w = 1.0/2;}
		if (x == 1.5 && theta > -M_PI/2) {v = 0; w = -M_PI/2;}
		if (x >= 3.5 && theta > 0) {v = 1; w = 0;}
		if (x >= 6.0 && theta > 0) {v = 1; w = 1.0/2;}
		if (x >= 6.0 && theta > M_PI/4) {v = 1; w = 0;}
		if (x > 10.0 || y > 5) {v = 1; w = 1.0/2;}
		if (x > 0 && abs(theta) > M_PI - 0.25) {v = 1; w = 0;}
		if (x <= 0 || y > 10) {v = 1; w = -1/2.5;}
	}

	void GenerateScan() {
		ros::Rate loop_rate(25);
		srand (time(NULL));
		ros::Time begin = ros::Time::now();

		//double angle_increment_deg = 0.25;
		const double angle_increment_rad = 0.25/360*2*M_PI;
		const double angle_min = -3.0/4*M_PI;
		const double angle_max = 3.0/4*M_PI;
		std::vector<DirectionalTilt> pole_tilt;

		for (int i = 0; i < 4; i++) {	//fill pole tilt vector
			DirectionalTilt temp_tilt;
			temp_tilt.tilt = ((rand() % 100) / 100.0 *max_pole_tilt);	//angle between [0, max_pole_tilt]
			temp_tilt.direction = ((rand() % 100) / 100.0 *2*M_PI);	//direction in [0,2M_PI]
			pole_tilt.push_back(temp_tilt);
		}
		double tilt_x_shift1 = pole_tilt[0].tilt*sensor_height*sin(pole_tilt[0].direction);
		double tilt_y_shift1 = pole_tilt[0].tilt*sensor_height*cos(pole_tilt[0].direction);
		double tilt_x_shift2 = pole_tilt[1].tilt*sensor_height*cos(pole_tilt[1].direction);
		double tilt_y_shift2 = pole_tilt[1].tilt*sensor_height*sin(pole_tilt[1].direction);
		double tilt_x_shift3 = pole_tilt[2].tilt*sensor_height*cos(pole_tilt[2].direction);
		double tilt_y_shift3 = pole_tilt[2].tilt*sensor_height*sin(pole_tilt[2].direction);
		double tilt_x_shift4 = pole_tilt[3].tilt*sensor_height*cos(pole_tilt[3].direction);
		double tilt_y_shift4 = pole_tilt[3].tilt*sensor_height*sin(pole_tilt[3].direction);

		while(ros::ok()) {
			ros::spinOnce();	//get velocity
			DirectionalTilt robot_tilt;
			robot_tilt.tilt = 0;
			robot_tilt.direction = ((rand() % 100) / 100.0 *2*M_PI);
			NormalizeAngle(theta);
			if ((ros::Time::now() - begin).sec > 5) {	//wait for initiation to finish
				if (use_testing_path) {
					TestingPath();
				}
				//make velocity step
				theta += w/2*1/25;
				x += v*cos(theta)*1/25;
				y += v*sin(theta)*1/25;
				theta += w/2*1/25;
				//constant robot tilt in beginning
				robot_tilt.tilt = ((rand() % 100) / 100.0 *max_robot_tilt);
			}
			ROS_INFO("Input pose [%f %f] %f rad", x, y, theta);
			//calculate angles for poles
			double angle1 = atan2((y-(yp1+tilt_y_shift1)),(x-(xp1+tilt_x_shift1)))+3.1415927-theta;
			double angle2 = atan2((y-(yp2+tilt_y_shift2)),(x-(xp2+tilt_x_shift2)))+3.1415927-theta;
			double angle3 = atan2((y-(yp3+tilt_y_shift3)),(x-(xp3+tilt_x_shift3)))+3.1415927-theta;
			double angle4 = atan2((y-(yp4+tilt_y_shift4)),(x-(xp4+tilt_x_shift4)))+3.1415927-theta;
			NormalizeAngle(angle1);
			NormalizeAngle(angle2);
			NormalizeAngle(angle3);
			NormalizeAngle(angle4);
			//ROS_INFO("angle1 %f", angle1);
			//ROS_INFO("angle2 %f", angle2);
			double dist1 = pow(pow(x-(xp1+tilt_x_shift1),2)+pow(y-(yp1+tilt_y_shift1),2),0.5);
			double dist2 = pow(pow(x-(xp2+tilt_x_shift2),2)+pow(y-(yp2+tilt_y_shift2),2),0.5);
			double dist3 = pow(pow(x-(xp3+tilt_x_shift3),2)+pow(y-(yp3+tilt_y_shift3),2),0.5);
			double dist4 = pow(pow(x-(xp4+tilt_x_shift4),2)+pow(y-(yp4+tilt_y_shift4),2),0.5);
			ROS_INFO("dist %f",dist1);
			//adjust distance with robot tilt
			//ROS_INFO("tilt %f", robot_tilt);
			dist1 *= 1/cos(robot_tilt.tilt*cos(0));
			ROS_INFO("dist %f",dist1);
			dist2 *= 1/cos(robot_tilt.tilt*cos(robot_tilt.direction-angle1));
			dist3 *= 1/cos(robot_tilt.tilt*cos(robot_tilt.direction-angle1));
			dist4 *= 1/cos(robot_tilt.tilt*cos(robot_tilt.direction-angle1));
			sensor_msgs::LaserScan scan;
			scan.header.seq = 1;
			scan.header.stamp = ros::Time::now();
			scan.header.frame_id = "laser_frame";
			scan.angle_min = angle_min;
			scan.angle_max = angle_max;
			scan.angle_increment = angle_increment_rad;

			/*ROS_INFO("pushed pole0 at %f m %f rad", dist1, angle1);
			ROS_INFO("pushed pole1 at %f m %f rad", dist2, angle2);
			ROS_INFO("pushed pole2 at %f m %f rad", dist3, angle3);
			ROS_INFO("pushed pole3 at %f m %f rad", dist4, angle4);*/

			for (int i = 0; i < (angle_max-angle_min)/angle_increment_rad; i++) {
				if(i == (int)((angle1-angle_min)/angle_increment_rad) && dist1*tan(robot_tilt.tilt) < 0.35) {
					scan.ranges.push_back(dist1 + (rand() % 200) / 100.0 *max_dist_error - max_dist_error - pole_radius);	//with error
					scan.intensities.push_back(2000);
					//ROS_INFO("pushed pole0.1 %f at index %u", scan.ranges.back(), i);
				}
				if(i == (int)((angle1-angle_min)/angle_increment_rad)+1 && dist1*tan(robot_tilt.tilt) < 0.35) {
					scan.ranges.push_back(dist1 + (rand() % 200) / 100.0 *max_dist_error - max_dist_error - pole_radius);
					scan.intensities.push_back(2000);
					//ROS_INFO("pushed pole0.2 %f at index %u", scan.ranges.back(), i);
				}
				if(i == (int)((angle2-angle_min)/angle_increment_rad) && dist2*tan(robot_tilt.tilt) < 0.35) {
					scan.ranges.push_back(dist2 + (rand() % 200) / 100.0 *max_dist_error - max_dist_error - pole_radius);
					scan.intensities.push_back(2000);
					//ROS_INFO("pushed %f at index %u", scan.ranges.back(), i);	
				}
				if(i == (int)((angle3-angle_min)/angle_increment_rad) && dist3*tan(robot_tilt.tilt) < 0.35) {
					scan.ranges.push_back(dist3 + (rand() % 200) / 100.0 *max_dist_error - max_dist_error - pole_radius);
					scan.intensities.push_back(2000);
					//ROS_INFO("pushed %f at index %u", scan.ranges.back(), i);	
				}
				if(i == (int)((angle4-angle_min)/angle_increment_rad) && dist4*tan(robot_tilt.tilt) < 0.35) {
					scan.ranges.push_back(dist4 + (rand() % 200) / 100.0 *max_dist_error - max_dist_error - pole_radius);
					scan.intensities.push_back(2000);
					//ROS_INFO("pushed pole3 %f at index %u", scan.ranges.back(), i);	
				}
				else {
					scan.ranges.push_back(0.0);
					scan.intensities.push_back(1);
				}
			}
			geometry_msgs::PoseStamped ref_pose;
			ref_pose.pose.position.x = x;
			ref_pose.pose.position.y = y;
			ref_pose.pose.position.z = 0;
			NormalizeAngle(theta);
			ref_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
			ref_pose.header.seq = 1;
			ref_pose.header.stamp = ros::Time::now();
			ref_pose.header.frame_id = "fixed_frame";
			nav_msgs::Odometry odom;
			odom.header.stamp = ros::Time::now();
			odom.header.frame_id = "robot_frame";
			odom.header.seq = 1;
			odom.child_frame_id = "robot_frame";
			odom.twist.twist.linear.x = v + w*b/2;	//speed of right wheel
			odom.twist.twist.linear.y = v +- w*b/2;	//speed of left wheel
			odom.pose.pose.position.x = (v + w*b/2)/25;	//distance travelled of right wheel
			odom.pose.pose.position.y = (v - w*b/2)/25;	//distance travelled of left wheel
			pub_odom.publish(odom);
			pub_ref_pose.publish(ref_pose);
			pub_scan.publish(scan);
			loop_rate.sleep();
		}
		
	}

 public:
 	FakeScan() {
		pub_scan = n.advertise<sensor_msgs::LaserScan>("/scan",1000);
		pub_ref_pose = n.advertise<geometry_msgs::PoseStamped>("ref_pose",1000);
		pub_odom = n.advertise<nav_msgs::Odometry>("/odom",1000);
		sub_vel = n.subscribe("cmd_vel",1000, &FakeScan::Callback, this);
 		//starting pose (has to be set appropriately, please leave as is)
 		x=1.5;
 		y=2;
 		theta = 0;
 		v=0;
 		w=0;
 		//read config file
 		if (ros::param::get("xp1", xp1));	
		if (ros::param::get("yp1", yp1));	
		if (ros::param::get("xp2", xp2));	
		if (ros::param::get("yp2", yp2));	
		if (ros::param::get("xp3", xp3));	
		if (ros::param::get("yp3", yp3));	
		if (ros::param::get("xp4", xp4));	
		if (ros::param::get("yp4", yp4));	
		if (ros::param::get("max_pole_tilt", max_pole_tilt)) max_pole_tilt *= 2*M_PI/360.0;
		if (ros::param::get("max_robot_tilt", max_robot_tilt)) max_robot_tilt *= 2*M_PI/360.0;
		if (ros::param::get("sensor_height", sensor_height));
		if (ros::param::get("max_dist_error", max_dist_error));
		if (ros::param::get("b", b));
		if (ros::param::get("pole_radius", pole_radius));
		if (ros::param::get("use_testing_path", use_testing_path));

 		GenerateScan();
 	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "fake_scan");
	FakeScan *fake_scan = new FakeScan();
}