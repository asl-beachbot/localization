#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"

#define PI 3.141592653589

void correctAngle(double& angle) {
    while(angle > PI) angle -= 2*PI;
    while(angle < -PI) angle += 2*PI;
 }

class FakeScan {
 private:
 	//start pose
	double x;		
	double y;
	double theta;
 	//pole coordinates
	const static double xp1 = 0;
	const static double yp1 = 0;
	const static double xp2 = 10;
	const static double yp2 = 0;
	const static double xp3 = 10;
	const static double yp3 = 10;
	const static double xp4 = 0;
	const static double yp4 = 10;
	//velocities
	double v;
	double w;

	ros::NodeHandle n;
	ros::Publisher pub_scan;
	ros::Publisher pub_ref_pose;
	void GenerateScan() {
		ros::Rate loop_rate(25);
		srand (time(NULL));
		ros::Time begin = ros::Time::now();

		//double angle_increment_deg = 0.25;
		double angle_increment_rad = 0.25/360*2*PI;
		double angle_min = -3.0/4*PI;
		double angle_max = 3.0/4*PI;
		double error = 0.08;	//[m]

		while(ros::ok()) {
			//make velocity step
			if ((ros::Time::now() - begin).sec > 5) {	//wait for initiation to finish
				theta += w/2*1/25;
				x += v*cos(theta)*1/25;
				y += v*sin(theta)*1/25;
				theta += w/2*1/25;
			}
			ROS_INFO("Input pose [%f %f] %f rad", x, y, theta);

			double angle1 = atan2((y-yp1),(x-xp1))+3.1415927-theta;
			double angle2 = atan2((y-yp2),(x-xp2))+3.1415927-theta;
			double angle3 = atan2((y-yp3),(x-xp3))+3.1415927-theta;
			double angle4 = atan2((y-yp4),(x-xp4))+3.1415927-theta;
			correctAngle(angle1);
			correctAngle(angle2);
			correctAngle(angle3);
			correctAngle(angle4);
			//ROS_INFO("angle1 %f", angle1);
			//ROS_INFO("angle2 %f", angle2);
			double dist1 = pow(pow(x-xp1,2)+pow(y-yp1,2),0.5);
			double dist2 = pow(pow(x-xp2,2)+pow(y-yp2,2),0.5);
			double dist3 = pow(pow(x-xp3,2)+pow(y-yp3,2),0.5);
			double dist4 = pow(pow(x-xp4,2)+pow(y-yp4,2),0.5);
			sensor_msgs::LaserScan scan;
			scan.header.seq = 1;
			scan.header.stamp = ros::Time::now();
			scan.header.frame_id = "laser_frame";
			scan.angle_min = angle_min;
			scan.angle_max = angle_max;
			scan.angle_increment = angle_increment_rad;

			for (int i = 0; i < (angle_max-angle_min)/angle_increment_rad; i++) {
				if(i == (int)((angle1-angle_min)/angle_increment_rad)) {
					//scan.ranges.push_back(dist1);
					scan.ranges.push_back(dist1 + (rand() % 100) / (100/error) - error/2);	//with error
					scan.intensities.push_back(2000);
					//ROS_INFO("pushed %f at index %u", scan.ranges.back(), i);
				}
				if(i == (int)((angle1-angle_min)/angle_increment_rad)+1) {
					//scan.ranges.push_back(dist1);
					scan.ranges.push_back(dist1 + (rand() % 100) / (100/error) - error/2);
					scan.intensities.push_back(2000);
					//ROS_INFO("pushed %f at index %u", scan.ranges.back(), i);
				}
				if(i == (int)((angle2-angle_min)/angle_increment_rad)) {
					//scan.ranges.push_back(dist2);
					scan.ranges.push_back(dist2 + (rand() % 100) / (100/error) - error/2);
					scan.intensities.push_back(2000);
					//ROS_INFO("pushed %f at index %u", scan.ranges.back(), i);	
				}
				if(i == (int)((angle3-angle_min)/angle_increment_rad)) {
					//scan.ranges.push_back(dist3);
					scan.ranges.push_back(dist3 + (rand() % 100) / (100/error) - error/2);
					scan.intensities.push_back(2000);
					//ROS_INFO("pushed %f at index %u", scan.ranges.back(), i);	
				}
				if(i == (int)((angle4-angle_min)/angle_increment_rad)) {
					//scan.ranges.push_back(dist4);
					scan.ranges.push_back(dist4 + (rand() % 100) / (100/error) - error/2);
					scan.intensities.push_back(2000);
					//ROS_INFO("pushed %f at index %u", scan.ranges.back(), i);	
				}
				else {
					scan.ranges.push_back(0.0);
					scan.intensities.push_back(1);
				}
			}
			//ROS_INFO("scan has %lu entries", scan.ranges.size());
			geometry_msgs::PoseStamped ref_pose;
			ref_pose.pose.position.x = x;
			ref_pose.pose.position.y = y;
			ref_pose.pose.position.z = 0;
			ref_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
			ref_pose.header.seq = 1;
			ref_pose.header.stamp = ros::Time::now();
			ref_pose.header.frame_id = "fixed_frame";
			pub_ref_pose.publish(ref_pose);
			pub_scan.publish(scan);
			ros::spinOnce();
			loop_rate.sleep();
		}
		
	}

 public:
 	FakeScan() {
		pub_scan = n.advertise<sensor_msgs::LaserScan>("scan",1000);
		pub_ref_pose = n.advertise<geometry_msgs::PoseStamped>("ref_pose",1000);
 		x=1.5;
 		y=5;
 		theta=PI/12;
 		v = 0.5;
		w = 0.0;
 		GenerateScan();
 	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "fake_scan");
	FakeScan *fake_scan = new FakeScan();
}