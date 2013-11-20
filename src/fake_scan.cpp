#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"

struct PoleTilt {
	double angle;
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
	const static double max_pole_tilt = 5/360*2*M_PI;	//[rad]
	const static double max_robot_tilt = 5/360*2*M_PI;	//[rad]
	const static double sensor_height = 0.5;	//[m]
	const static double max_dist_error = 0.10;	//[m]

	ros::NodeHandle n;
	ros::Publisher pub_scan;
	ros::Publisher pub_ref_pose;
	ros::Subscriber sub_vel;

	void Callback(const geometry_msgs::Twist &vel) {	//update velocity
		v = vel.linear.x;
		w = vel.angular.z;
	}

	void GenerateScan() {
		ros::Rate loop_rate(25);
		srand (time(NULL));
		ros::Time begin = ros::Time::now();

		//double angle_increment_deg = 0.25;
		const double angle_increment_rad = 0.25/360*2*M_PI;
		const double angle_min = -3.0/4*M_PI;
		const double angle_max = 3.0/4*M_PI;
		std::vector<PoleTilt> pole_tilt;

		for (int i = 0; i < 4; i++) {	//fill pole tilt vector
			PoleTilt temp_tilt;
			temp_tilt.angle = ((rand() % 100) / 100.0 *max_pole_tilt);	//angle between [0, max_pole_tilt]
			//ROS_INFO("pole tilt: %f", temp_tilt.angle);
			temp_tilt.direction = ((rand() % 100) / 100.0 *2*M_PI);	//direction in [0,2M_PI]
			//ROS_INFO("pole direction: %f", temp_tilt.direction);
			pole_tilt.push_back(temp_tilt);
		}
		double tilt_x_shift1 = pole_tilt[0].angle*sensor_height*sin(pole_tilt[0].direction);
		double tilt_y_shift1 = pole_tilt[0].angle*sensor_height*cos(pole_tilt[0].direction);
		double tilt_x_shift2 = pole_tilt[1].angle*sensor_height*cos(pole_tilt[1].direction);
		double tilt_y_shift2 = pole_tilt[1].angle*sensor_height*sin(pole_tilt[1].direction);
		double tilt_x_shift3 = pole_tilt[2].angle*sensor_height*cos(pole_tilt[2].direction);
		double tilt_y_shift3 = pole_tilt[2].angle*sensor_height*sin(pole_tilt[2].direction);
		double tilt_x_shift4 = pole_tilt[3].angle*sensor_height*cos(pole_tilt[3].direction);
		double tilt_y_shift4 = pole_tilt[3].angle*sensor_height*sin(pole_tilt[3].direction);

		while(ros::ok()) {
			ros::spinOnce();	//get velocity
			double robot_tilt = ((rand() % 100) / 100.0 *max_robot_tilt);
			if ((ros::Time::now() - begin).sec > 5) {	//wait for initiation to finish
				//make velocity step
				theta += w/2*1/25;
				x += v*cos(theta)*1/25;
				y += v*sin(theta)*1/25;
				theta += w/2*1/25;
				//constant robot tilt in beginning
				robot_tilt = max_robot_tilt;
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
			//adjust distance with robot tilt
			dist1 *= 1/cos(robot_tilt);
			dist2 *= 1/cos(robot_tilt);
			dist3 *= 1/cos(robot_tilt);
			dist4 *= 1/cos(robot_tilt);
			sensor_msgs::LaserScan scan;
			scan.header.seq = 1;
			scan.header.stamp = ros::Time::now();
			scan.header.frame_id = "laser_frame";
			scan.angle_min = angle_min;
			scan.angle_max = angle_max;
			scan.angle_increment = angle_increment_rad;

			//ROS_INFO("pushed pole0 at %f m %f rad", dist1, angle1);
			//ROS_INFO("pushed pole1 at %f m %f rad", dist2, angle2);
			//ROS_INFO("pushed pole2 at %f m %f rad", dist3, angle3);
			//ROS_INFO("pushed pole3 at %f m %f rad", dist4, angle4);

			for (int i = 0; i < (angle_max-angle_min)/angle_increment_rad; i++) {
				if(i == (int)((angle1-angle_min)/angle_increment_rad)) {
					scan.ranges.push_back(dist1 + (rand() % 200) / 100.0 *max_dist_error - max_dist_error);	//with error
					scan.intensities.push_back(2000);
					//ROS_INFO("pushed pole0.1 %f at index %u", scan.ranges.back(), i);
				}
				if(i == (int)((angle1-angle_min)/angle_increment_rad)+1) {
					scan.ranges.push_back(dist1 + (rand() % 200) / 100.0 *max_dist_error - max_dist_error);
					scan.intensities.push_back(2000);
					//ROS_INFO("pushed pole0.2 %f at index %u", scan.ranges.back(), i);
				}
				if(i == (int)((angle2-angle_min)/angle_increment_rad)) {
					scan.ranges.push_back(dist2 + (rand() % 200) / 100.0 *max_dist_error - max_dist_error);
					scan.intensities.push_back(2000);
					//ROS_INFO("pushed %f at index %u", scan.ranges.back(), i);	
				}
				if(i == (int)((angle3-angle_min)/angle_increment_rad)) {
					scan.ranges.push_back(dist3 + (rand() % 200) / 100.0 *max_dist_error - max_dist_error);
					scan.intensities.push_back(2000);
					//ROS_INFO("pushed %f at index %u", scan.ranges.back(), i);	
				}
				if(i == (int)((angle4-angle_min)/angle_increment_rad)) {
					scan.ranges.push_back(dist4 + (rand() % 200) / 100.0 *max_dist_error - max_dist_error);
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
			pub_ref_pose.publish(ref_pose);
			pub_scan.publish(scan);
			loop_rate.sleep();
		}
		
	}

 public:
 	FakeScan() {
		pub_scan = n.advertise<sensor_msgs::LaserScan>("/scan",1000);
		pub_ref_pose = n.advertise<geometry_msgs::PoseStamped>("ref_pose",1000);
		sub_vel = n.subscribe("velocity",1000, &FakeScan::Callback, this);
 		//starting pose (has to be set appropriately, please leave as is)
 		x=1.5;
 		y=2;
 		theta = 0;
 		v=0;
 		w=0;
 		GenerateScan();
 	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "fake_scan");
	FakeScan *fake_scan = new FakeScan();
}