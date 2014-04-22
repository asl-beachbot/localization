#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "tf/transform_datatypes.h"
#include <cmath>

class OutputSimulator {
 private:
 	double xp1_;
	double yp1_;
	double xp2_;
	double yp2_;
	double xp3_;
	double yp3_;
	double xp4_;
	double yp4_;
	double x_;
	double y_;
	double theta_;

	ros::Publisher pose_pub_;
	ros::Publisher pole_pub_;
	ros::NodeHandle n_;
	ros::Time begin_;

	void NormalizeAngle(double& angle) {	//keeps angle in [-M_PI, M_PI]
    while(angle > M_PI) angle -= 2*M_PI;
    while(angle < -M_PI) angle += 2*M_PI;
	}

	void PublishOutput() {
		geometry_msgs::PoseStamped pose;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "fixed_frame";
		pose.header.seq = 1;
		pose.pose.position.x = x_;
		pose.pose.position.y = y_;
		pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_);
		pose_pub_.publish(pose);

		geometry_msgs::PointStamped point;
		point.header.stamp = ros::Time::now();
		point.header.frame_id = "fixed_frame";
		point.header.seq = 1;
		point.point.x = xp1_;
		point.point.y = yp1_;
		pole_pub_.publish(point);
		point.header.stamp = ros::Time::now();
		point.header.frame_id = "fixed_frame";
		point.header.seq = 1;
		point.point.x = xp2_;
		point.point.y = yp2_;
		pole_pub_.publish(point);
		point.header.stamp = ros::Time::now();
		point.header.frame_id = "fixed_frame";
		point.header.seq = 1;
		point.point.x = xp3_;
		point.point.y = yp3_;
		pole_pub_.publish(point);
		point.header.stamp = ros::Time::now();
		point.header.frame_id = "fixed_frame";
		point.header.seq = 1;
		point.point.x = xp4_;
		point.point.y = yp4_;
		pole_pub_.publish(point);
	}

	void Move() {
		const ros::Time now = ros::Time::now();
		const double time_is_reserved = (begin_-now).toSec();
		x_ = 1*cos(time_is_reserved/1*M_PI)+2;
		y_ = 1*sin(time_is_reserved/1*M_PI)+2;
		theta_ = (-cos(time_is_reserved/1*M_PI)+1)*2*M_PI;
		NormalizeAngle(theta_);
	}

 public:
 	void Step() {
 		Move();
 		PublishOutput();
 	}

 	OutputSimulator() {
 		pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/localization/bot_pose", 1);
 		pole_pub_ = n_.advertise<geometry_msgs::PointStamped>("/localization/pole_pos", 4);
 		begin_ = ros::Time::now();
 		if (ros::param::get("xp1", xp1_));	
		if (ros::param::get("yp1", yp1_));	
		if (ros::param::get("xp2", xp2_));	
		if (ros::param::get("yp2", yp2_));	
		if (ros::param::get("xp3", xp3_));	
		if (ros::param::get("yp3", yp3_));	
		if (ros::param::get("xp4", xp4_));	
		if (ros::param::get("yp4", yp4_));
 	}

 	~OutputSimulator() {

 	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "output_simulator");
	OutputSimulator *output_simulator = new OutputSimulator();
	ros::Rate loop_rate(25);
	while (ros::ok()) {
		output_simulator->Step();
		loop_rate.sleep();
	}
	delete output_simulator;
	return 0;
}