#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/PoseStamped.h"
#include <fstream>
#include <iostream>

class ErrorChecker {
 public:
 	ErrorChecker() {
 		pose_sub = n.subscribe("bot_pose", 1000, &ErrorChecker::PoseCallback, this);
 		ref_sub = n.subscribe("ref_pose", 1000, &ErrorChecker::ReferencePoseCallback, this);
 		file_path_ = "/home/lorenz/beachbot/error_evaluation/error_0laser_0pole_0pitch.dat";
 		out_stream.open(file_path_.c_str());
 		out_stream << "time [s]" << "\t" << "x_error [m]" << "\t" << "y_error [m]" << "\t" << "theta_error [rad]" << "\n";
 		out_stream.close();
 		CheckErrors();
 		/*out_stream.open("/home/lorenz/beachbot/error_evaluation/error_0laser_0pole_0pitch.dat");
 		if (out_stream.is_open()) out_stream << "x_error" << "\t" << "y_error" << "\t" << "theta_error" << "\n";
 		else {
 			ROS_ERROR("Failed to open error output file");
 			delete this;
 		}*/
 	}

	~ErrorChecker() {
		out_stream.close();
	}

 private:
 	struct ErrorStruct
 	{
 		ros::Duration t;
 		double x_error;
 		double y_error;
 		double theta_error;
 	};
	ros::NodeHandle n;
	ros::Subscriber ref_sub;
	ros::Subscriber pose_sub;
	std::ofstream out_stream;
	geometry_msgs::PoseStamped bot_pose_;
	geometry_msgs::PoseStamped ref_pose_;
	std::vector<ErrorStruct> error_cache_;
	std::string file_path_;
	double last_error_;

	void CheckErrors() {
		ros::Rate loop_rate(25);
		ros::Time begin = ros::Time::now();
		while (ros::ok()) {
			ros::Time temp_time = ros::Time::now();
			while((ros::Time::now()-temp_time).sec < 1) {	//cache errors for 1 second
				ros::spinOnce();
				ErrorStruct temp_error;
				temp_error.t = ref_pose_.header.stamp - begin;
				temp_error.x_error = bot_pose_.pose.position.x - ref_pose_.pose.position.x;
				temp_error.y_error = bot_pose_.pose.position.y - ref_pose_.pose.position.y;
				temp_error.theta_error = tf::getYaw(bot_pose_.pose.orientation) - tf::getYaw(ref_pose_.pose.orientation);
				if (temp_error.theta_error < 5000000 && last_error_ != temp_error.x_error) {	//dont use if during initialization or shutdown
					error_cache_.push_back(temp_error);
					last_error_ = temp_error.x_error;
				}
				loop_rate.sleep();
			}
			WriteErrorToFile();	//write errors to file
		}
	}

	void WriteErrorToFile() {
		out_stream.open(file_path_.c_str(), std::ios::out | std::ios::app);
		if (out_stream.is_open()) {
			for (int i = 0; i < error_cache_.size(); i++) {
				out_stream 
				<< error_cache_[i].t.sec + (error_cache_[i].t.nsec*1e-09) << "\t"
				<< error_cache_[i].x_error << "\t" 
				<< error_cache_[i].y_error << "\t" 
				<< error_cache_[i].theta_error << "\n";
			}
			out_stream.close();
			ROS_INFO("Successfully wrote error file");
			error_cache_.clear();
		}
 		else {
 			ROS_ERROR("Failed to open error output file");
 		}
	}

	void PoseCallback(const geometry_msgs::PoseStamped &bot_pose) {
		bot_pose_ = bot_pose;
	}

	void ReferencePoseCallback(const geometry_msgs::PoseStamped &ref_pose) {
		ref_pose_ = ref_pose;
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "error_checker");
	ErrorChecker *err_check = new ErrorChecker();
}