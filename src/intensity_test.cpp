#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <fstream>
#include <cassert>

class IntensityTest {
 public:
 	IntensityTest() {
 		sub_ = n_.subscribe("/scan",1000, &IntensityTest::Callback, this);
 		file_path_ = "/home/lorenz/beachbot/intensity_test/measurement.dat";
 		ROS_INFO("Writing file to: %s", file_path_.c_str());
 		out_stream_.open(file_path_.c_str());
 		out_stream_ << "distance [m]" << "\t" << "intensity [-]" << "\n";
 		out_stream_.close();
 		TestIntensities();
 		/*out_stream.open("/home/lorenz/beachbot/error_evaluation/error_0laser_0pole_0pitch.dat");
 		if (out_stream.is_open()) out_stream << "x_error" << "\t" << "y_error" << "\t" << "theta_error" << "\n";
 		else {
 			ROS_ERROR("Failed to open error output file");
 			delete this;
 		}*/
 	}

	~IntensityTest() {
		ROS_INFO("Shutting down intensity test");
	}

 private:
	struct IntensityStruct {
		double intensity;
		double distance;
	};

	ros::NodeHandle n_;
	ros::Subscriber sub_;
	std::ofstream out_stream_;
	sensor_msgs::LaserScan scan_;
	std::string file_path_;
	std::vector<IntensityStruct> intensity_vector_;
	const static bool filter_max_intensites_ = true;

	void TestIntensities() {
		ROS_INFO("Starting intensity test");
		ros::Rate loop_rate(25);
		ROS_INFO("Rate: 25Hz");
		ros::Duration wait(1);
		ROS_INFO("Waiting one second");
		wait.sleep();	//wait for laser to send data
		ROS_INFO("Entering loop");
		sensor_msgs::LaserScan dummy_scan;
		scan_ = dummy_scan;
		while (ros::ok()) {
			ros::Time temp_time = ros::Time::now();
			while((ros::Time::now()-temp_time).sec < 1) {	//cache errors for 1 second
				//ROS_INFO("Spinning...");
				//do ros::spinOnce(); while (scan_.intensities == dummy_scan.intensities);
				ros::spinOnce();
				if (scan_.intensities.empty()) {
					ROS_ERROR("Failed to read scan");
					delete this;
				}
				//else ROS_INFO("Success!");
				if (filter_max_intensites_) {//Only read maximum intensity
					IntensityStruct temp_intensity;
					double max_intensity = 0;
					int max_index = -1;
					for (int i = 0; i < scan_.intensities.size(); i++) {	//find maximum intensity
						//ROS_INFO("%d", i);
						if (scan_.intensities[i] > max_intensity) {
							max_intensity = scan_.intensities[i];
							max_index = i;
						}
					}
					temp_intensity.distance = scan_.ranges[max_index];
					temp_intensity.intensity = max_intensity;
					ROS_INFO("Found max intensity %f at %fm", temp_intensity.intensity, temp_intensity.distance);
					assert(max_index != -1);
					intensity_vector_.push_back(temp_intensity);
				}
				else {///*Read all data above threshold
					for (int i = 0; i < scan_.intensities.size(); i++) {
						IntensityStruct temp_intensity;
						temp_intensity.distance = scan_.ranges[i];
						temp_intensity.intensity = scan_.intensities[i];
						double compare_intensity = 750 + (600-750)/(9-1)*temp_intensity.distance;
						if (temp_intensity.intensity > compare_intensity) intensity_vector_.push_back(temp_intensity);
					}
				}
				//*/
				loop_rate.sleep();
			}
			WriteToFile();	//write errors to file
		}
	}

	void WriteToFile() {
		out_stream_.open(file_path_.c_str(), std::ios::out | std::ios::app);
		if (out_stream_.is_open()) {
			for (int i = 0; i < intensity_vector_.size(); i++) {
				out_stream_ << intensity_vector_[i].distance << "\t" << intensity_vector_[i].intensity << "\n";
			}
			out_stream_.close();
			ROS_INFO("Successfully wrote file");
			intensity_vector_.clear();
		}
 		else {
 			ROS_ERROR("Failed to open output file");
 		}
	}

	void Callback(const sensor_msgs::LaserScan &scan) {
		//ROS_INFO("Got new scan");
		scan_ = scan;
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "intensity_test");
	IntensityTest *int_test = new IntensityTest();
}