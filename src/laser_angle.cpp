#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

void Callback(const sensor_msgs::LaserScan &scan) {
	bool found_pole = false;
	double pole_distance = 0;
	for (int i = 0; i < scan.ranges.size(); i++) {
		double comparison_intensity = -1;
		const double distance = scan.ranges[i];
		const double intensity = scan.intensities[i];
		if (distance < 0.5) comparison_intensity = 5000;
		if (distance >= 0.5 && distance < 1) comparison_intensity = (1850-1050)/(1-0.326)*(distance-0.326)+1050;
		if (distance >= 1 && distance < 3.627) comparison_intensity = (1475-1850)/(3.627-1)*(distance-1)+1850;
		if (distance >= 3.627 && distance <= 8) comparison_intensity = (1275-1475)/(5.597-3.627)*(distance-3.627)+1475;
		if (distance > 8) comparison_intensity = 1031;	//mostly in because of fake_scan
		if (intensity > comparison_intensity) {
			found_pole = true;
			pole_distance = distance;
		}
		else found_pole = false|found_pole;
	}
	if (found_pole) ROS_INFO("Found pole at %fm", pole_distance);
	else ROS_WARN("No pole!");
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "laser_angle");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/scan", 2000, Callback);
	ros::spin();
}