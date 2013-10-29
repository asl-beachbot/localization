#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_loc/xy_vector.h"
#include "laser_loc/scan_point.h"
#include <cmath>
#include <vector>

#define PI 3.141592653589

class init_poles {
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Publisher pub;
	std::vector<sensor_msgs::LaserScan> scans;

	bool init_check() {
		if(scans.size() < 25) {
			ROS_INFO("Pole initialization failed");
			scans.clear();	//discard gathered data
			return false;
		}
		else return true;
	}

	void init() {

	}

	void chatterCallback(const sensor_msgs::LaserScan& temp) {
		scans.push_back(temp);
	}


public:
	init_poles() {
		sub = n.subscribe("scan", 1000, &init_poles::chatterCallback, this);
		pub = n.advertise<laser_loc::xy_vector>("pole_positions",1000);
		//pump callbacks for 5 seconds until at least 25 scans arrive
		do {	
			ros::Time begin = ros::Time::now();
			while((ros::Time::now()-begin).sec < 5) {
				ros::spinOnce();
			}

		} while(!init_check());
	}


};

int main(int argc, char **argv) {
	ros::init(argc, argv, "init_poles");
	init_poles* init = new init_poles();
	

}