#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_loc/xy_cords.h"
#include "laser_loc/xy_vector.h"
#include "laser_loc/scan_point.h"
#include "laser_loc/scan_vector.h"
#include <cmath>
#include <vector>

#define PI 3.141592653589

class init_poles {
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Publisher pub_xy;
	ros::Publisher pub_scan;
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

	//transforms vector of scan data to vector of xy coordinates in robot cs
	void scanToCoords(const laser_loc::scan_vector& scan, laser_loc::xy_vector& coords) {
		std::vector<laser_loc::xy_cords> coords_vec;
		for (int i = 0; i < scan.scans.size(); i++) {
			laser_loc::xy_cords point;
			point.x = scan.scans[i].distance*cos(scan.scans[i].angle);
			point.y = scan.scans[i].distance*sin(scan.scans[i].angle);
			coords_vec.push_back(point);
		}
		coords.points = coords_vec;
	}

	void chatterCallback(const sensor_msgs::LaserScan& temp) {
		scans.push_back(temp);
	}


public:
	init_poles() {
		sub = n.subscribe("scan", 1000, &init_poles::chatterCallback, this);
		pub_xy = n.advertise<laser_loc::xy_vector>("pole_positions",1000);
		//pump callbacks for 5 seconds until at least 25 scans arrive
		do {	
			ros::Time begin = ros::Time::now();
			while((ros::Time::now()-begin).sec < 5) {
				ros::spinOnce();
			}

		} while(!init_check());
		//TODO: write service to average mutliple points
		laser_loc::scan_vector av_scans;
		laser_loc::xy_vector coords_botcs;
		scanToCoords(av_scans, coords_botcs);
	}


};

int main(int argc, char **argv) {
	ros::init(argc, argv, "init_poles");
	init_poles* init = new init_poles();
	

}