#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_loc/xy_cords.h"
#include "laser_loc/xy_vector.h"
#include "laser_loc/scan_point.h"
#include "laser_loc/scan_vector.h"
#include <cmath>
#include <vector>
#include <cassert>

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
		for (int i = 0; i < scan.scans.size(); i++) {
			laser_loc::xy_cords point;
			point.x = scan.scans[i].distance*cos(scan.scans[i].angle);
			point.y = scan.scans[i].distance*sin(scan.scans[i].angle);
			coords.points.push_back(point);
		}
	}

	//rotates the CS so that line between pole 1 & 2 is base line
	void rotateCS(const laser_loc::xy_vector botcs, laser_loc::xy_vector polecs) {
		/*/find closest pole
		int closest_i = 0;
		double dist = 1000000;
		for (int i = 0; i < botcs.points.size(); i++) {
			double x = botcs.points[i].x;
			double y = botcs.points[i].y;
			if(x*x+y*y < dist) {
				dist = x*x+y*y;
				closest_i = i;
			}
		}*/

		//shift cs so 0,0 is at pole 1
		double x_dif = botcs.points[0].x;
		double y_dif = botcs.points[0].y;
		for (int i = 0; i < botcs.points.size(); i++) {
			laser_loc::xy_cords temp;
			temp.x = botcs.points[i].x - x_dif;
			temp.y = botcs.points[i].y - y_dif;
			polecs.points.push_back(temp);
		}
		assert(polecs.points[0].x == 0);
		assert(polecs.points[0].y == 0);

		//rotate cs so that pole 2 is x,0
		//TODO: make finding pole 2 smarter
		double rot_ang = atan2(polecs.points[1].y-polecs.points[0].y, polecs.points[1].x-polecs.points[0].x);
		for (int i = 1; i < botcs.points.size(); i++) {
			polecs.points[i].x = cos(rot_ang)*polecs.points[i].x + sin(rot_ang)*polecs.points[i].y;
			polecs.points[i].y = -sin(rot_ang)*polecs.points[i].x + cos(rot_ang)*polecs.points[i].y;
		}
		assert(polecs.points[1].y < 0.01);
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
		laser_loc::xy_vector coords_polecs;
		rotateCS(coords_botcs, coords_polecs);
	}


};

int main(int argc, char **argv) {
	ros::init(argc, argv, "init_poles");
	init_poles* init = new init_poles();
	

}