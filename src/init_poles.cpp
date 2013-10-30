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
	ros::Publisher pub;
	std::vector<laser_loc::scan_vector> scans;

	//initiation process
	void init() {
		//pump callbacks for 5 seconds until at least 25 scans arrive
		do {	
			ros::Time begin = ros::Time::now();
			ros::Rate loop_rate(25);
			while((ros::Time::now()-begin).sec < 5) {
				ros::spinOnce();
				ROS_INFO("%lu/125", scans.size());
				loop_rate.sleep();
			}
		} while(!init_check());
		ROS_INFO("%lu/125 scans came through", scans.size());
		
		laser_loc::scan_vector av_scans;
		average(scans, av_scans);	//writes to latter vector
		
		laser_loc::xy_vector xy_botcs;
		scanToCoords(av_scans, xy_botcs);	//writes to latter vector
		
		laser_loc::xy_vector xy_polecs;
		rotateCS(xy_botcs, xy_polecs);	//writes to latter vector
		ROS_INFO("rotate success");
		for (int i = 0; i < xy_polecs.points.size(); i++) ROS_INFO("pole at [%f %f]", xy_polecs.points[i].x, xy_polecs.points[i].y);
	}

	//check if enough scan data came trough
	bool init_check() {
		if(scans.size() < 25) {
			ROS_INFO("Pole initialization failed");
			scans.clear();	//discard gathered data
			return false;
		}
		else return true;
	}

	//averages the collection of pole scan data
	void average(const std::vector<laser_loc::scan_vector> &collection, laser_loc::scan_vector average) {
		//empty vector and push # of poles elements
		average.scans.clear();
		for (int i = 0; i < collection[0].scans.size(); i++) {
			laser_loc::scan_point temp;
			temp.distance = 0;
			temp.angle = 0;
			average.scans.push_back(temp);
		}
		//sum all ranges, angles
		for (int i = 0; i < collection.size(); i++) {
			for (int j = 0; j < collection[i].scans.size(); j++) {
				average.scans[j].distance += collection[i].scans[j].distance;
				average.scans[j].angle += collection[i].scans[j].angle;
			}
		}
		//average ranges, angles
		for (int i = 0; i < average.scans.size(); i++) {
			average.scans[i].distance /= collection.size();
			average.scans[i].angle /= collection.size();
		}

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
		//////////HIER STIRBT ER IRGENDWO///////////////////
		//shift cs so 0,0 is at pole 1
		ROS_INFO("started rotation");
		double x_dif = botcs.points[0].x;
		double y_dif = botcs.points[0].y;
		for (int i = 0; i < botcs.points.size(); i++) {
			laser_loc::xy_cords temp;
			temp.x = botcs.points[i].x - x_dif;
			temp.y = botcs.points[i].y - y_dif;
			polecs.points.push_back(temp);
		}
		ROS_INFO("move success");
		assert(polecs.points[0].x == 0);
		assert(polecs.points[0].y == 0);

		ROS_INFO("assert success");
		//rotate cs so that pole 2 is x,0
		//TODO: make finding pole 2 smarter
		double rot_ang = atan2(polecs.points[1].y-polecs.points[0].y, polecs.points[1].x-polecs.points[0].x);
		for (int i = 1; i < botcs.points.size(); i++) {
			polecs.points[i].x = cos(rot_ang)*polecs.points[i].x + sin(rot_ang)*polecs.points[i].y;
			polecs.points[i].y = -sin(rot_ang)*polecs.points[i].x + cos(rot_ang)*polecs.points[i].y;
		}
		assert(polecs.points[1].y < 0.01);
	}

	void chatterCallback(const laser_loc::scan_vector temp) {
		scans.push_back(temp);
	}


public:
	init_poles() {
		sub = n.subscribe("pole_scan", 1000, &init_poles::chatterCallback, this);
		pub = n.advertise<laser_loc::xy_vector>("pole_positions",1000);
		init();
	}


};

int main(int argc, char **argv) {
	ros::init(argc, argv, "init_poles");
	init_poles* init = new init_poles();

}