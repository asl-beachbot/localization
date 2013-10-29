#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_loc/pole_scan.h"
#include "laser_loc/scan_point.h"
#include <cmath>
#include <vector>

#define PI 3.141592653589

/*//////////
takes laser scan and extracts pole scan points
//////////*/

class get_poles {
	
	struct scan_point {
		double angle;
		double range;
		double intensity;
	};

	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Publisher pub;

	void chatterCallback(const sensor_msgs::LaserScan& scan) {
		//normalize angle for 360° = 2PI
		double k_cor = 360.0/270;
		std::vector<scan_point> pole_scans;
		//extract pole scans
		for (int i = 0; i < scan.intensities.size(); i++) {
			//TODO: some kind of clever function
			if (scan.intensities[i] > -20*scan.ranges[i]+1200) {
				scan_point temp;
				temp.range = scan.ranges[i];
				temp.intensity = scan.intensities[i];
				temp.angle = (scan.angle_min+scan.angle_increment*i)*k_cor;
				pole_scans.push_back(temp);
			}
		}
		
		//average multiple points of single poles
		std::vector<scan_point> av_pole_scans;
		std::vector<int> trash;
		//don't run if no poles visible
		if (!pole_scans.empty()) {
			//loop over all poles
			for (int i = 0; i < pole_scans.size(); i++) {
				//don't run if pole is already done
				if(std::find(trash.begin(), trash.end(), i) != trash.end());
				else {
					//loop over remaining poles
					//check if point is last in vector
					av_pole_scans.push_back(pole_scans[i]);
					int ppp = 1;	//points per pole
					if(i+1 != pole_scans.size()) for (int j = i+1; j < pole_scans.size(); j++) {
						//check trash
						if(std::find(trash.begin(), trash.end(), j) != trash.end());
						else {
							//check if close enough
							if (abs((pole_scans[i].angle-pole_scans[j].angle)*pole_scans[i].range) < 0.2 
								&& abs(pole_scans[i].range-pole_scans[j].range) < 0.2) {
								ppp++;
								trash.push_back(j);
								av_pole_scans.back().angle += pole_scans[j].angle;
								av_pole_scans.back().range += pole_scans[j].range;
							}
						}
					}
					trash.push_back(i);
					//average
					av_pole_scans.back().range /= ppp;
					av_pole_scans.back().angle /= ppp;
				}
				
			}
		}
		//publish pole data
		laser_loc::pole_scan data;
		for (int i = 0; i < av_pole_scans.size(); i++) {
			laser_loc::scan_point temp;
			temp.distance = av_pole_scans[i].range;
			temp.angle = av_pole_scans[i].angle;
			data.poles.push_back(temp);
		}
		pub.publish(data);
	}


public:
	get_poles() {
		sub = n.subscribe("scan", 1000, &get_poles::chatterCallback, this);
		pub = n.advertise<laser_loc::pole_scan>("pole_scan",1000);
		ros::spin();


	}








};

int main(int argc, char **argv) {
	ros::init(argc, argv, "get_poles");
	get_poles *poles = new get_poles();
	

}