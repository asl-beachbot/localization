#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "pole.h"
#include "localization/scan_point.h"
#include <cmath>

const double kPi = 3.141592653589;

class Loc {
 public:
	Loc() {
		ROS_INFO("Started localization node");
		sub_ = n_.subscribe("scan",1000, &Loc::Callback, this);
		ROS_INFO("Subscribed to \"scan\" topic");
		pub_pose_ = n_.advertise<geometry_msgs::PoseStamped>("bot_pose",1000);
		pub_pole_ = n_.advertise<geometry_msgs::PointStamped>("pole_pos",1000);
		initiation_ = true;
		StateHandler();
	}

 private:
	ros::NodeHandle n_;
	ros::Subscriber sub_;
	ros::Publisher pub_pose_;
	ros::Publisher pub_pole_;

	sensor_msgs::LaserScan scan_;
	std::vector<Pole> poles_;
	geometry_msgs::PoseStamped pose_;
	bool initiation_;

	void InitiatePoles() {
		ros::Time begin = ros::Time::now();
		std::vector<std::vector<localization::scan_point> > extracted_scan_points;
		while ((ros::Time::now()-begin).sec < 1 && ros::ok()) {	//gather data for 5 seconds
			ros::Rate loop_rate(25);
			ros::spinOnce();	//get one scan
			std::vector<localization::scan_point> temp_scan_points;
			ExtractPoleScans(&temp_scan_points);	//extract relevant poles
			extracted_scan_points.push_back(temp_scan_points);	//save them
			loop_rate.sleep();
		}
		ROS_INFO("Gathered %lu/125 scans", extracted_scan_points.size());
		if (extracted_scan_points.size() < 25) {		//check if enough data was gathered
			extracted_scan_points.clear();	//discard data
			ROS_WARN("Gathering data failed during initiation!");
		}
		else {
			ROS_INFO("Gathering data for initiation successful");
			std::vector<localization::scan_point> averaged_scan_points;
			for (int i = 0; i < extracted_scan_points.size(); i++) {
				for (int j = 0; j < extracted_scan_points[i].size(); j++) {
					averaged_scan_points.push_back(extracted_scan_points[i][j]);
				}
			}
			MinimizeScans(&averaged_scan_points);
			for (int i = 0; i < averaged_scan_points.size(); i++) {
				ROS_INFO("found pole at %f m %f rad", averaged_scan_points[i].distance, averaged_scan_points[i].angle);
				if (i == 0) ROS_INFO("error %f",averaged_scan_points[i].distance-6.666667);
				if (i == 1) ROS_INFO("error %f",averaged_scan_points[i].distance-8.91667);
			}
			initiation_ = false;
		}
	}

	bool BelongTogether(const localization::scan_point &point1, const localization::scan_point &point2) {
		if (abs(point1.distance - point2.distance) < 1 && abs(point1.angle - point2.angle)*point1.distance < 1) return true;
		else return false;
	}

	void Locate() {

	}

	void StateHandler() {
		if (initiation_) {
			ROS_INFO("started initiation");
			while (initiation_ && ros::ok()) InitiatePoles();
		}
		if (!initiation_) {
			ROS_INFO("started localization");
			while (!initiation_ && ros::ok()) Locate();
		}
	}

	void ExtractPoleScans(std::vector<localization::scan_point> *scan_pole_points) {
		scan_pole_points->clear();	//clear old scan points
		for (int i = 0; i < scan_.intensities.size(); i++) {
			//TODO: some kind of clever function for intensities
			if (scan_.intensities[i] > 1000) {
				localization::scan_point temp;
				temp.distance = scan_.ranges[i];
				temp.angle = (scan_.angle_min+scan_.angle_increment*i);
				scan_pole_points->push_back(temp);
			}
		}
		MinimizeScans(scan_pole_points);
	}

	void MinimizeScans(std::vector<localization::scan_point> *scan) {
		std::vector<localization::scan_point> target; 
		std::vector<int> trash;
		//don't run if no poles visible
		if (!scan->empty()) {
			//loop over all poles
			for (int i = 0; i < scan->size(); i++) {
				//don't run if pole is already done
				if(std::find(trash.begin(), trash.end(), i) != trash.end());
				else {
					//loop over remaining poles
					//check if point is last in vector
					target.push_back(scan->at(i));
					int ppp = 1;	//points per pole
					if(i+1 != scan->size()) for (int j = i+1; j < scan->size(); j++) {
						//check trash
						if(std::find(trash.begin(), trash.end(), j) != trash.end());
						else {
							//check if close enough
							if (abs((scan->at(i).angle - scan->at(j).angle)*scan->at(i).distance) < 0.2 
								&& abs(scan->at(i).distance - scan->at(j).distance) < 0.2) {
								ppp++;
								trash.push_back(j);
								target.back().angle += scan->at(j).angle;
								target.back().distance += scan->at(j).distance;
							}
						}
					}
					trash.push_back(i);
					//average
					ROS_INFO("Found %d point/s for pole %d", ppp, i+1);
					target.back().distance /= ppp;
					target.back().angle /= ppp;
				}
				
			}
		}
		*scan = target;
	}

	void Callback(const sensor_msgs::LaserScan &scan) {
		scan_ = scan;
	}











};

int main(int argc, char **argv) {
	ros::init(argc, argv, "get_poles");
	Loc *loc = new Loc();
	ROS_INFO("Location node shutting down!");
}