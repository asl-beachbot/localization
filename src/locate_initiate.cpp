#include "locate.h"

void Loc::InitiatePoles() {
	ros::Time begin = ros::Time::now();
	std::vector<std::vector<localization::scan_point> > extracted_scan_points;
	ROS_INFO("Gathering data...");
	double init_duration = 5;
	while ((ros::Time::now()-begin).sec < init_duration && ros::ok()) {	//gather data for 5 seconds
		ros::Rate loop_rate(25);
		ros::spinOnce();	//get one scan
		std::vector<localization::scan_point> temp_scan_points;
		ExtractPoleScans(&temp_scan_points);	//extract relevant poles
		extracted_scan_points.push_back(temp_scan_points);	//save them
		loop_rate.sleep();
	}
	ROS_INFO("Gathered %lu/%d scans", extracted_scan_points.size(), (int)(25*init_duration));
	if (extracted_scan_points.size() < 25) {		//check if enough data was gathered
		extracted_scan_points.clear();	//discard data
		ROS_WARN("Not enough scans gathered!");
	}
	else {
		ROS_INFO("Gathered enough scans!");
		std::vector<localization::scan_point> averaged_scan_points;
		for (int i = 0; i < extracted_scan_points.size(); i++) {	//combine all vectors of different measurements to one vector
			for (int j = 0; j < extracted_scan_points[i].size(); j++) {
				averaged_scan_points.push_back(extracted_scan_points[i][j]);
			}
		}
		MinimizeScans(&averaged_scan_points);		//average over all measurements
		for (int i = 0; i < averaged_scan_points.size(); i++) {	
			ROS_INFO("pole (polar) at %f m %f rad", averaged_scan_points[i].distance, averaged_scan_points[i].angle);
			//if (i == 0) ROS_INFO("error %f m",averaged_scan_points[i].distance-6.666667);		//check distance errors
			//if (i == 1) ROS_INFO("error %f m",averaged_scan_points[i].distance-8.91667);
		}
		if (averaged_scan_points.size() > 1) {
			std::vector<localization::xy_point> xy_poles = ScanToXY(averaged_scan_points);
			for (int i = 0; i < xy_poles.size(); i++) ROS_INFO("pole (kart.) at [%f %f]", xy_poles[i].x, xy_poles[i].y);	//print poles for debugging
			for (int i = 0; i < averaged_scan_points.size(); i++) {	//fill pole vector
				poles_.push_back(Pole(xy_poles[i], averaged_scan_points[i], ros::Time::now(), i));
			}
			PublishPoles();
			initiation_ = false;
		}
		else ROS_WARN("Only found %lu poles. At least 2 needed.", averaged_scan_points.size());
	}
}

std::vector<localization::xy_point> Loc::ScanToXY(const std::vector<localization::scan_point> scan) {
	std::vector<localization::xy_point> xy_vector;
	for (int i = 0; i < scan.size(); i++) {
		//convert scan to xy in robot coordinate system
		localization::xy_point point;
		point.x = scan[i].distance*cos(scan[i].angle);
		point.y = scan[i].distance*sin(scan[i].angle);
		xy_vector.push_back(point);
	}
	double x_dif = xy_vector[0].x;
	double y_dif = xy_vector[0].y;
	double rot_ang = atan2(xy_vector[1].y-xy_vector[0].y, xy_vector[1].x-xy_vector[0].x);	//get rotational angle
	//ROS_INFO("rotational angle %f", rot_ang);
	for (int i = 0; i < xy_vector.size(); i++) {
		//rotate and move from robot to fixed coordinate system
		xy_vector[i].x = xy_vector[i].x - x_dif;	//move
		xy_vector[i].y = xy_vector[i].y - y_dif;
		//ROS_INFO("pole before rotating [%f %f]", xy_vector[i].x, xy_vector[i].y);
		double temp_x = xy_vector[i].x;
		xy_vector[i].x = cos(rot_ang)*temp_x + sin(rot_ang)*xy_vector[i].y;	//rotate
		xy_vector[i].y = -sin(rot_ang)*temp_x + cos(rot_ang)*xy_vector[i].y;
	}
	return xy_vector;
}