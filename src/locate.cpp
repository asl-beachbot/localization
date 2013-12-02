#include "locate_helper.cpp"
#include "localization/scan_point.h"

Loc::Loc() {
	ROS_INFO("Started localization node");
	using_pioneer_ = true;	//if using pioneer for testing
	sub_scan_ = n_.subscribe("/scan",1000, &Loc::ScanCallback, this);
	if(using_pioneer_) sub_odom_ = n_.subscribe("/pose",1000, &Loc::OdomCallback, this);
	else sub_odom_ = n_.subscribe("/odom",1000, &Loc::OdomCallback, this);
	ROS_INFO("Subscribed to \"scan\" topic");
	pub_pose_ = n_.advertise<geometry_msgs::PoseStamped>("bot_pose",1000);
	pub_pole_ = n_.advertise<geometry_msgs::PointStamped>("pole_pos",1000);
	initiation_ = true;	//start with initiation
	pose_.pose.pose.position.x = -2000;	//for recognition if first time calculating
	odom_.pose.pose.position.x = -2000;	//for recognition if no odometry data
	StateHandler();
}

void Loc::StateHandler() {	//runs either initiation or localization
	while (ros::ok()) {
		if (initiation_) {
			ROS_INFO("started initiation");
			while (initiation_ && ros::ok()) InitiatePoles();
		}
		ROS_INFO("started localization");
		if (!initiation_) {
			while (!initiation_ && ros::ok()) Locate();
		}
	}
}

void Loc::Locate() {
	ros::Rate loop_rate(25);
	RefreshData();
	DoTheKalman();
	EstimateInvisiblePoles();
	PrintPose();
	PublishPoles();
	PublishPose();
	loop_rate.sleep();
}

//takes a vector of pole scan data and assigns them to the respective poles
void Loc::UpdatePoles(const std::vector<localization::scan_point> &scans_to_sort) {
	for(int i = 0; i < scans_to_sort.size(); i++) {	//find closest pole for every scan
		double min_dist = 2000000;
		int index = -1;
		for (int j = 0; j < poles_.size(); j++) {
			localization::scan_point current_scan = poles_[j].laser_coords();
			double current_dist = pow(scans_to_sort[i].distance*cos(scans_to_sort[i].angle) - current_scan.distance*cos(current_scan.angle),2)
				+pow(scans_to_sort[i].distance*sin(scans_to_sort[i].angle) - current_scan.distance*sin(current_scan.angle),2);
			if (current_dist < min_dist) {
				min_dist = current_dist;
				index = j;
			}
		}
		assert(index != -1);
		poles_[index].update(scans_to_sort[i], current_time_);
	}
	for (int i = 0; i < poles_.size(); i++) {	//hide all missing poles
		if (poles_[i].time() != current_time_) poles_[i].disappear();
	}
	PrintPoleScanData();
}



void Loc::EstimateInvisiblePoles() {
	//ROS_INFO("Estimating poles");
	for (int i = 0; i < poles_.size(); i++) {
		if (!poles_[i].visible()) {
			double dx = pose_.pose.pose.position.x - poles_[i].xy_coords().x;
			double dy = pose_.pose.pose.position.y - poles_[i].xy_coords().y;
			localization::scan_point temp_scan;
			temp_scan.angle = atan2(dy,dx)+3.1415927-tf::getYaw(pose_.pose.pose.orientation);
			NormalizeAngle(temp_scan.angle);
			temp_scan.distance = pow(pow(dx,2)+pow(dy,2),0.5);
			poles_[i].update(temp_scan);
			//ROS_INFO("Changed pole %d to %f m %f rad", i, temp_scan.distance, temp_scan.angle);
		}
	}
	//ROS_INFO("Done estimating");
}

bool Loc::IsPolePoint(const double &intensity, const double &distance) {
	double comparison_intensity = -1;
	if (distance < 0.5) return false;
	if (distance >= 0.5 && distance < 1) comparison_intensity = (1850-1050)/(1-0.326)*(distance-0.326)+1050;
	if (distance >= 1 && distance < 3.627) comparison_intensity = (1475-1850)/(3.627-1)*(distance-1)+1850;
	if (distance >= 3.627 && distance <= 8) comparison_intensity = (1275-1475)/(5.597-3.627)*(distance-3.627)+1475;
	if (distance > 8) comparison_intensity = 1900;	//mostly in because of fake_scan
	if (intensity > comparison_intensity) return true;
	else return false;
}

//extracts pole points from latest scan data based on intensity and writes them into scan_pole_points
void Loc::ExtractPoleScans(std::vector<localization::scan_point> *scan_pole_points) {	
	scan_pole_points->clear();	//clear old scan points
	for (int i = 0; i < scan_.intensities.size(); i++) {
		//TODO: some kind of clever function for intensities
		if (IsPolePoint(scan_.intensities[i], scan_.ranges[i])) {
			localization::scan_point temp;
			temp.distance = scan_.ranges[i];
			temp.angle = (scan_.angle_min+scan_.angle_increment*i);
			//ROS_INFO("Found point at %fm %frad", temp.distance, temp.angle);
			scan_pole_points->push_back(temp);
		}
	}
	MinimizeScans(scan_pole_points);
}

//takes vector of scan points and groupus all scan points that belong to a pole together
//writes back in the input vector
void Loc::MinimizeScans(std::vector<localization::scan_point> *scan) {
	std::vector<localization::scan_point> target; 
	std::vector<int> already_processed;
	//don't run if no poles visible
	if (!scan->empty()) {
		//loop over all poles
		for (int i = 0; i < scan->size(); i++) {
			//don't run if pole is already done
			if(std::find(already_processed.begin(), already_processed.end(), i) != already_processed.end());
			else {
				//loop over remaining poles
				//check if point is last in vector
				target.push_back(scan->at(i));
				int ppp = 1;	//points per pole
				if(i+1 != scan->size()) for (int j = i+1; j < scan->size(); j++) {
					//check already_processed
					if(std::find(already_processed.begin(), already_processed.end(), j) != already_processed.end());
					else {
						//check if close enough
						if (std::abs((scan->at(i).angle - scan->at(j).angle)*scan->at(i).distance) < 0.5
							&& std::abs(scan->at(i).distance - scan->at(j).distance) < 0.5) {
							ppp++;
							already_processed.push_back(j);
							target.back().angle += scan->at(j).angle;
							target.back().distance += scan->at(j).distance;
						}
					}
				}
				already_processed.push_back(i);
				//average
				//ROS_INFO("Found %d point/s for pole %d", ppp, i+1);
				target.back().distance /= ppp;
				target.back().angle /= ppp;
			}
			
		}
	}
	*scan = target;
}

void Loc::ScanCallback(const sensor_msgs::LaserScan &scan) {
	if (scan.intensities.size() > 0) {	//don't take scans from old laser
		scan_ = scan;
		scan_.header.stamp = current_time_;
	}
	//ROS_INFO("scan %d", scan.header.seq);
	//ROS_INFO("scan_ %d", scan_.header.seq);
}

void Loc::OdomCallback(const nav_msgs::Odometry &odom) {
	odom_ = odom;
	odom_.header.stamp = current_time_;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "localization");
	Loc *loc = new Loc();
	ROS_INFO("Location node shutting down!");
}