#include "locate_helper.cpp"
#include "localization/scan_point.h"

Loc::Loc() {
	ROS_INFO("Started localization node");
	//read config from file
	if (ros::param::get("b", b));	//wheel distance of robot
	else {
		b = 0.264;
		ROS_WARN("Didn't find config for b");
	}
	if (ros::param::get("pole_radius", pole_radius)) ;	//wheel distance of robot
	else {
		pole_radius = 0.027;
		ROS_WARN("Didn't find config for pole_radius");
	}
	if (ros::param::get("use_odometry", use_odometry_));	//wheel distance of robot
	else {
		use_odometry_ = false;
		ROS_WARN("Didn't find config for use_odometry_");
	}
	sub_scan_ = n_.subscribe("/scan",1, &Loc::ScanCallback, this);
	sub_odom_ = n_.subscribe("/odometry",1, &Loc::OdomCallback, this);
	srv_init_ = n_.advertiseService("initialize_localization", &Loc::InitService, this);
	ROS_INFO("Subscribed to \"scan\" topic");
	pub_pose_ = n_.advertise<geometry_msgs::PoseStamped>("bot_pose",1000);
	pub_pole_ = n_.advertise<geometry_msgs::PointStamped>("pole_pos",1000);
	pub_map_ = n_.advertise<localization::beach_map>("beach_map",1000,true);
	SetInit(true);	//start with initiation
	pose_.pose.pose.position.x = -2000;	//for recognition if first time calculating
	last_pose_.pose.pose.position.x = -2000;	
	odom_.pose.pose.position.x = -2000;	
	last_odom_.pose.pose.position.x = -2000;
	ros::spinOnce();	//get initial data
	StateHandler();
}

void Loc::StateHandler() {	//runs either initiation or localization
	while (ros::ok()) {
		if (initiation_) {
			if(sub_scan_.getNumPublishers() == 0) {	//wait for laser to publish data
				ros::Rate scan_rate(1);
				ROS_WARN("No publisher on topic \"/scan\". Trying again every second...");
				while(sub_scan_.getNumPublishers() == 0 && ros::ok()) {
					scan_rate.sleep();
				}
			}
			InitiatePoles();
		}
		if (!initiation_) {
			Locate();
		}
	}
}

void Loc::Locate() {
	ros::Rate loop_rate(25);
	//RefreshData();
	ros::spinOnce();
	if (!scan_.ranges.empty()) DoTheKalman();
	PublishPose();
	EstimateInvisiblePoles();
	//PrintPose();
	PublishPoles();
	PublishTf();
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
		min_dist = std::abs(scans_to_sort[i].distance - poles_[index].laser_coords().distance);
		double min_angle = std::abs(scans_to_sort[i].angle - poles_[index].laser_coords().angle);
		if (poles_[index].visible()) {
			if (min_dist < 0.2 && min_angle < 0.1) {
				poles_[index].update(scans_to_sort[i], scan_.header.stamp);
			}
		}
		else {//more tolerance if pole wasnt visible
			if (min_dist < 0.4 && min_angle < 0.2) {
				poles_[index].update(scans_to_sort[i], scan_.header.stamp);		//how close the new measurement has to be to the old one !d²!
			}
		}
	}
	for (int i = 0; i < poles_.size(); i++) {	//hide all missing poles
		if (poles_[i].time() != scan_.header.stamp) poles_[i].disappear();
	}
	//PrintPoleScanData();
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
	if (distance >= 0.5 && distance < 1) comparison_intensity = (1750-950)/(1-0.326)*(distance-0.326)+950;
	if (distance >= 1 && distance < 3.627) comparison_intensity = (1375-1750)/(3.627-1)*(distance-1)+1750;
	if (distance >= 3.627 && distance <= 8) comparison_intensity = (1175-1375)/(5.597-3.627)*(distance-3.627)+1375;
	if (distance > 8) comparison_intensity = 931;	//mostly in because of fake_scan
	if (intensity > comparison_intensity && intensity < 3000) return true; //intensity <3000 to filter blinding sunlight
	else return false;
}

//extracts pole points from latest scan data based on intensity and writes them into scan_pole_points
void Loc::ExtractPoleScans(std::vector<localization::scan_point> *scan_pole_points) {	
	scan_pole_points->clear();	//clear old scan points
	for (int i = 0; i < scan_.intensities.size(); i++) {
		if (IsPolePoint(scan_.intensities[i], scan_.ranges[i])) {
			localization::scan_point temp;
			temp.distance = scan_.ranges[i] + pole_radius;	//add radius of poles 
			temp.angle = (scan_.angle_min+scan_.angle_increment*i);
			temp.intensity = scan_.intensities[i];
			//ROS_INFO("Found point at %fm %frad", temp.distance, temp.angle);
			scan_pole_points->push_back(temp);
		}
	}
	MinimizeScans(scan_pole_points);
}

//takes vector of scan points and finds closest point of the ones belonging to one pole
//writes back in the input vector
void Loc::MinimizeScans(std::vector<localization::scan_point> *scan, const int &threshold) {
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
				double max_intensity = 1;	//find closest point in vincinity
				int ppp = 1;
				if(i+1 != scan->size()) for (int j = i+1; j < scan->size(); j++) {
					//check already_processed
					if(std::find(already_processed.begin(), already_processed.end(), j) != already_processed.end());
					else {
						//check if close enough
						if (std::abs((scan->at(i).angle - scan->at(j).angle)*scan->at(i).distance) < 0.5
							&& std::abs(scan->at(i).distance - scan->at(j).distance) < 1.5) {
							ppp++;
							if (scan->at(j).intensity > max_intensity) {
								already_processed.push_back(j);
								target.back().angle = scan->at(j).angle;
								target.back().distance = scan->at(j).distance;
								target.back().intensity = scan->at(j).intensity;
							}
						}
					}
				}
				already_processed.push_back(i);
				//average
				//ROS_INFO("Found %d point/s for pole %d", ppp, i+1);
				if (ppp < threshold) target.pop_back();	//check if more scan points than threshold were gathered
			}
			
		}
	}
	*scan = target;
}

void Loc::ScanCallback(const sensor_msgs::LaserScan &scan) {
	if (scan.intensities.size() > 0) {	//don't take scans from old laser
		scan_ = scan;
	}
	else ROS_ERROR("Receiving empty laser messages");
	SetTime();
	//ROS_INFO("scan %d", scan.header.seq);
	//ROS_INFO("scan_ %d", scan_.header.seq);
}

void Loc::OdomCallback(const nav_msgs::Odometry &odom) {
	odom_ = odom;
	if (last_odom_.pose.pose.position.x == -2000) {
		initial_odom_ = odom;
		/*ROS_INFO("Initial odom pose: [%f %f] %f", 
			initial_odom_.pose.pose.position.x, initial_odom_.pose.pose.position.x, 
			tf::getYaw(initial_odom_.pose.pose.orientation));*/
	}
}

bool Loc::InitService(localization::InitLocalization::Request &req, localization::InitLocalization::Response &res) {
	if(!initiation_ && req.init) {
		SetInit(true); 
		//ROS_ERROR("initiation for localization commented out");
		ros::Time begin = ros::Time::now();
		while(initiation_ && ros::ok() && (ros::Time::now() - begin).sec < 15) {
			pose_.pose.pose.position.x = -2000;	//for recognition if first time calculating
			odom_.pose.pose.position.x = -2000;	//for recognition if no odometry data
			last_odom_.pose.pose.position.x = -2000;
			poles_.clear();
			StateHandler();
		}
		if ((ros::Time::now() - begin).sec < 15) {
			ROS_ERROR("Initiation failed for 15 seconds");
			res.success = false;
		}
		else {
			res.success = true;
		}
	}
}

void Loc::SetTime() {
	const double current_sec = scan_.header.stamp.toSec() + scan_.time_increment * scan_.ranges.size();
	current_time_.fromSec(current_sec);	//use time of last scan measurement
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "localization");
	Loc *loc = new Loc();
	ROS_INFO("Location node shutting down!");
}
