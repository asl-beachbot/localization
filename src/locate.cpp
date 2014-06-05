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
	if (ros::param::get("scan_covariance", scan_covariance_));	//wheel distance of robot
	else {
		scan_covariance_ = 0.02*0.02;
		ROS_WARN("Didn't find config for scan_covariance_");
	}
	if (ros::param::get("k_s", k_s_));	//wheel distance of robot
	else {
		k_s_ = 100;
		ROS_WARN("Didn't find config for k_s_");
	}
	if (ros::param::get("k_th", k_th_));	//wheel distance of robot
	else {
		k_th_ = 100;
		ROS_WARN("Didn't find config for k_th_");
	}
	if (ros::param::get("laser_offset", laser_offset_));	//wheel distance of robot
	else {
		laser_offset_ = 0.05;
		ROS_WARN("Didn't find config for laser_offset");
	}
	if (ros::param::get("laser_height", laser_height_));	//wheel distance of robot
	else {
		laser_height_ = 0.35;
		ROS_WARN("Didn't find config for laser_height");
	}
	sub_scan_ = n_.subscribe("/output",1, &Loc::ScanCallback, this);
	sub_odom_ = n_.subscribe("/io_from_board",1, &Loc::OdomCallback, this);
	sub_imu_ = n_.subscribe("/imu/data",5, &Loc::ImuCallback, this);
	srv_init_ = n_.advertiseService("initialize_localization", &Loc::InitService, this);
	ROS_INFO("Subscribed to \"scan\" topic");
	pub_pose_ = n_.advertise<geometry_msgs::PoseStamped>("bot_pose",1000);
	pub_pole_ = n_.advertise<geometry_msgs::PointStamped>("pole_pos",1000);
	pub_map_ = n_.advertise<localization::beach_map>("beach_map",1000,true);
	pub_marker_ = n_.advertise<visualization_msgs::Marker>("/lines", 10, true);
	SetInit(true);	//start with initiation
	pose_.pose.pose.position.x = -2000;	//for recognition if first time calculating
	last_pose_.pose.pose.position.x = -2000;	
	odom_.timestamp = 0;	
	last_odom_.timestamp = 0;
	attitude_.orientation.x = -2000;
	last_attitude_.orientation.x = -2000;
	ros::spinOnce();	//get initial data
	ScanToCloud();
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
	ScanToCloud();
	if (!scan_.ranges.empty()) DoTheKalman();
	PublishPose();
	EstimateInvisiblePoles();
	//PrintPose();
	PublishPoles();
	PublishTf();
	loop_rate.sleep();
}

//takes a vector of pole scan data and assigns them to the respective poles
void Loc::UpdatePoles(const std::vector<Eigen::Vector3d> &scans_to_sort) {
	ROS_INFO("pred_movement [%f %f] %frad", pred_pose_.position.x - pose_.pose.pose.position.x, 
		pred_pose_.position.y - pose_.pose.pose.position.y,
		tf::getYaw(pred_pose_.orientation) - tf::getYaw(pose_.pose.pose.orientation));
	if (last_pose_.pose.pose.position.x != -2000 && pose_.pose.pose.position.x != -2000) {
		for(int i = 0; i < scans_to_sort.size(); i++) {	//find closest pole for every scan
			double min_dist = 2000000;
			double polar_dist;
			Eigen::Vector3d correct_scan;
			int index = -1;
			//ROS_INFO("sort_scan_dist %f sort_scan_angle %f", scans_to_sort[i].distance, scans_to_sort[i].angle);
			for (int j = 0; j < poles_.size(); j++) {
				const Eigen::Vector3d current_pole = poles_[j].line().p;
				const double dx = current_pole.x() - pred_pose_.position.x;
				const double dy = current_pole.y() - pred_pose_.position.y;
				Eigen::Vector3d current_scan(dx, dy, 0);
				const double theta = tf::getYaw(pred_pose_.orientation);
				Eigen::Matrix3d rot;
				rot = Eigen::AngleAxis<double>(-theta, Eigen::Vector3d::UnitZ());
				current_scan = rot * current_scan;
				const double current_dist = (scans_to_sort[i].x() - current_scan.x()) * (scans_to_sort[i].x() - current_scan.x())
					+ (scans_to_sort[i].y() - current_scan.y()) * (scans_to_sort[i].y() - current_scan.y());
				//ROS_INFO("i %d j %d current_dist %f scan_dist %f scan_angle %f", i, j, current_dist, current_scan.distance, current_scan.angle);
				if (current_dist < min_dist) {
					min_dist = current_dist;
					correct_scan = current_scan;
					index = j;
				}
			}
			assert(index != -1);
			double min_angle = atan2(scans_to_sort[i].y(), scans_to_sort[i].x() ) - atan2(correct_scan.y(), correct_scan.x() );
			NormalizeAngle(min_angle);
			min_angle = std::abs(min_angle);
			ROS_INFO("min dist %f min angle %f", min_dist, min_angle);
			if (poles_[index].visible()) {
				if (min_dist < 0.2*0.2 && min_angle < 0.1) {
					poles_[index].update(scans_to_sort[i], cloud_.header.stamp);
				}
			}
			else {//more tolerance if pole wasn't visible
				if (min_dist < 0.4*0.4 && min_angle < 0.2) {
					poles_[index].update(scans_to_sort[i], cloud_.header.stamp);		//how close the new measurement has to be to the old one !d²!
				}
			}
		}
		for (int i = 0; i < poles_.size(); i++) {	//hide all missing poles
			if (poles_[i].time() != cloud_.header.stamp) poles_[i].disappear();
		}
		//PrintPoleScanData();
	}
}

void Loc::EstimateInvisiblePoles() {
	//ROS_INFO("Estimating poles");
	for (int i = 0; i < poles_.size(); i++) {
		if (!poles_[i].visible()) {
			Eigen::Matrix3d rot;
			rot = Eigen::AngleAxis<double>(-tf::getYaw(pose_.pose.pose.orientation), Eigen::Vector3d::UnitZ());
			poles_[i].update(rot * poles_[i].line().p);
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

//Groups cloud points belonging to one pole together and averages them
void Loc::MinimizeScans(std::vector<Eigen::Vector3d> *scan) {
	scan->clear();
	std::vector<geometry_msgs::Point32> target; 
	std::vector<int> already_processed;
	//don't run if no poles visible
	if (!cloud_.points.empty()) {
		for (int i = 0; i < cloud_.points.size(); i++) {	//loop over all poles
			//don't run if pole is already done
			if(std::find(already_processed.begin(), already_processed.end(), i) != already_processed.end());
			else {	//loop over remaining poles
				target.push_back(cloud_.points.at(i));	//check if point is last in vector
				int ppp = 1;
				if(i+1 != cloud_.points.size()) for (int j = i+1; j < cloud_.points.size(); j++) {
					//check already_processed
					if(std::find(already_processed.begin(), already_processed.end(), j) != already_processed.end());
					else {	//check if close enough
						const double dx = cloud_.points.at(i).x - cloud_.points.at(j).x;
						const double dy = cloud_.points.at(i).y - cloud_.points.at(j).y;
						if ( (dx * dx + dy * dy) < 0.5 * 0.5) {	//group if in circle of 0.5m; disregard z-value
							ppp++;
							already_processed.push_back(j);
							target.back().x += cloud_.points.at(j).x;
							target.back().y += cloud_.points.at(j).y;
							target.back().y += cloud_.points.at(j).z;
						}
					}
				}
				already_processed.push_back(i);
				target.back().x /= ppp; //average
				target.back().y /= ppp;
				target.back().z /= ppp;
				//ROS_INFO("Found %d point/s for pole %d", ppp, i);
				//ROS_INFO("Pole %d minimized [%f %f %f]", i, target.back().x, target.back().y, target.back().z);
			}
		}
	}
	for (int i = 0; i < target.size(); i++) {	//convert Point32 to Vector3d
		scan->push_back( Eigen::Vector3d( target[i].x, target[i].y, target[i].z ) );
	}
}

void Loc::CorrectMoveError(std::vector<Eigen::Vector3d> *scan_pole_points) {	//correct error due to moving laser
	if (last_pose_.pose.pose.position.x != -2000 && pose_.pose.pose.position.x != -2000) {	
		for (int i = 0; i < scan_pole_points->size(); i++) {
			Eigen::Vector3d temp_point = scan_pole_points->at(i);
			double scan_angle = atan2( temp_point.y(), temp_point.x() );
			const double scan_dist = sqrt( (temp_point.x() * temp_point.x() ) + (temp_point.y() * temp_point.y() ) );
			const int scan_index = (int)( scan_angle - scan_.angle_min ) / scan_.angle_increment;
			const double measurement_delay = ( scan_.ranges.size() - scan_index ) * scan_.time_increment;
			const double delta_t_old = ( attitude_.header.stamp - last_attitude_.header.stamp ).toSec();
			const double time_scale = measurement_delay / delta_t_old;
			const double last_theta = tf::getYaw( last_attitude_.orientation );
			const double current_theta = tf::getYaw( attitude_.orientation );
			double delta_theta = ( current_theta - last_theta );
			NormalizeAngle(delta_theta);
			delta_theta *= time_scale;
			//ROS_INFO("Corrected angle %d by %f with scale %f", i, delta_theta, time_scale);
			scan_angle -= delta_theta;
			scan_pole_points->at(i).x() = scan_dist * cos(scan_angle);
			scan_pole_points->at(i).y() = scan_dist * sin(scan_angle);
		}
	}
}

void Loc::ScanToCloud() {
	laser_geometry::LaserProjection projector;
	if(!listener_.waitForTransform(scan_.header.frame_id,"/robot_frame",
		scan_.header.stamp + ros::Duration().fromSec(scan_.ranges.size()*scan_.time_increment),
		ros::Duration(0.1))) {
			ROS_WARN("Got no transform");
			return;
  }
	sensor_msgs::PointCloud temp_cloud;
	try {
		projector.transformLaserScanToPointCloud("/robot_frame",scan_,cloud_,listener_);
		cloud_.header.stamp = scan_.header.stamp + ros::Duration().fromSec(scan_.ranges.size() * scan_.time_increment);
	}
	catch(tf2::ExtrapolationException) {
		ROS_WARN("Error when extrapolating");
	}
}

void Loc::ScanCallback(const sensor_msgs::LaserScan &scan) {
	if (scan.intensities.size() > 0) {	//don't take scans from old laser
		scan_ = scan;
		
	}
	else ROS_ERROR("Receiving empty laser messages");
	SetTime();
}

void Loc::OdomCallback(const localization::IOFromBoard &odom) {
	ROS_INFO("odom: right %d left %d", odom.deltaUmRight, odom.deltaUmLeft);
	last_odom_ = odom_;
	odom_ = odom;
}

void Loc::ImuCallback(const sensor_msgs::Imu &attitude) {
	//ROS_INFO("Callback");
	last_attitude_ = attitude_;	
	attitude_.header = attitude.header;
	Eigen::Quaternion<double> rotate_helper;
	const double x = attitude.orientation.x;
	const double y = attitude.orientation.y;
	const double z = attitude.orientation.z;
	const double w = attitude.orientation.w;
	rotate_helper = Eigen::Quaternion<double>(w,x,y,z);
	Eigen::Quaternion<double> rot;
	rot = Eigen::AngleAxis<double>(M_PI/2, Eigen::Vector3d(0,0,1));
	rotate_helper *= rot;	//correct weird imu cs
	rot = Eigen::AngleAxis<double>(M_PI/2, Eigen::Vector3d(0,1,0));
	rotate_helper *= rot;	//rotate to sensor mount orientation
	attitude_.orientation.x = rotate_helper.x();
	attitude_.orientation.y = rotate_helper.y();
	attitude_.orientation.z = rotate_helper.z();
	attitude_.orientation.w = rotate_helper.w();
	//remove yaw from orientation
	geometry_msgs::Quaternion temp = attitude_.orientation;
	tf::Quaternion yaw_quat = tf::createQuaternionFromYaw(tf::getYaw(temp));
	tf::Quaternion temp_quat;
	tf::quaternionMsgToTF(temp, temp_quat);
	temp_quat = yaw_quat.inverse() * temp_quat;
	tf::quaternionTFToMsg(temp_quat, temp);
	//Broadcast transform
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(0.0, 0.0, laser_height_));
	transform.setRotation(temp_quat);
	br.sendTransform(tf::StampedTransform(transform, attitude.header.stamp, "robot_frame", "laser_frame"));
}

bool Loc::InitService(localization::InitLocalization::Request &req, localization::InitLocalization::Response &res) {
	if(!initiation_ && req.init) {
		SetInit(true); 
		//ROS_ERROR("initiation for localization commented out");
		ros::Time begin = ros::Time::now();
		while(initiation_ && ros::ok() && (ros::Time::now() - begin).sec < 15) {
			pose_.pose.pose.position.x = -2000;	//for recognition if first time calculating
			odom_.timestamp = 0;	//for recognition if no odometry data
			last_odom_.timestamp = 0;
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
