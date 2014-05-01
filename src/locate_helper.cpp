#include "locate_initiate.cpp"

void Loc::NormalizeAngle(double& angle) {
  while(angle > M_PI && ros::ok()) angle -= 2*M_PI;
  while(angle < -M_PI && ros::ok()) angle += 2*M_PI;
}

void Loc::PublishPoles() {
	//ROS_INFO("Publishing poles...");
	int j = 0;
	for (int i = 0; i < poles_.size(); i++) {
		//if(poles_[i].visible()) {
			geometry_msgs::PointStamped point;
			point.header.seq = 1;
			point.header.stamp = current_time_;
			point.header.frame_id = "laser_frame";
			localization::scan_point temp_point;
			temp_point = poles_[i].laser_coords();
			point.point.x = temp_point.distance * cos(temp_point.angle);
			point.point.y = temp_point.distance * sin(temp_point.angle);
			point.point.z = 0;
			pub_pole_.publish(point);
		//}
		if (poles_[i].visible()) j++;
	}
	//ROS_INFO("seeing %d poles\n", j);
	//ROS_INFO("Success!");
}

void Loc::PublishPose() {
	//ROS_INFO("Publishing pose...");
	geometry_msgs::PoseStamped temp_pose;
	temp_pose.pose.position.x = pose_.pose.pose.position.x;
	temp_pose.pose.position.y = pose_.pose.pose.position.y;
	temp_pose.pose.orientation = pose_.pose.pose.orientation;
	temp_pose.header = pose_.header;
	pub_pose_.publish(temp_pose);
	//ROS_INFO("delay: %fms", (ros::Time::now()-current_time_).toSec()*1000);
}

void Loc::PublishMap() {
	localization::beach_map beach_map;
	for (int i = 0; i < poles_.size(); i++) {
		geometry_msgs::PointStamped point;
		point.point.x = poles_[i].xy_coords().x;
		point.point.y = poles_[i].xy_coords().y;
		beach_map.poles.push_back(point);
	}
	beach_map.basestation.pose = pose_.pose.pose;
	double yaw = tf::getYaw(beach_map.basestation.pose.orientation);
	beach_map.basestation.pose.position.x -= cos(yaw)*1.0;	//translate pose 1m against driving 
	beach_map.basestation.pose.position.y -= sin(yaw)*1.0;	//direction to get pose of base station
	pub_map_.publish(beach_map);
}

void Loc::PublishTf() {
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(pose_.pose.pose.position.x, pose_.pose.pose.position.y, 0.0));
	geometry_msgs::Quaternion quat = pose_.pose.pose.orientation;
	transform.setRotation(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
	br.sendTransform(tf::StampedTransform(transform, current_time_, "fixed_frame", "robot_frame"));
	br.sendTransform(tf::StampedTransform(transform, current_time_, "fixed_frame", "laser_frame"));
}

void Loc::PrintPose() {
	ROS_INFO("Estimate [%f %f] %f rad\n", pose_.pose.pose.position.x, pose_.pose.pose.position.y, tf::getYaw(pose_.pose.pose.orientation));
}

void Loc::PrintPoleScanData() {
	for (int i = 0; i < poles_.size(); i++) 
		if (poles_[i].visible()) ROS_INFO("found pole%d at %f m %f rad", i, poles_[i].laser_coords().distance, poles_[i].laser_coords().angle);
}

//function to fill the poles with data from the current laser scan
void Loc::RefreshData() {
	std::vector<localization::scan_point> locate_scans;	
	ExtractPoleScans(&locate_scans);	//get relevant scan points
	CorrectMoveError(&locate_scans);	
	UpdatePoles(locate_scans);		//assign scans to respective poles
	//if (locate_scans.size() == 0) ROS_WARN("Not seeing any poles");
}

void Loc::SetInit(const bool &init) {
	initiation_ = init;
	if (init) ROS_INFO("Started pole initialization");
	else ROS_INFO("Started localization");
}