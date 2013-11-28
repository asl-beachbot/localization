#include "locate_initiate.cpp"

void Loc::NormalizeAngle(double& angle) {
  while(angle > M_PI) angle -= 2*M_PI;
  while(angle < -M_PI) angle += 2*M_PI;
}

void Loc::PublishPoles() {
	//ROS_INFO("Publishing poles...");
	for (int i = 0; i < poles_.size(); i++) {
		geometry_msgs::PointStamped point;
		point.header.seq = 1;
		point.header.stamp = current_time_;
		point.header.frame_id = "fixed_frame";
		point.point.x = poles_[i].xy_coords().x;
		point.point.y = poles_[i].xy_coords().y;
		point.point.z = 0;
		pub_pole_.publish(point);
	}
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
	//ROS_INFO("Success!");
}

void Loc::PrintPose() {
	ROS_INFO("Averaged [%f %f] %f rad\n", pose_.pose.pose.position.x, pose_.pose.pose.position.y, tf::getYaw(pose_.pose.pose.orientation));
}

void Loc::PrintPoleScanData() {
	for (int i = 0; i < poles_.size(); i++) 
		ROS_INFO("found pole%d at %f m %f rad", i, poles_[i].laser_coords().distance, poles_[i].laser_coords().angle);
}