#include "locate_initiate.cpp"
#include <algorithm>

void Loc::NormalizeAngle(double& angle) {
  while(angle > M_PI && ros::ok()) angle -= 2*M_PI;
  while(angle < -M_PI && ros::ok()) angle += 2*M_PI;
}

void Loc::PublishPoles() {
	//ROS_INFO("Publishing poles...");
	int j = 0;
	visualization_msgs::Marker line_list;
	for (int i = 0; i < poles_.size(); i++) {
		line_list.header = cloud_.header;
		line_list.header.frame_id = "fixed_frame";
		line_list.ns = "points_and_lines";
		line_list.action = visualization_msgs::Marker::ADD;
		line_list.pose.orientation.w = 1.0;
		line_list.id = 0;
		line_list.type = visualization_msgs::Marker::LINE_LIST;
		line_list.scale.x = poles_[0].line().d;
		line_list.color.b = 1.0;
		line_list.color.a = 1.0;
		if(poles_[i].visible()) {
			geometry_msgs::PointStamped point;
			point.header.seq = 1;
			point.header.stamp = current_time_;
			point.header.frame_id = "robot_frame";
			Eigen::Vector3d temp_point;
			temp_point = poles_[i].laser_coords();
			point.point.x = temp_point.x();
			point.point.y = temp_point.y();
			point.point.z = temp_point.z();
			pub_pole_.publish(point);
		}
		geometry_msgs::Point start, end;
		start.x = poles_[i].line().p.x(); start.y = poles_[i].line().p.y(); start.z = poles_[i].line().p.z();
		end.x = poles_[i].line().end.x(); end.y = poles_[i].line().end.y(); end.z = poles_[i].line().end.z();
		line_list.points.push_back(start);
		line_list.points.push_back(end);
		if (poles_[i].visible()) j++;
	}
	pub_marker_.publish(line_list);
	ROS_INFO("seeing %d poles", j);
	//ROS_INFO("Success!");
}

void Loc::PublishCloud(const sensor_msgs::PointCloud &cloud) {
	pub_cloud_.publish(cloud);
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
	
bool sortByAngle(Pole i, Pole j) {return atan2(i.laser_coords().y(), i.laser_coords().x())
	 < atan2(j.laser_coords().y(), j.laser_coords().x());}

void Loc::PublishMap() {
	localization::beach_map beach_map;
	std::vector<Pole> sorted_poles = poles_;
	std::sort(sorted_poles.begin(), sorted_poles.end(), sortByAngle);
	for (int i = 0; i < sorted_poles.size(); i++) {
		geometry_msgs::PointStamped point;
		point.point.x = sorted_poles[i].line().p.x();
		point.point.y = sorted_poles[i].line().p.y();
		point.point.z = sorted_poles[i].line().p.z();
		beach_map.poles.push_back(point);
		localization::line line;
		line.p.x = sorted_poles[i].line().p.x();
		line.p.y = sorted_poles[i].line().p.y();
		line.p.z = sorted_poles[i].line().p.z();
		line.u.x = sorted_poles[i].line().u.x();
		line.u.y = sorted_poles[i].line().u.y();
		line.u.z = sorted_poles[i].line().u.z();
		line.end.x = sorted_poles[i].line().end.x();
		line.end.y = sorted_poles[i].line().end.y();
		line.end.z = sorted_poles[i].line().end.z();
		line.d = sorted_poles[i].line().d;
		beach_map.lines.push_back(line);
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
}

void Loc::PrintPose() {
	ROS_INFO("Estimate [%f %f] %f rad\n", pose_.pose.pose.position.x, pose_.pose.pose.position.y, tf::getYaw(pose_.pose.pose.orientation));
}

//function to fill the poles with data from the current laser scan
void Loc::RefreshData() {
	std::vector<Eigen::Vector3d> locate_scans;	
	MinimizeScans(&locate_scans);	//get relevant scan points
	CorrectMoveError(&locate_scans);	
	UpdatePoles(locate_scans);		//assign scans to respective poles
}

void Loc::SetInit(const bool &init) {
	initiation_ = init;
	if (init) ROS_INFO("Started pole initialization");
	else ROS_INFO("Started localization");
}