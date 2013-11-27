#include "locate_helper.cpp"
#include "localization/scan_point.h"
#include "tf/transform_datatypes.h"

Loc::Loc() {
	ROS_INFO("Started localization node");
	sub_ = n_.subscribe("/scan",1000, &Loc::Callback, this);
	ROS_INFO("Subscribed to \"scan\" topic");
	pub_pose_ = n_.advertise<geometry_msgs::PoseStamped>("bot_pose",1000);
	pub_pole_ = n_.advertise<geometry_msgs::PointStamped>("pole_pos",1000);
	initiation_ = true;	//start with initiation
	pose_.pose.pose.position.x = -2000;	//for recognition if first time calculating
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
	ros::spinOnce();
	std::vector<localization::scan_point> locate_scans;	//TODO: put most of the following stuff in callback
	ExtractPoleScans(&locate_scans);	//get relevant scan points
	if (locate_scans.size() > 1) UpdatePoles(locate_scans);		//assign scans to respective poles
	else ROS_WARN("Only seeing %lu poles. At least 2 needed.", locate_scans.size());
	PublishPoles();
	PublishPose();
	loop_rate.sleep();
}

void Loc::UpdatePoles(const std::vector<localization::scan_point> &scans_to_sort) {
	ros::Time current_time = ros::Time::now();
	for(int i = 0; i < scans_to_sort.size(); i++) {	//find closest pole for every scan
		double min_dist = 2000000;
		int index = -1;
		for (int j = 0; j < poles_.size(); j++) {
			localization::scan_point current_scan = poles_[j].laser_coords();
			double current_dist = pow(scans_to_sort[i].distance*cos(scans_to_sort[i].angle) - current_scan.distance*cos(current_scan.angle),2)
			+ pow(scans_to_sort[i].distance*sin(scans_to_sort[i].angle) - current_scan.distance*sin(current_scan.angle),2);
			if (current_dist < min_dist) {
				min_dist = current_dist;
				index = j;
			}
		}
		assert(index != -1);
		poles_[index].update(scans_to_sort[i], current_time);
	}
	for (int i = 0; i < poles_.size(); i++) {	//hide all missing poles
		if (poles_[i].time() != current_time) poles_[i].disappear();
	}
	PrintPoleScanData();
	GetPose();
	EstimateInvisiblePoles();
	PrintPose();
}

void Loc::GetPose() {
	std::vector<geometry_msgs::Pose> pose_vector;
	for (int i = 0; i < poles_.size(); i++) {		//loop over poles
		if (!poles_[i].visible()) continue;
		int j = i+1;
		while (!poles_[j].visible() && j < poles_.size()) j++;
		if (j > poles_.size()-1) break;
		calcPose(poles_[i], poles_[j], &pose_vector);
		i = j;
	}
	double x = 0, y = 0, theta = 0;
	//ROS_INFO("y: %f", y);
	for (int i = 0; i < pose_vector.size(); i++) {	//average over all results
		x += pose_vector[i].position.x;
		y += pose_vector[i].position.y;
		//ROS_INFO("y: %f", y);
		theta += tf::getYaw(pose_vector[i].orientation);
	}
	x /= pose_vector.size();
	y /= pose_vector.size();
	theta /= pose_vector.size();
	pose_.pose.pose.position.x = x;
	pose_.pose.pose.position.y = y;
	pose_.pose.pose.position.z = 0;
	pose_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
	pose_.header.seq = 1;
	pose_.header.stamp = ros::Time::now();
	pose_.header.frame_id = "fixed_frame";
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
			ROS_INFO("Changed pole %d to %f m %f rad", i, temp_scan.distance, temp_scan.angle);
		}
	}
	//ROS_INFO("Done estimating");
}

void Loc::PrintPose() {
	ROS_INFO("Averaged [%f %f] %f rad\n", pose_.pose.pose.position.x, pose_.pose.pose.position.y, tf::getYaw(pose_.pose.pose.orientation));
}

void Loc::PrintPoleScanData() {
	for (int i = 0; i < poles_.size(); i++) 
		ROS_INFO("found pole%d at %f m %f rad", i, poles_[i].laser_coords().distance, poles_[i].laser_coords().angle);
}

void Loc::calcPose(const Pole &pole1, const Pole &pole2, std::vector<geometry_msgs::Pose> *pose_vector) {
  //pole coordinates
  double xp1 = pole1.xy_coords().x;
  double yp1 = pole1.xy_coords().y;
  double xp2 = pole2.xy_coords().x;
  double yp2 = pole2.xy_coords().y;

  //scan coordinates
  double a_dist = pole1.laser_coords().distance;
  double a_ang = pole1.laser_coords().angle;
  double b_dist = pole2.laser_coords().distance;
  double b_ang = pole2.laser_coords().angle;

  geometry_msgs::Pose temp_pose;
  temp_pose.position.z = 0;

  //calculate possible points
  const double D = pow((xp2-xp1)*(xp2-xp1)+(yp2-yp1)*(yp2-yp1),0.5);
  double to_root = (D+a_dist+b_dist)*(D+a_dist-b_dist)*(D-a_dist+b_dist)*(-D+a_dist+b_dist);	//to check if circles have intersection
  const int max_iter = 50;
  int iter = 0;
  while (to_root < 0 && ros::ok() && iter < max_iter) {		//if no intersection slowly widen circles
  	if(a_dist > pow(xp1*xp1+xp2*xp2,0.5)+b_dist) {
  		a_dist -= 0.01;
  		b_dist += 0.01;
  	}
  	if(b_dist > pow(xp1*xp1+xp2*xp2,0.5)+b_dist) {
  		a_dist += 0.01;
  		b_dist -= 0.01;
  	}
  	else {
  		a_dist += 0.01;
  		b_dist += 0.01;
  	}
  	ROS_INFO("corrected a_dist to %f", a_dist);
  	ROS_INFO("corrected b_dist to %f", b_dist);
  	to_root = (D+a_dist+b_dist)*(D+a_dist-b_dist)*(D-a_dist+b_dist)*(-D+a_dist+b_dist);
  	//ROS_INFO("to root: %f", to_root);
  	//ROS_INFO("corrected");
  	iter++;
  }
  const double delta = 1.0/4*pow(to_root,0.5);
  const double x1_circle = (xp1+xp2)/2+(xp2-xp1)*(a_dist*a_dist-b_dist*b_dist)/(2*D*D) + 2*(yp1-yp2)/(D*D)*delta;
  const double x2_circle = (xp1+xp2)/2+(xp2-xp1)*(a_dist*a_dist-b_dist*b_dist)/(2*D*D) - 2*(yp1-yp2)/(D*D)*delta;
  const double y1_circle = (yp1+yp2)/2+(yp2-yp1)*(a_dist*a_dist-b_dist*b_dist)/(2*D*D) - 2*(xp1-xp2)/(D*D)*delta;
  const double y2_circle = (yp1+yp2)/2+(yp2-yp1)*(a_dist*a_dist-b_dist*b_dist)/(2*D*D) + 2*(xp1-xp2)/(D*D)*delta;
  
  //bot orientation for possible points
  double theta1_circle = M_PI - a_ang + atan2(y1_circle-yp1,x1_circle-xp1);
  double theta2_circle = M_PI - a_ang + atan2(y2_circle-yp1,x2_circle-xp1);
  ROS_INFO("P1C [%f %f] %f", x1_circle, y1_circle, theta1_circle);
  ROS_INFO("P2C [%f %f] %f", x2_circle, y2_circle, theta2_circle);

  if (pose_.pose.pose.position.x != -2000) {
  	//////////////////Newton Method////////////////////////
    NormalizeAngle(theta1_circle);
    NormalizeAngle(theta2_circle);
  	double theta_old = -2000;
  	double theta_newton = tf::getYaw(pose_.pose.pose.orientation);
  	int it = 0;
  	while(std::abs(theta_newton - theta_old) > 0.001 && ros::ok() && it < 5) {
  		theta_old = theta_newton;
  		//ROS_INFO("theta_old %f", theta_old);
  		double f_x = a_dist*cos(a_ang + theta_old -M_PI) +xp1 -b_dist*cos(b_ang + theta_old - M_PI) -xp2;
  		//ROS_INFO("f_x %15.15f", f_x);
  		double f_x_prime = -a_dist*sin(a_ang + theta_old -M_PI) + b_dist*sin(b_ang + theta_old - M_PI);
  		//ROS_INFO("f_x_prime %15.15f", f_x_prime);
  		theta_newton = theta_old - (f_x/f_x_prime);
  		//ROS_INFO("Newton iteration");
  		it++;
  	}
  	const double alpha1 = a_ang +theta_newton -M_PI;
  	const double alpha2 = b_ang +theta_newton -M_PI;
  	const double x1_newton = a_dist*cos(alpha1)+xp1;
  	const double x2_newton = b_dist*cos(alpha2)+xp2;
  	const double y1_newton = a_dist*sin(alpha1)+yp1;
  	const double y2_newton = b_dist*sin(alpha2)+yp2;
  	const double x_newton = (x1_newton+x2_newton)/2;
  	const double y_newton = (y1_newton+y2_newton)/2;
  	NormalizeAngle(theta_newton);
  	ROS_INFO("P_N [%f %f] %f", x_newton, y_newton, theta_newton);
  	const double check_dist_newton1 = pow(pow(x1_circle-x_newton,2)+pow(y1_circle-y_newton,2),0.5);
  	const double check_dist_newton2 = pow(pow(x2_circle-x_newton,2)+pow(y2_circle-y_newton,2),0.5);
    if (check_dist_newton1 < 0.1 || check_dist_newton2 < 0.1) {		//only use when Newton estimate is good
	    	if (check_dist_newton1 < check_dist_newton2) {	//use newton's method to find best circle point
	    		const double check_dist1 = pow(pow(x1_circle-pose_.pose.pose.position.x,2)+pow(y1_circle-pose_.pose.pose.position.y,2),0.5);
	    		if (check_dist1 < 1) {	//check if newton point is close by chance
		    		temp_pose.orientation = tf::createQuaternionMsgFromYaw(theta1_circle); 
		    		temp_pose.position.x = x1_circle; 
		    		temp_pose.position.y = y1_circle;
		    	}
	    	}
	    	else {
	    		const double check_dist2 = pow(pow(x2_circle-pose_.pose.pose.position.x,2)+pow(y2_circle-pose_.pose.pose.position.y,2), 0.5);
	    		if (check_dist2 < 1) {
		    		temp_pose.orientation = tf::createQuaternionMsgFromYaw(theta2_circle); 
		    		temp_pose.position.x = x2_circle; 
		    		temp_pose.position.y = y2_circle;
		    	}
	    	}
    }
    else {	//use closest point to last measurement
    	const double check_dist1 = pow(pow(x1_circle-pose_.pose.pose.position.x,2)+pow(y1_circle-pose_.pose.pose.position.y,2),0.5);
  		const double check_dist2 = pow(pow(x2_circle-pose_.pose.pose.position.x,2)+pow(y2_circle-pose_.pose.pose.position.y,2), 0.5);
  		if (check_dist1 < check_dist2) {	//use newton's method to find best circle point
    		temp_pose.orientation = tf::createQuaternionMsgFromYaw(theta1_circle); 
    		temp_pose.position.x = x1_circle; 
    		temp_pose.position.y = y1_circle;
    	}
    	else {
    		temp_pose.orientation = tf::createQuaternionMsgFromYaw(theta2_circle); 
    		temp_pose.position.x = x2_circle; 
    		temp_pose.position.y = y2_circle;
    	}
    }
  }
  else {	//no Newton if first time
  	//check which pose is the correct one 
  	const bool first = (M_PI + atan2(y1_circle-yp2,x1_circle-xp2)-theta1_circle-b_ang < 0.1);
  	const bool second = (M_PI + atan2(y1_circle-yp2,x1_circle-xp2)-theta2_circle-b_ang < 0.1);
  	assert(first || second);
  	NormalizeAngle(theta1_circle);
  	NormalizeAngle(theta2_circle);
  	//put in correct pose
  	if (first) {
  		temp_pose.orientation = tf::createQuaternionMsgFromYaw(theta1_circle); 
  		temp_pose.position.x = x1_circle; 
  		temp_pose.position.y = y1_circle;
  	}
  	else {
  		temp_pose.orientation = tf::createQuaternionMsgFromYaw(theta2_circle); 
  		temp_pose.position.x = x2_circle; 
  		temp_pose.position.y = y2_circle;
  	}
  }

  //print pose 
  ROS_INFO("From poles %d,%d: [%f %f] %f rad", pole1.i(), pole2.i(), temp_pose.position.x, temp_pose.position.y, tf::getYaw(temp_pose.orientation));
  pose_vector->push_back(temp_pose);
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
						if (std::abs((scan->at(i).angle - scan->at(j).angle)*scan->at(i).distance) < 1
							&& std::abs(scan->at(i).distance - scan->at(j).distance) < 1) {
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

void Loc::Callback(const sensor_msgs::LaserScan &scan) {
	scan_ = scan;
	//ROS_INFO("scan %d", scan.header.seq);
	//ROS_INFO("scan_ %d", scan_.header.seq);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "localization");
	Loc *loc = new Loc();
	ROS_INFO("Location node shutting down!");
}