#include "locate_kalman.cpp"

void Loc::InitiatePoles() {
	ros::Rate loop_rate(25);
	ros::Time begin = ros::Time::now();
	std::vector<std::vector<localization::scan_point> > extracted_scan_points;
	ROS_INFO("Gathering data...");
	double init_duration = 5;
	while ((ros::Time::now()-begin).sec < init_duration && ros::ok()) {	//gather data for 5 seconds
		ros::spinOnce();	//get one scan
		if ((std::abs(odom_.twist.twist.linear.x) > 0.01 || std::abs(odom_.twist.twist.angular.z) > 0.02) && use_odometry_) {
			//Reset initialization if robot move is detected
			ROS_WARN("Robot moved! Restarting Initialization.");
			StateHandler();
		}
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
		MinimizeScans(&averaged_scan_points, 25);		//average over all measurements
		for (int i = 0; i < averaged_scan_points.size(); i++) {	
			//ROS_INFO("pole (polar) at %f m %f rad", averaged_scan_points[i].distance, averaged_scan_points[i].angle);
			//if (i == 0) ROS_INFO("error %f m",averaged_scan_points[i].distance-6.666667);		//check distance errors
			//if (i == 1) ROS_INFO("error %f m",averaged_scan_points[i].distance-8.91667);
		}
		if (averaged_scan_points.size() > 1) {
			std::vector<localization::xy_point> xy_poles = ScanToXY(averaged_scan_points);
			//for (int i = 0; i < xy_poles.size(); i++) ROS_INFO("pole (kart.) at [%f %f]", xy_poles[i].x, xy_poles[i].y);	//print poles for debugging
			for (int i = 0; i < averaged_scan_points.size(); i++) {	//fill pole vector
				poles_.push_back(Pole(xy_poles[i], averaged_scan_points[i], scan_.header.stamp, i));
			}
			PublishPoles();
			SetInit(false);
		}
		else ROS_WARN("Only found %lu poles. At least 2 needed.", averaged_scan_points.size());
	}
	//get first initial pose for kalman filter
	GetPose();
	initial_pose_.pose = pose_.pose.pose;
	initial_pose_.header = pose_.header;
	RefreshData();
	EstimateInvisiblePoles();
	//PrintPose();
	PublishPoles();
	PublishPose();
	PublishMap();
	loop_rate.sleep();
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

void Loc::GetPose() {
	std::vector<geometry_msgs::Pose> pose_vector;
	for (int i = 0; i < poles_.size(); i++) {		//loop over poles
		if (!poles_[i].visible()) continue;
		int j = i+1;
		while (!poles_[j].visible() && j < poles_.size() && ros::ok()) j++;
		if (j > poles_.size()-1) break;
		CalcPose(poles_[i], poles_[j], &pose_vector);
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
	pose_.header.stamp = current_time_;
	pose_.header.frame_id = "fixed_frame";
	//initiate probability matrix
	for (int i = 0; i < pose_.pose.covariance.size(); i++) pose_.pose.covariance[i] = 0;
	pose_.pose.covariance[0] = 0.1;
	pose_.pose.covariance[7] = 0.1;
	pose_.pose.covariance[35] = 0.1;
}

//used to calculate initial position
void Loc::CalcPose(const Pole &pole1, const Pole &pole2, std::vector<geometry_msgs::Pose> *pose_vector) {
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
  //ROS_INFO("P1C [%f %f] %f", x1_circle, y1_circle, theta1_circle);
  //ROS_INFO("P2C [%f %f] %f", x2_circle, y2_circle, theta2_circle);

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
  	const bool first = (M_PI + atan2(y1_circle-yp2,x1_circle-xp2)-theta1_circle-b_ang < M_PI);
  	const bool second = (M_PI + atan2(y1_circle-yp2,x1_circle-xp2)-theta2_circle-b_ang < M_PI);
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
  //ROS_INFO("From poles %d,%d: [%f %f] %f rad", pole1.i(), pole2.i(), temp_pose.position.x, temp_pose.position.y, tf::getYaw(temp_pose.orientation));
  pose_vector->push_back(temp_pose);
}


