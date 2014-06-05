#include "locate_kalman.cpp"
#include "find_poles.cpp"
#include <laser_geometry/laser_geometry.h>
#include "tf/transform_listener.h"

void Loc::InitiatePoles() {
	ROS_INFO("Gathering data...");
	ros::Rate loop_rate(25);
	laser_geometry::LaserProjection projector;
	tf::TransformListener listener;
	//Read parameters for initial scanning
	std::string address;
	double rev_time, roll_min, roll_max, pitch_min, pitch_max;
	if (ros::param::get("address", address));	//get address from parameters
	else {
		address = "dev/ttyUSB0"; ROS_WARN("Did not find config for motor controller address!");
	}
	if (ros::param::get("T", rev_time));	//get revolution time
	else {
		rev_time = 5; ROS_WARN("Did not find config for revolution time");
	}
	if (ros::param::get("roll_min", roll_min));	//get revolution time
	else {
		roll_min = -0.175; ROS_WARN("Did not find config for roll_min");
	}
	if (ros::param::get("roll_max", roll_max));	//get revolution time
	else {
		roll_max = 0.115; ROS_WARN("Did not find config for roll_max");
	}
	if (ros::param::get("pitch_min", pitch_min));	//get revolution time
	else {
		pitch_min = -0.095; ROS_WARN("Did not find config for pitch_min");
	}
	if (ros::param::get("pitch_max", pitch_max));	//get revolution time
	else {
		pitch_max = 0.193; ROS_WARN("Did not find config for pitch_max");
	}
	const double roll_amp = (roll_max - roll_min) / 2, pitch_amp = (pitch_max - pitch_min) / 2;	//angle amplitudes
	const double roll_mid = (roll_max + roll_min) / 2, pitch_mid = (pitch_max + pitch_min) / 2;	//angle midpoints
	//SerialCom *serial_com = new SerialCom(address);	//open serial communication
	sensor_msgs::PointCloud cloud;
	ros::Time begin = ros::Time::now();
	while ((ros::Time::now() - begin).toSec() < (rev_time + 2) && ros::ok()) {	//gather data for T + 2 seconds
		ros::spinOnce();	//get one scan
		//add new scan to pointcloud
		if(!listener.waitForTransform(scan_.header.frame_id,"/robot_frame",
        scan_.header.stamp + ros::Duration().fromSec(scan_.ranges.size()*scan_.time_increment),
        ros::Duration(1))){
     			ROS_WARN("Got no transform");
     			return;
  	}
		sensor_msgs::PointCloud temp_cloud;
		try {
			projector.transformLaserScanToPointCloud("/robot_frame",scan_,temp_cloud,listener);
			cloud.header.stamp = scan_.header.stamp + ros::Duration().fromSec(scan_.ranges.size() * scan_.time_increment);
			cloud.points.insert(cloud.points.end(), temp_cloud.points.begin(), temp_cloud.points.end());
			cloud.channels.insert(cloud.channels.end(), temp_cloud.channels.begin(), temp_cloud.channels.end());
		}
		catch(tf2::ExtrapolationException) {
			ROS_WARN("Error when extrapolating");			
		}
		//set new laser angle
		const double current = (ros::Time::now() - begin).toSec();
		const double roll = roll_mid + roll_amp * sin(current / rev_time * 2 * M_PI);
		const double pitch = pitch_mid + pitch_amp * cos(current / rev_time * 2*  M_PI);
		const int roll_data = roll * 1000 / M_PI * 180;	//controller wants degree*1000
		const int pitch_data = pitch * 1000 / M_PI * 180;	
		std::string data = "roll ";
		stringstream ss;
		ss << roll_data << " pitch " << pitch_data;
		data.append(ss.str());
		//ROS_INFO("%s", data.c_str());
		//serial_com->Send(data);
		loop_rate.sleep();
	}
	//delete serial_com;
	FindPoles find_poles(cloud);
	find_poles.CalcPoles();
	std::vector<Pole::Line> lines = find_poles.GetPoles();
	if (lines.size() > 1) {
		Eigen::Vector3d translate = lines[0].p;	//translate vector to make pole 0 [0 0]
		Eigen::Vector3d second = lines[1].p - translate;
		Eigen::Matrix3d rotate;	//rotate matrix to make pole 1 [x 0]
		rotate = Eigen::AngleAxis<double>(-atan2(second.y(), second.x()), Eigen::Vector3d::UnitZ());
		for (int i = 0; i < lines.size(); i++) {
			const double x = lines[i].p.x(); const double y = lines[i].p.y();
			const double scan_dist = sqrt((x * x) + (y * y));
			const double scan_angle = atan2(y, x);
			localization::scan_point scan_point;
			scan_point.distance = scan_dist;
			scan_point.angle = scan_angle;
			//apply transforms
			lines[i].p -= translate;
			lines[i].end -= translate;
			lines[i].p = rotate * lines[i].p;
			lines[i].end = rotate * lines[i].end;
			lines[i].u = rotate * lines[i].u;
			Pole temp_pole(lines[i], scan_point, current_time_, i);
			poles_.push_back(temp_pole);
		}
		PublishPoles();
		SetInit(false);
	}
	else ROS_WARN("Only found %lu poles. At least 2 needed.", lines.size());
	//get first initial pose for kalman filter
	RefreshData();
	GetPose();
	initial_pose_.pose = pose_.pose.pose;
	initial_pose_.header = pose_.header;
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


