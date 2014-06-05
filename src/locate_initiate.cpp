#include "locate_kalman.cpp"
#include "find_poles.cpp"
#include <laser_geometry/laser_geometry.h>

void Loc::InitiatePoles() {
	ROS_INFO("Gathering data...");
	ros::Rate loop_rate(25);
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
		ros::spinOnce();	//get one scan and corresponding pointcloud
		ScanToCloud();
		cloud.points.insert(cloud.points.end(), cloud_.points.begin(), cloud_.points.end());
		cloud.channels.insert(cloud.channels.end(), cloud_.channels.begin(), cloud_.channels.end());
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
	//serial_com->Send("roll 0 pitch 0");	//reset laser pose ot start localization and control
	//delete serial_com;
	FindPoles find_poles(cloud);
	find_poles.CalcPoles();
	std::vector<Pole::Line> lines = find_poles.GetPoles();
	if (lines.size() > 1) {
		Eigen::Vector3d translate(lines[0].p.x(), lines[0].p.y(), 0);	//translate vector to make pole 0 [0 0]
		Eigen::Vector3d second = lines[1].p - translate;
		Eigen::Matrix3d rotate;	//rotate matrix to make pole 1 [x 0]
		rotate = Eigen::AngleAxis<double>(-atan2(second.y(), second.x()), Eigen::Vector3d::UnitZ());
		for (int i = 0; i < lines.size(); i++) {
			const double x = lines[i].p.x(); const double y = lines[i].p.y();
			Eigen::Vector3d scan_point = lines[i].p;	//save coords in robot cs
			//apply transforms
			lines[i].p -= translate;
			lines[i].end -= translate;
			lines[i].p = rotate * lines[i].p;
			lines[i].end = rotate * lines[i].end;
			lines[i].u = rotate * lines[i].u;
			Pole temp_pole(lines[i], scan_point, current_time_, i);
			poles_.push_back(temp_pole);
			ROS_INFO("base for pole %d [%f %f %f]", i, lines[i].p.x(), lines[i].p.y(), lines[i].p.z() );
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

void Loc::GetPose() {
	Eigen::Vector2d x(0,0), x_old(2000,2000);
	while ( (x_old - x).norm() > 0.01) {
		x_old = x;
		Eigen::MatrixXd jacobi(poles_.size(), 2);
		Eigen::VectorXd f_x(poles_.size() );
		Eigen::VectorXd c(poles_.size() );
		for (int i = 0; i < poles_.size(); i++) {
			Eigen::Vector2d x_p( poles_[i].line().p.x(), poles_[i].line().p.y());
			Eigen::Vector2d x_m( poles_[i].laser_coords().x(), poles_[i].laser_coords().y());
			jacobi(i, 0) = 2 * x_old.x() - 2 * x_p.x();
			jacobi(i, 1) = 2 * x_old.y() - 2 * x_p.y();
			f_x(i) = (x_p.x() - x_old.x() ) * (x_p.x() - x_old.x() ) + (x_p.y() - x_old.y() ) * (x_p.y() - x_old.y() );
			c(i) = x_m.x() * x_m.x() + x_m.y() * x_m.y();
		}
		x += jacobi.colPivHouseholderQr().solve(c - f_x);
		ROS_INFO("iter");
	}	
	ROS_INFO("initial pos [%f %f]", x.x(), x.y() );
	double theta = 0;
	for (int i = 0; i < poles_.size(); i++) {	//average over all results
		Eigen::Vector2d x_p( poles_[i].line().p.x(), poles_[i].line().p.y());
		Eigen::Vector2d x_m( poles_[i].laser_coords().x(), poles_[i].laser_coords().y());
		const double cur_theta = atan2( x_p.y() - x.y(), x_p.x() - x.x() ) - atan2( x_m.y(), x_m.x() );
		ROS_INFO("cur_theta %f", cur_theta);
		theta += cur_theta;
	}
	theta /= poles_.size();
	ROS_INFO("theta %f", theta);
	pose_.pose.pose.position.x = x.x();
	pose_.pose.pose.position.y = x.y();
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
