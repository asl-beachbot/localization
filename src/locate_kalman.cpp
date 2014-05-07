#include "locate.h"
#include <iostream>

void Loc::DoTheKalman() {
	//create eigen vector and matrix from ros message
	Eigen::Vector3d state;
	state[0] = pose_.pose.pose.position.x;
	state[1] = pose_.pose.pose.position.y;
	state[2] = tf::getYaw(pose_.pose.pose.orientation);
	Eigen::Matrix3d covariance;
	covariance << 
		pose_.pose.covariance[0], 0, 0,
		0, pose_.pose.covariance[7], 0, 
		0, 0, pose_.pose.covariance[35];
	//ROS_INFO("covariance %f", covariance(0,0));

	//predict
	if (odom_.pose.pose.position.x != -2000.0 && last_odom_.pose.pose.position.x != -2000.0 && use_odometry_) {
		const double predicted_theta = tf::getYaw(odom_.pose.pose.orientation);
		const double last_theta = tf::getYaw(last_odom_.pose.pose.orientation);
		const double current_theta = tf::getYaw(pose_.pose.pose.orientation);
		double dx = (odom_.pose.pose.position.x - last_odom_.pose.pose.position.x);
		double dy = (odom_.pose.pose.position.y - last_odom_.pose.pose.position.y);
		double theta_delta_robot_cs = predicted_theta - last_theta;
		const double delta_t_odom = (odom_.header.stamp - last_odom_.header.stamp).toSec();
		const double delta_t_pose = (current_time_ - pose_.header.stamp).toSec();
		assert(delta_t_pose != 0 && delta_t_odom != 0);
		const double time_scale = delta_t_pose/delta_t_odom;
		const double delta_s = pow(dx *dx + dy * dy, 0.5)*time_scale;
		theta_delta_robot_cs *= time_scale;
		const double x_delta_robot_cs = delta_s * cos(theta_delta_robot_cs/2);
		const double y_delta_robot_cs = delta_s * sin(theta_delta_robot_cs/2);
		state[0] += (x_delta_robot_cs * cos(current_theta) + y_delta_robot_cs * sin(current_theta));
		state[1] += -(x_delta_robot_cs * sin(-current_theta) + y_delta_robot_cs * cos(current_theta));
		state[2] += theta_delta_robot_cs;

		const double ds = (odom_.pose.pose.position.x - last_odom_.pose.pose.position.x) * cos(tf::getYaw(last_odom_.pose.pose.orientation))
			+ (odom_.pose.pose.position.y - last_odom_.pose.pose.position.y) * sin(tf::getYaw(last_odom_.pose.pose.orientation));
		const double dth = tf::getYaw(odom_.pose.pose.orientation) - tf::getYaw(last_odom_.pose.pose.orientation);
		const double theta = tf::getYaw(pose_.pose.pose.orientation);
		
		//state += PredictPositionDelta();	//position delta resulting from odometry prediction
		Eigen::Matrix3d f_x = StateJacobi(ds, dth, theta);
		covariance = f_x*covariance*f_x.transpose();
		Eigen::MatrixXd f_u = InputJacobi(ds, dth, theta);

		covariance += f_u*Q(ds, dth)*f_u.transpose();
		ROS_INFO("everything");
	}
	//ROS_INFO("cov_pred_end: x %f y %f th %f", covariance(0,0), covariance(1,1), covariance(2,2));
	else if (last_pose_.pose.pose.position.x != -2000 && last_attitude_.orientation.x != -2000) {	
	//no odometry --> enlarge covariance; use old pose and imu to predict
		const double delta_t_pose = (current_time_ - pose_.header.stamp).toSec();
		const double delta_t_imu = (attitude_.header.stamp - last_attitude_.header.stamp).toSec();
		const double delta_t_old = (pose_.header.stamp - last_pose_.header.stamp).toSec();
		const double time_scale_imu = delta_t_pose/delta_t_imu;
		const double time_scale_pose = delta_t_pose/delta_t_old;
		const double delta_x = (pose_.pose.pose.position.x - last_pose_.pose.pose.position.x);
		const double delta_y = (pose_.pose.pose.position.y - last_pose_.pose.pose.position.y);
		const double delta_s = pow(delta_x * delta_x + delta_y * delta_y, 0.5);
		const double last_theta = tf::getYaw(last_attitude_.orientation);
		const double this_theta = tf::getYaw(attitude_.orientation);
		ROS_INFO("theta %f", this_theta);
		double delta_theta = (this_theta - last_theta);
		NormalizeAngle(delta_theta);	//prevent angle difference error when going from -pi to pi
		ROS_INFO("v_theta: %f", delta_theta/(current_time_ - pose_.header.stamp).toSec());
		//state update
		state[2] += delta_theta/2*time_scale_imu;	//use leapfrog to find x,y
		state[0] += cos(state[2])*delta_s*time_scale_pose;
		state[1] += sin(state[2])*delta_s*time_scale_pose;
		//covariance update
		Eigen::Matrix3d f_x;
		f_x << 
			1, 0, -sin(state[2])*delta_s*time_scale_pose,
			0, 1, cos(state[2])*delta_s*time_scale_pose,
			0, 0, 1;
		Eigen::MatrixXd f_u(3,2);
		f_u(0,0) = cos(state[2])*time_scale_pose; f_u(0,1) = -sin(state[2])*delta_s*time_scale_pose;
		f_u(1,0) = sin(state[2])*time_scale_pose; f_u(1,1) = cos(state[2])*delta_s*time_scale_pose;
		f_u(2,0) = 0; f_u(2,1) = time_scale_imu;
		Eigen::Matrix2d q_t;
		q_t(0,0) = delta_s*time_scale_pose*k_s_; q_t(0,1) = 0;
		q_t(1,0) = 0; q_t(1,1) = delta_theta*time_scale_imu*k_th_;
		covariance = f_x*covariance*f_x.transpose() + f_u*q_t*f_u.transpose();
		state[2] += delta_theta/2*time_scale_imu;	//second leap frog step later because cov uses intermediate angle
		ROS_INFO("No odom but laser");
	}
	else {	//no laser, just enlarge convariance
		ROS_INFO("No odom no laser");
	}
	//Write prediction so poles can be assigned properly
	pred_pose_.position.x = state[0];
	pred_pose_.position.y = state[1];
	pred_pose_.orientation = tf::createQuaternionMsgFromYaw(state[2]);
	RefreshData();
	//measure
	std::vector<Pole> visible_poles;	//get all visible poles
	for (int i = 0; i < poles_.size(); i++) if (poles_[i].visible()) visible_poles.push_back(poles_[i]);
	if (visible_poles.size() > 0) {		//dont make scan step if no poles visible
		Eigen::VectorXd h_x = EstimateReferencePoint(visible_poles, state);
		Eigen::MatrixXd H = EstimateJacobi(visible_poles, state);
		Eigen::MatrixXd R = ErrorMatrix(visible_poles);
		Eigen::VectorXd z = CalculateMeasuredPoints(visible_poles);
		Eigen::MatrixXd Sigma = H*covariance*H.transpose()+R;
		Eigen::MatrixXd K = covariance*H.transpose()*Sigma.inverse();
		Eigen::VectorXd nu = z-h_x;
		//std::cout << "nu\n" << nu << std::endl;
		state += K*(nu);	//update state with measurement
		covariance -= K*Sigma*K.transpose();	//update covariance with measurement
	}
	//std::cout << "cov\n" << covariance << std::endl;
	
	//write vector and matrix back to ros message
	last_pose_ = pose_;
	pose_.header.stamp = current_time_;
	pose_.pose.pose.position.x = state[0]; // - 0.07*cos(state[2]);
	pose_.pose.pose.position.y = state[1]; // - 0.07*sin(state[2]);
	pose_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(state[2]);
	pose_.pose.covariance[0] = covariance(0,0);
	pose_.pose.covariance[7] = covariance(1,1);
	pose_.pose.covariance[35] = covariance(2,2);
	//ROS_INFO("cov_end: x %f y %f th %f", pow(covariance(0,0),0.5), pow(covariance(1,1),0.5), pow(covariance(2,2),0.5));
	//reset odometry
	if (odom_.pose.pose.position.x != -2000.0) last_odom_ = odom_;
	odom_.pose.pose.position.x = -2000.0;
	//reset laser
	scan_.intensities.clear();
	scan_.ranges.clear();
}

Eigen::Matrix3d Loc::StateJacobi(const double &ds, const double &dth, const double &theta) {
	Eigen::Matrix3d f_x;
	f_x << 
		1, 0, -ds*sin(theta+dth/2),
		0, 1, ds*cos(theta + dth/2),
		0, 0, 1;
	return f_x;
}

Eigen::MatrixXd Loc::InputJacobi(const double &ds, const double &dth, const double &theta) {
	//ROS_INFO("ds %f dth %f theta %f", ds, dth, theta);
	Eigen::MatrixXd f_u(3,2);
	f_u << 
		0.5*cos(theta + dth/2)-ds/(2*b)*sin(theta + dth/2), 0.5*cos(theta + dth/2)+ds/(2*b)*sin(theta + dth/2),
		0.5*sin(theta + dth/2)+ds/(2*b)*cos(theta + dth/2), 0.5*sin(theta + dth/2)-ds/(2*b)*cos(theta + dth/2),
		1/b, 1/b;
		return f_u;
}

Eigen::Matrix2d Loc::Q(const double &ds, const double &dth) {
	const double k1 = 0.07;
	const double k2 = 0.07;
	const double s_l = ds - dth*b/2;
	const double s_r = ds + dth*b/2;
	Eigen::Matrix2d q;
	q <<
		k1*odom_.pose.pose.position.x, 0,
		0, k2*odom_.pose.pose.position.y;
	return q;
}

Eigen::VectorXd Loc::EstimateReferencePoint(const std::vector<Pole> &visible_poles, const Eigen::Vector3d &state) {
	Eigen::VectorXd h_x(visible_poles.size()*2);
	for (int i = 0; i < visible_poles.size(); i++) {
		const double xp = visible_poles[i].xy_coords().x;
		const double yp = visible_poles[i].xy_coords().y;
		h_x[2*i] = cos(state[2])*(xp-state[0])+sin(state[2])*(yp-state[1]);
		h_x[2*i+1] = -sin(state[2])*(xp-state[0])+cos(state[2])*(yp-state[1]);
	}
	return h_x;
}

Eigen::MatrixXd Loc::EstimateJacobi(const std::vector<Pole> &visible_poles, const Eigen::Vector3d &state) {
	Eigen::MatrixXd H(visible_poles.size()*2, 3);
	for (int i = 0; i < visible_poles.size(); i++) {
		const double xp = visible_poles[i].xy_coords().x;
		const double yp = visible_poles[i].xy_coords().y;
		H(2*i,0) = -cos(state[2]);
		H(2*i,1) = -sin(state[2]);
		H(2*i,2) = -sin(state[2])*(xp-state[0])+cos(state[2])*(yp-state[1]);
		H(2*i+1,0) = sin(state[2]);
		H(2*i+1,1) = -cos(state[2]);
		H(2*i+1,2) = -cos(state[2])*(xp-state[0])-sin(state[2])*(yp-state[1]);
	}
	return H;
}

Eigen::MatrixXd Loc::ErrorMatrix(const std::vector<Pole> &visible_poles) {
	Eigen::MatrixXd R = Eigen::MatrixXd::Zero(visible_poles.size()*2,visible_poles.size()*2);
	for (int i = 0; i < visible_poles.size(); i++) {
		R(2*i,2*i) = scan_covariance_;
		R(2*i+1,2*i+1) = scan_covariance_;
	}
	return R;
}

Eigen::VectorXd Loc::CalculateMeasuredPoints(const std::vector<Pole> &visible_poles) {
	Eigen::VectorXd z(2*visible_poles.size());
	for (int i = 0; i < visible_poles.size(); i++) {
		const double distance = visible_poles[i].laser_coords().distance;
		const double angle = visible_poles[i].laser_coords().angle;
		z[2*i] = cos(angle)*distance;
		z[2*i+1] = sin(angle)*distance;
	}
	return z;
}
