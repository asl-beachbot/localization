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
	if (odom_.timestamp != 0 && last_odom_.timestamp != -0 && use_odometry_) {
		const double delta_t_pose = (current_time_ - pose_.header.stamp).toSec();
		const double delta_t_imu = (attitude_.header.stamp - last_attitude_.header.stamp).toSec();
		const double delta_t_old = (odom_.timestamp - last_odom_.timestamp)/1000.0;
		const double time_scale_imu = delta_t_pose/delta_t_imu;
		const double time_scale_pose = delta_t_pose/delta_t_old;
		const double delta_s = (odom_.deltaUmLeft + odom_.deltaUmRight)/2/1000000.0;
		//ROS_INFO("delta_s_odom %fm", delta_s);
		const double last_theta = tf::getYaw(last_attitude_.orientation);
		const double this_theta = tf::getYaw(attitude_.orientation);
		double delta_theta = (this_theta - last_theta);
		NormalizeAngle(delta_theta);	//prevent angle difference error when going from -pi to pi
		//ROS_INFO("v_theta: %f", delta_theta/(current_time_ - pose_.header.stamp).toSec());
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
		f_u(0,0) = cos(state[2])*time_scale_pose; f_u(0,1) = -0.5*sin(state[2])*delta_s*time_scale_pose;
		f_u(1,0) = sin(state[2])*time_scale_pose; f_u(1,1) = 0.5*cos(state[2])*delta_s*time_scale_pose;
		f_u(2,0) = 0; f_u(2,1) = time_scale_imu;
		Eigen::Matrix2d q_t;
		q_t(0,0) = std::abs(delta_s)*time_scale_pose*k_s_; q_t(0,1) = 0;
		q_t(1,0) = 0; q_t(1,1) = std::abs(delta_theta)*time_scale_imu*k_th_;
		covariance = f_x*covariance*f_x.transpose() + f_u*q_t*f_u.transpose();
		//ROS_INFO("action cov [%f %f] %f", covariance(0,0), covariance(1,1), covariance(2,2));
		state[2] += delta_theta/2*time_scale_imu;	//second leap frog step later because cov uses intermediate angle
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
		//ROS_INFO("delta_s %f", delta_s);
		//ROS_INFO("timescale imu %f pose %f", time_scale_imu, time_scale_pose);
		double delta_theta = (this_theta - last_theta);
		NormalizeAngle(delta_theta);	//prevent angle difference error when going from -pi to pi
		//ROS_INFO("v_theta: %f", delta_theta/(attitude_.header.stamp - last_attitude_.header.stamp).toSec());
		//state update
		state[2] += delta_theta/2*time_scale_imu;	//use leapfrog to find x,y
		//covariance update
		Eigen::Matrix3d f_x;
		f_x << 
			1, 0, -sin(state[2])*delta_s*time_scale_pose,
			0, 1, cos(state[2])*delta_s*time_scale_pose,
			0, 0, 1;
		Eigen::MatrixXd f_u(3,2);
		f_u(0,0) = cos(state[2])*time_scale_pose; f_u(0,1) = -0.5*sin(state[2])*delta_s*time_scale_pose;
		f_u(1,0) = sin(state[2])*time_scale_pose; f_u(1,1) = 0.5*cos(state[2])*delta_s*time_scale_pose;
		f_u(2,0) = 0; f_u(2,1) = time_scale_imu;
		Eigen::Matrix2d q_t;
		q_t(0,0) = std::abs(delta_s)*time_scale_pose*k_s_; q_t(0,1) = 0;
		q_t(1,0) = 0; q_t(1,1) = std::abs(delta_theta)*time_scale_imu*k_th_;
		covariance = f_x*covariance*f_x.transpose();
		//ROS_INFO("action cov interm [%f %f] %f", covariance(0,0), covariance(1,1), covariance(2,2));
		covariance += f_u*q_t*f_u.transpose();
		//ROS_INFO("action cov end [%f %f] %f", covariance(0,0), covariance(1,1), covariance(2,2));
		state[2] += delta_theta/2*time_scale_imu;	//second leap frog step later because cov uses intermediate angle
		//ROS_INFO("No odom but laser");
	}
	else {	//no laser
		//ROS_INFO("No odom no laser");
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
		Eigen::MatrixXd R = ErrorMatrix(visible_poles, state);
		Eigen::VectorXd z = CalculateMeasuredPoints(visible_poles);
		Eigen::MatrixXd Sigma = H*covariance*H.transpose()+R;
		Eigen::MatrixXd K = covariance*H.transpose()*Sigma.inverse();	//!!!inverse bad?!
		Eigen::VectorXd nu = z-h_x;
		state += K*nu;	//update state with measurement
		covariance -= K*Sigma*K.transpose();	//update covariance with measurement
	}
	//ROS_INFO("update cov [%f %f] %f", covariance(0,0), covariance(1,1), covariance(2,2));
	
	//write vector and matrix back to ros message
	last_pose_ = pose_;
	pose_.header.stamp = current_time_;
	pose_.pose.pose.position.x = state[0];
	pose_.pose.pose.position.y = state[1];
	pose_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(state[2]);
	pose_.pose.covariance[0] = covariance(0,0);
	pose_.pose.covariance[7] = covariance(1,1);
	pose_.pose.covariance[35] = covariance(2,2);
	ROS_INFO("pose [%f %f] %f rad", state[0], state[1], state[2]);
	//reset laser
	scan_.intensities.clear();
	scan_.ranges.clear();
	last_attitude_ = attitude_;
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

/*Eigen::Matrix2d Loc::Q(const double &ds, const double &dth) {
	const double k1 = 0.07;
	const double k2 = 0.07;
	const double s_l = ds - dth*b/2;
	const double s_r = ds + dth*b/2;
	Eigen::Matrix2d q;
	q <<
		k1*odom_.pose.pose.position.x, 0,
		0, k2*odom_.pose.pose.position.y;
	return q;
}*/

Eigen::VectorXd Loc::EstimateReferencePoint(const std::vector<Pole> &visible_poles, const Eigen::Vector3d &state) {
	Eigen::VectorXd h_x(visible_poles.size()*2);
	for (int i = 0; i < visible_poles.size(); i++) {
		const double xp = visible_poles[i].line().p.x();
		const double yp = visible_poles[i].line().p.y();
		h_x[2*i] = cos(state[2])*(xp-state[0])+sin(state[2])*(yp-state[1]);
		h_x[2*i+1] = -sin(state[2])*(xp-state[0])+cos(state[2])*(yp-state[1]);
	}
	return h_x;
}

Eigen::MatrixXd Loc::EstimateJacobi(const std::vector<Pole> &visible_poles, const Eigen::Vector3d &state) {
	Eigen::MatrixXd H(visible_poles.size()*2, 3);
	for (int i = 0; i < visible_poles.size(); i++) {
		const double xp = visible_poles[i].line().p.x();
		const double yp = visible_poles[i].line().p.y();
		H(2*i,0) = -cos(state[2]);
		H(2*i,1) = -sin(state[2]);
		H(2*i,2) = -sin(state[2])*(xp-state[0])+cos(state[2])*(yp-state[1]);
		H(2*i+1,0) = sin(state[2]);
		H(2*i+1,1) = -cos(state[2]);
		H(2*i+1,2) = -cos(state[2])*(xp-state[0])-sin(state[2])*(yp-state[1]);
	}
	return H;
}

Eigen::MatrixXd Loc::ErrorMatrix(const std::vector<Pole> &visible_poles, const Eigen::Vector3d &state) {
	Eigen::MatrixXd R = Eigen::MatrixXd::Zero(visible_poles.size()*2,visible_poles.size()*2);
	for (int i = 0; i < visible_poles.size(); i++) {
		const double xp = visible_poles[i].line().p.x();
		const double yp = visible_poles[i].line().p.y();
		const double vis_angle = atan2(yp - state[2], xp - state[1]);
		R(2*i,2*i) = scan_covariance_ * cos(vis_angle) * cos(vis_angle);
		R(2*i+1,2*i+1) = scan_covariance_ * sin(vis_angle) * sin(vis_angle);
		//TODO: maybe add variance due to limited angular resolution. Might be fine without due to averaging
	}
	return R;
}

Eigen::VectorXd Loc::CalculateMeasuredPoints(const std::vector<Pole> &visible_poles) {
	Eigen::VectorXd z(2*visible_poles.size());
	for (int i = 0; i < visible_poles.size(); i++) {
		z[2*i] = visible_poles[i].laser_coords().x();
		z[2*i+1] = visible_poles[i].laser_coords().y();
	}
	return z;
}
