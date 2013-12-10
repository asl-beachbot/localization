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
	if (odom_.pose.pose.position.x != -2000.0 && last_odom_.pose.pose.position.x != -2000.0) {
		if (using_pioneer_) {
			const double init_theta = tf::getYaw(initial_pose_.orientation);
			const double predicted_theta = tf::getYaw(odom_.pose.pose.orientation);
			ROS_INFO("predicted_theta %f", predicted_theta);
			const double x_delta_robot_cs = odom_.pose.pose.position.x - last_odom_.pose.pose.position.x;
			ROS_INFO("init_theta %f", init_theta);
			ROS_INFO("x: %f", x_delta_robot_cs);
			const double y_delta_robot_cs = odom_.pose.pose.position.y - last_odom_.pose.pose.position.y;
			ROS_INFO("y: %f", y_delta_robot_cs);
			const double theta_delta_robot_cs = predicted_theta - tf::getYaw(last_odom_.pose.pose.orientation);
			state[0] += (x_delta_robot_cs * cos(init_theta) - y_delta_robot_cs * sin(init_theta));
			state[1] += (x_delta_robot_cs * sin(init_theta) + y_delta_robot_cs * cos(init_theta));
			state[2] += theta_delta_robot_cs;
			covariance << 
				odom_.pose.covariance[0], 0, 0,
				0, odom_.pose.covariance[7], 0, 
				0, 0, odom_.pose.covariance[35];
		}
		else {	
			state += PredictPositionDelta();	//position delta resulting from odometry prediction
			Eigen::Matrix3d f_x = StateJacobi();
			covariance = f_x*covariance*f_x.transpose();
			Eigen::MatrixXd f_u = InputJacobi();
			covariance += f_u*Q()*f_u.transpose();
		}
		ROS_INFO("predicted: [%f %f] %f", state[0], state[1], state[2]);
	}
	else {
		covariance <<
			9999999, 0, 0,
			0, 9999999, 0, 
			0, 0, 9999999;
		ROS_INFO("No Odometry data");
	}
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
	pose_.pose.pose.position.x = state[0];
	pose_.pose.pose.position.y = state[1];
	pose_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(state[2]);
	pose_.pose.covariance[0] = covariance(0,0);
	pose_.pose.covariance[7] = covariance(1,1);
	pose_.pose.covariance[35] = covariance(2,2);
	//reset odometry
	if (odom_.pose.pose.position.x != -2000.0) last_odom_ = odom_;
	odom_.pose.pose.position.x = -2000.0;
	//reset laser
	scan_.intensities.clear();
}

Eigen::Vector3d Loc::PredictPositionDelta() {
	Eigen::Vector3d delta_predict;
	const double ds = (odom_.pose.pose.position.x+odom_.pose.pose.position.y)/2;
	const double dth = (odom_.pose.pose.position.x-odom_.pose.pose.position.y)/b;
	const double theta = tf::getYaw(pose_.pose.pose.orientation);
	delta_predict[0] = ds*cos(theta + dth/2);
	delta_predict[1] = ds*sin(theta + dth/2);
	delta_predict[2] = dth;
	return delta_predict;
}

Eigen::Matrix3d Loc::StateJacobi() {
	const double ds = (odom_.pose.pose.position.x+odom_.pose.pose.position.y)/2;
	const double dth = (odom_.pose.pose.position.x-odom_.pose.pose.position.y)/b;
	const double theta = tf::getYaw(pose_.pose.pose.orientation);
	Eigen::Matrix3d f_x;
	f_x << 
		1, 0, -ds*sin(theta+dth/2),
		0, 1, ds*cos(theta + dth/2),
		0, 0, 1;
	return f_x;
}

Eigen::MatrixXd Loc::InputJacobi() {
	const double ds = (odom_.pose.pose.position.x+odom_.pose.pose.position.y)/2;
	const double dth = (odom_.pose.pose.position.x-odom_.pose.pose.position.y)/b;
	const double theta = tf::getYaw(pose_.pose.pose.orientation);
	Eigen::MatrixXd f_u(3,2);
	f_u << 
		0.5*cos(theta + dth/2)-ds/(2*b)*sin(theta + dth/2), 0.5*cos(theta + dth/2)+ds/(2*b)*sin(theta + dth/2),
		0.5*sin(theta + dth/2)+ds/(2*b)*cos(theta + dth/2), 0.5*sin(theta + dth/2)-ds/(2*b)*cos(theta + dth/2),
		1/b, 1/b;
		return f_u;
}

Eigen::Matrix2d Loc::Q() {
	const double k1 = 0.1;
	const double k2 = 0.1;
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
		R(2*i,2*i) = 0.012*0.012;
		R(2*i+1,2*i+1) = 0.012*0.012;
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