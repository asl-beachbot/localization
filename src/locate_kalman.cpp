#include "locate.h"

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

	//predict
	state += PredictPositionDelta();	//position delta resulting from odometry prediction
	Eigen::Matrix3d f_x = StateJacobi();
	covariance = f_x*covariance*f_x.transpose();
	Eigen::MatrixXd f_u = InputJacobi();
	covariance += f_u*Q()*f_u.transpose();
	
	//write vector and matrix back to ros message
	pose_.pose.pose.position.x = state[0];
	pose_.pose.pose.position.y = state[1];
	pose_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(state[2]);
	pose_.pose.covariance[0] = covariance(0,0);
	pose_.pose.covariance[7] = covariance(1,1);
	pose_.pose.covariance[35] = covariance(2,2);
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
	Eigen::Matrix3d jacobi;
	jacobi << 
		1, 0, -ds*sin(theta+dth/2),
		0, 1, ds*cos(theta + dth/2),
		0, 0, 1;
	return jacobi;
}

Eigen::MatrixXd Loc::InputJacobi() {
	const double ds = (odom_.pose.pose.position.x+odom_.pose.pose.position.y)/2;
	const double dth = (odom_.pose.pose.position.x-odom_.pose.pose.position.y)/b;
	const double theta = tf::getYaw(pose_.pose.pose.orientation);
	Eigen::MatrixXd jacobi(3,2);
	jacobi << 
		0.5*cos(theta + dth/2)-ds/(2*b)*sin(theta + dth/2), 0.5*cos(theta + dth/2)+ds/(2*b)*sin(theta + dth/2),
		0.5*sin(theta + dth/2)+ds/(2*b)*cos(theta + dth/2), 0.5*sin(theta + dth/2)-ds/(2*b)*cos(theta + dth/2),
		1/b, 1/b;
		return jacobi;
}

Eigen::Matrix2d Loc::Q() {
	const double k1 = 0.01;
	const double k2 = 0.01;
	Eigen::Matrix2d q;
	q <<
		k1*odom_.pose.pose.position.x, 0,
		0, k2*odom_.pose.pose.position.y;
}