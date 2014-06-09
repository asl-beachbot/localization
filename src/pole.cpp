#include "localization/pole.h"

Pole::Pole(const Line &line, const Eigen::Vector3d &laser_coords, const ros::Time &t, const unsigned int &i) {
	line_ = line;
	laser_coords_ = laser_coords;
	time_ = t;
	i_ = i;
	visible_ = true;
}

void Pole::update(const Eigen::Vector3d &laser_coords, const ros::Time &t) {
	time_ = t;
	laser_coords_ = laser_coords;
	visible_ = true;
}

void Pole::update(const Eigen::Vector3d &laser_coords) {
	laser_coords_ = laser_coords;
}

void Pole::disappear() {
	visible_ = false;
}

Eigen::Vector3d Pole::laser_coords() const {
	return laser_coords_;
}

ros::Time Pole::time() const {
	return time_;
}

unsigned int Pole::i() const {
	return i_;
}

bool Pole::visible() const {
	return visible_;
}

Pole::Line Pole::line() const {
	return line_;
}