#include "pole.h"

Pole::Pole(const localization::xy_point &xy_coords, const localization::scan_point &laser_coords, const ros::Time &t, const unsigned int &i) {
	xy_coords_ = xy_coords;
	laser_coords_ = laser_coords;
	time_ = t;
	i_ = i;
	visible_ = true;
}

Pole::Pole(const Line &line, const localization::scan_point &laser_coords, const ros::Time &t, const unsigned int &i) {
	xy_coords_.x = line.p.x();
	xy_coords_.y = line.p.y();
	line_ = line;
	laser_coords_ = laser_coords;
	time_ = t;
	i_ = i;
	visible_ = true;
}

void Pole::update(const localization::scan_point &laser_coords, const ros::Time &t) {
	time_ = t;
	laser_coords_ = laser_coords;
	visible_ = true;
}

void Pole::update(const localization::scan_point &laser_coords) {
	laser_coords_ = laser_coords;
}

void Pole::disappear() {
	visible_ = false;
}

localization::xy_point Pole::xy_coords() const {
	return 	xy_coords_;
}
	
localization::scan_point Pole::laser_coords() const {
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