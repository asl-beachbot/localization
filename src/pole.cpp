#include "pole.h"

Pole::Pole(const localization::xy_point &xy_coords, const localization::scan_point &laser_coords, const ros::Time &t, const unsigned int &i) {
	xy_coords_ = xy_coords;
	laser_coords_ = laser_coords;
	time_ = t;
	i_ = i;
}

localization::xy_point Pole::xy_coords() {
	return xy_coords_;
}
	
localization::scan_point Pole::laser_coord() {
	return laser_coords_;
}

ros::Time Pole::time() {
	return time_;
}

unsigned int Pole::i() {
	return i_;
}