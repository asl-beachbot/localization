#include <ros/ros.h>
#include <localization/scan_point.h>
#include <localization/xy_point.h>

class Pole {
 public:
	Pole();
	Pole(const localization::xy_point &xy_coords);
	Pole(const localization::xy_point &xy_coords, const localization::scan_point &laser_coords, const ros::Time &t, const unsigned int &i);
	void update(const localization::scan_point &laser_coords);
	void update(const ros::Time &t);
	void update(const localization::scan_point &laser_coords, const ros::Time &t);
	localization::xy_point xy_coords() const;
	localization::scan_point laser_coords() const;
	ros::Time time() const;
	unsigned int i() const;

 private:
	localization::xy_point xy_coords_;		//xy coordinates of the pole
	localization::scan_point laser_coords_;		//last known laser scan data of pole
	ros::Time time_;		//time of last sighting
	unsigned int i_;		//index of pole




 




};