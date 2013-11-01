#include <ros/ros.h>
#include <localization/scan_point.h>
#include <localization/xy_point.h>





class Pole {
 public:
	Pole();
	void update(const localization::scan_point scan_point);

 private:
	localization::xy_point xy_coords_;
	localization::scan_point laser_coords_;
	ros::Time time_;
	unsigned int i_;




 




};