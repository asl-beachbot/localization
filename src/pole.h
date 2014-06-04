#include <ros/ros.h>
#include <localization/scan_point.h>
#include <localization/xy_point.h>
#include <Eigen/Dense>

class Pole {
 public:
 	struct Line {
		Eigen::Vector3d p;	//base point
		Eigen::Vector3d u;	//direction
		Eigen::Vector3d end;	//end point
		double d; 	//diameter
	};
	
	Pole();
	Pole(const localization::xy_point &xy_coords);
	Pole(const Line &line);
	Pole(const localization::xy_point &xy_coords, const localization::scan_point &laser_coords, const ros::Time &t, const unsigned int &i);
	Pole(const Line &line, const localization::scan_point &laser_coords, const ros::Time &t, const unsigned int &i);
	void update(const localization::scan_point &laser_coords);
	void update(const ros::Time &t);
	void update(const localization::scan_point &laser_coords, const ros::Time &t);
	void disappear();
	localization::xy_point xy_coords() const;
	localization::scan_point laser_coords() const;
	ros::Time time() const;
	unsigned int i() const;
	bool visible() const;

 private:
	localization::xy_point xy_coords_;		//xy coordinates of the pole
	localization::scan_point laser_coords_;		//last known laser scan data of pole
	Line line_;		//3D position of pole
	ros::Time time_;		//time of last sighting
	unsigned int i_;		//index of pole
	bool visible_;

};