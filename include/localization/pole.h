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
	Pole(const Line &line);
	Pole(const Line &line, const Eigen::Vector3d &laser_coords, const ros::Time &t, const unsigned int &i);
	void update(const Eigen::Vector3d &laser_coords);
	void update(const ros::Time &t);
	void update(const Eigen::Vector3d &laser_coords, const ros::Time &t);
	void disappear();
	Eigen::Vector3d laser_coords() const;
	ros::Time time() const;
	unsigned int i() const;
	bool visible() const;
	Line line() const;

 private:
	Eigen::Vector3d laser_coords_;		//last known laser scan data of pole
	Line line_;		//3D position of pole
	ros::Time time_;		//time of last sighting
	unsigned int i_;		//index of pole
	bool visible_;
};