#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "pole.cpp"
#include <cmath>

class Loc {
 public:
	Loc();

 private:
	ros::NodeHandle n_;
	ros::Subscriber sub_scan_;
	ros::Subscriber sub_odom_;
	ros::Publisher pub_pose_;
	ros::Publisher pub_pole_;

	sensor_msgs::LaserScan scan_;
	std::vector<Pole> poles_;
	geometry_msgs::PoseWithCovarianceStamped pose_;
	bool initiation_;
	ros::Time current_time_;

	void NormalizeAngle(double& angle);
	void StateHandler();
	void InitiatePoles();
	void PublishPoles();
	void PublishPose();
	std::vector<localization::xy_point> ScanToXY(const std::vector<localization::scan_point> scan);
	void Locate();
	void UpdatePoles(const std::vector<localization::scan_point> &scans_to_sort);
	void GetPose();
	void EstimateInvisiblePoles();
	void PrintPose();
	void PrintPoleScanData();
	void calcPose(const Pole &pole1, const Pole &pole2, std::vector<geometry_msgs::Pose> *pose_vector);
	bool IsPolePoint(const double &intensity, const double &distance);
	void ExtractPoleScans(std::vector<localization::scan_point> *scan_pole_points);
	void MinimizeScans(std::vector<localization::scan_point> *scan);
	void ScanCallback(const sensor_msgs::LaserScan &scan);
	void OdomCallback(const nav_msgs::Odometry &odom);
};