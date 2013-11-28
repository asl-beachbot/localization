#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "pole.cpp"
#include <Eigen/Dense>
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

	static const double b = 0.5;	//wheel distance of robot
	sensor_msgs::LaserScan scan_;
	nav_msgs::Odometry odom_;
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
	void RefreshData();
	void UpdatePoles(const std::vector<localization::scan_point> &scans_to_sort);
	void GetPose();
	void EstimateInvisiblePoles();
	void PrintPose();
	void PrintPoleScanData();
	void CalcPose(const Pole &pole1, const Pole &pole2, std::vector<geometry_msgs::Pose> *pose_vector);
	bool IsPolePoint(const double &intensity, const double &distance);
	void ExtractPoleScans(std::vector<localization::scan_point> *scan_pole_points);
	void MinimizeScans(std::vector<localization::scan_point> *scan);
	void ScanCallback(const sensor_msgs::LaserScan &scan);
	void OdomCallback(const nav_msgs::Odometry &odom);
	//Kalman functions
	void DoTheKalman();
	Eigen::Vector3d PredictPositionDelta();
	Eigen::Matrix3d StateJacobi();
	Eigen::MatrixXd InputJacobi();
	Eigen::Matrix2d Q();
	Eigen::VectorXd EstimateReferencePoint(const std::vector<Pole> &visible_poles, const Eigen::Vector3d &state);
	Eigen::MatrixXd EstimateJacobi(const std::vector<Pole> &visible_poles, const Eigen::Vector3d &state);
	Eigen::MatrixXd ErrorMatrix(const std::vector<Pole> &visible_poles);
	Eigen::VectorXd CalculateMeasuredPoints(const std::vector<Pole> &visible_poles);
};