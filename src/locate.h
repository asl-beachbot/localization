#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "bbcontrol/State.h"
#include "nav_msgs/Odometry.h"
#include "localization/beach_map.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
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
	ros::Subscriber state_sub_;
	ros::Publisher pub_pose_;
	ros::Publisher pub_pole_;
	ros::Publisher pub_map_;

	double b;	//wheel distance of robot
	double pole_radius;	//radius of reflective poles
	bool using_pioneer_;	//if using pioneer for testing
	sensor_msgs::LaserScan scan_;
	nav_msgs::Odometry odom_;
	nav_msgs::Odometry last_odom_;
	nav_msgs::Odometry initial_odom_;
	std::vector<Pole> poles_;
	geometry_msgs::PoseWithCovarianceStamped pose_;
	geometry_msgs::PoseStamped initial_pose_;
	bool initiation_;
	ros::Time current_time_;

	void NormalizeAngle(double& angle);
	void StateHandler();
	void InitiatePoles();
	void PublishPoles();
	void PublishPose();
	void PublishMap();
	void PublishTf();
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
	void StateCallback(const bbcontrol::State &new_state);
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
