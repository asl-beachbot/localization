#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "visualization_msgs/Marker.h"
#include <cassert>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <numeric>

class FindPoles {
 public:

 	void CalcPoles() {
    //ROS_INFO("FindPoleClouds");
    FindPoleClouds();
    //ROS_INFO("FilterPoleClouds");
    FilterPoleClouds();
    //ROS_INFO("FitLines");
    FitLines();
    //ROS_INFO("PublishLines");
    GetDiameter();
    PublishLines();
    //ROS_INFO("done");
 	}

  std::vector<Pole::Line> GetPoles() {
    return lines_;
  }

 	FindPoles(const sensor_msgs::PointCloud &cloud) {
    cloud_ = cloud;
 		pub_ = n_.advertise<visualization_msgs::Marker>("/lines", 10, true);
 	}

 private:
 	sensor_msgs::PointCloud cloud_;
 	std::vector<sensor_msgs::PointCloud> pole_clouds_;
 	std::vector<Pole::Line> lines_;
 	ros::NodeHandle n_;
  ros::Publisher pub_;

 	void FindPoleClouds() {
 		for (int i = 0; i < cloud_.points.size(); i++) {
 			const geometry_msgs::Point32 temp_point = cloud_.points[i];
 			bool has_pole = false;
 			for (int j = 0; j < pole_clouds_.size(); j++) {	//find pole for current point
 				const double px = pole_clouds_[j].points[0].x;
 				const double py = pole_clouds_[j].points[0].y;
 				const double dist = (px-temp_point.x)*(px-temp_point.x) + (py-temp_point.y)*(py-temp_point.y);
 				if(dist < 0.25) {
 					has_pole = true;
 					pole_clouds_[j].points.push_back(temp_point);
 				}
 			}
 			if (!has_pole) {	//make new pole if point doesnt fit
 				sensor_msgs::PointCloud temp_cloud;
 				temp_cloud.header = cloud_.header;
 				temp_cloud.points.push_back(temp_point);
 				pole_clouds_.push_back(temp_cloud);
 			}
 		}
 	}

 	void FilterPoleClouds() {
 		int count = 0;
 		int n = pole_clouds_.size();
 		for(int i = 0; i < n; i++) {
 			count += pole_clouds_[i].points.size();
 		}
 		count /= n;
 		ROS_INFO("Average %d points", count);
 		for (int i = 0; i < pole_clouds_.size(); i++) {
 			if ((double)pole_clouds_[i].points.size()/count < 0.1) {
 				ROS_INFO("Discarded pole %d with %lu points", i, pole_clouds_[i].points.size());
 				pole_clouds_.erase(pole_clouds_.begin()+i);
 				i--;
 			}
 			else ROS_INFO("Kept pole %d with %lu points", i, pole_clouds_[i].points.size());
 		}
 	}

 	void FitLines() {
 		for (int i = 0; i < pole_clouds_.size(); i++) {	//Fit for every pole
 			const int cloud_size = pole_clouds_[i].points.size();
 			Eigen::Vector3d mean(0,0,0);
 			double min_z = 2000;
 			double max_z = -2000;
 			for (int j = 0; j < cloud_size; j++) {	//find average of points
 				const double x = pole_clouds_[i].points[j].x;
 				const double y = pole_clouds_[i].points[j].y;
 				const double z = pole_clouds_[i].points[j].z;
 				Eigen::Vector3d temp(x,y,z);
 				mean += temp;
 				if (z < min_z) min_z = z;
 				if (z > max_z) max_z = z;
 			}
 			mean /= cloud_size;
 			//ROS_INFO("mean point for pole %d \t[%f %f %f]", i, mean.x(), mean.y(), mean.z());
 			Pole::Line temp_line;
 			temp_line.p = mean;
 			lines_.push_back(temp_line);	//add mean point as base point for line
			Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero() ;
 			for (int j = 0; j < cloud_size; j++) {	//de-mean points
 				const double x = pole_clouds_[i].points[j].x - mean.x();
 				const double y = pole_clouds_[i].points[j].y - mean.y();
 				const double z = pole_clouds_[i].points[j].z - mean.z();
 				Eigen::Vector3d temp(x,y,z);
				matrix += 1.0/cloud_size*temp*temp.transpose();
			}
 			Eigen::EigenSolver<Eigen::Matrix3d> solver(matrix);
 			Eigen::Vector3d eigenvalues = solver.eigenvalues().real();	//get eigenvalues
 			Eigen::Matrix3d eigenvectors = solver.eigenvectors().real();	//get eigenvectors
 			double biggest_index = 0;
 			for (int j = 1; j < eigenvalues.size(); j++) {	//find biggest eigenvalue
 				if (eigenvalues(j) > eigenvalues(biggest_index)) biggest_index = j;
 			}
 			Eigen::Vector3d u = eigenvectors.col(biggest_index);
 			if (u.z() < 0) u *= -1;	//make direction point up
 			//ROS_INFO("direction for pole %d \t[%f %f %f]", i, u.x(), u.y(), u.z());
 			lines_.back().u = u;
 			//find real base of pole
 			const double scale_min = (min_z - lines_[i].p.z())/lines_[i].u.z();
 			const Eigen::Vector3d new_base = lines_[i].p + scale_min*lines_[i].u;
 			//ROS_INFO("New base for pole %d \t[%f %f %f]", i, new_base.x(), new_base.y(), new_base.z());
 			lines_.back().p = new_base;
 			//find top end of pole
 			const double scale_max = (max_z - lines_[i].p.z())/lines_[i].u.z();
 			const Eigen::Vector3d new_end = lines_[i].p + scale_max*lines_[i].u;
 			//ROS_INFO("New end for pole %d \t[%f %f %f]", i, new_end.x(), new_end.y(), new_end.z());
 			lines_.back().end = new_end;
 		}
 	}

 	static bool CompFunc(const geometry_msgs::Point32 &i, const geometry_msgs::Point32 &j) {
 		return i.z < j.z;
 	}

 	void GetDiameter() {
 		for (int i = 0; i < pole_clouds_.size(); i++) {
 			//sort poitncloud by ascending z-values
 			std::sort(pole_clouds_[i].points.begin(), pole_clouds_[i].points.end(), FindPoles::CompFunc);
 			int j = 0;
 			std::vector<double> diameters;
 			while (j < pole_clouds_[i].points.size() && ros::ok()) {	//iterate through all points
 				std::vector<Eigen::Vector3d> points;
 				int temp = j;
 				double max_pos = -2000;
 				double max_neg = 2000;
 				for (j = temp; j < (temp + 100) && j < pole_clouds_[i].points.size(); j++) {	//look at 100 points at once to build dataset
 					const double x = pole_clouds_[i].points[j].x;
 					const double y = pole_clouds_[i].points[j].y;
 					const double z = pole_clouds_[i].points[j].z;
 					Eigen::Vector3d temp_point(x,y,z);
 					//project point on normal plane of pole direction
 					temp_point -= lines_[i].p;	//point realtive to base point
 					const double dist = temp_point.dot(lines_[i].u);
 					temp_point -= dist*lines_[i].u;
 					//rotate point to pole cs
 					const double theta = atan2(lines_[i].p.y(), lines_[i].p.x());
 					Eigen::Matrix3d transform;
 					transform = Eigen::AngleAxisd(-theta, Eigen::Vector3d::UnitZ());
 					temp_point = transform*temp_point;
 					points.push_back(temp_point);
 					//ROS_INFO("temp_point [%f %f %f]", temp_point.x(), temp_point.y(), temp_point.z());
 					if (temp_point.y() > max_pos) max_pos = temp_point.y();
 					if (temp_point.y() < max_neg) max_neg = temp_point.y();
 				}
 				diameters.push_back(std::abs(max_pos - max_neg));
 			}
 			lines_[i].d = std::accumulate(diameters.begin(), diameters.end(), 0.0)/diameters.size();
 			ROS_INFO("diameter of pole %d \t%f",i ,lines_[i].d);
 		}
 	}

 	void PublishLines() {
 		visualization_msgs::Marker points, line_list;
 		points.header = line_list.header = cloud_.header;
 		points.ns = line_list.ns = "points_and_lines";
 		points.action = line_list.action = visualization_msgs::Marker::ADD;
 		points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
 		points.id = 0;
    line_list.id = 2;
    points.type = visualization_msgs::Marker::POINTS;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    points.scale.x = lines_[0].d;
    points.scale.y = lines_[0].d;
    line_list.scale.x = lines_[0].d;	//TODO: make each pole different or use mean
    points.color.g = 1.0f;
    points.color.a = 1.0;
    line_list.color.b = 1.0;
    line_list.color.a = 1.0;
    for (int i = 0; i < lines_.size(); i++) {
    	geometry_msgs::Point point, start, end;
    	point.x = lines_[i].p.x(); point.y = lines_[i].p.y(); point.z = lines_[i].p.z();
    	start.x = lines_[i].p.x(); start.y = lines_[i].p.y(); start.z = lines_[i].p.z();
    	end.x = lines_[i].end.x(); end.y = lines_[i].end.y(); end.z = lines_[i].end.z();
    	points.points.push_back(point);
    	line_list.points.push_back(start);
    	line_list.points.push_back(end);
    }
    pub_.publish(points);
    pub_.publish(line_list);
 	}
};
