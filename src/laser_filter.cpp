#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include "tf/transform_listener.h"
#include "filters/filter_chain.h"
#include <pluginlib/class_loader.h>

class GenericLaserScanFilterNode
{
protected:
  // Our NodeHandle
  ros::NodeHandle nh_;

  // Components for tf::MessageFilter
  tf::TransformListener tf_;
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> tf_filter_;

  // Filter Chain
  filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;

  // Components for publishing
  sensor_msgs::LaserScan msg_;
  ros::Publisher output_pub_;

public:
  // Constructor
  GenericLaserScanFilterNode() :
      scan_sub_(nh_, "/scan", 50),
      tf_filter_(scan_sub_, tf_, "laser_frame", 50),
      filter_chain_("sensor_msgs::LaserScan")
  {
    // Configure filter chain
    filter_chain_.configure("laser_filter/test");

    // Setup tf::MessageFilter for input
    tf_filter_.registerCallback(
        boost::bind(&GenericLaserScanFilterNode::callback, this, _1));
    tf_filter_.setTolerance(ros::Duration(0.03));

    // Advertise output
    output_pub_ = nh_.advertise<sensor_msgs::LaserScan>("output", 1000);
  }

  // Callback
  void callback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
  {
    //ROS_INFO("damn gurl");
    // Run the filter chain
    filter_chain_.update (*msg_in, msg_);

    // Publish the output
    output_pub_.publish(msg_);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_filter");

  GenericLaserScanFilterNode t;
  ros::spin();

  return 0;
}