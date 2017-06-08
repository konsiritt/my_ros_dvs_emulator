#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include <ros/ros.h>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

// store measurements
#include <vector>

// service file
#include "ros_dvs_service/GetTime.h"

namespace ros_dvs_benchmark
{

class BenchSubscr {
public:
  BenchSubscr(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  virtual ~BenchSubscr();

  bool reset_measurements(ros_dvs_service::GetTime::Request &req,
                          ros_dvs_service::GetTime::Response &res);

  // returns average duration of all transmitted packets in [ms]
  bool get_duration(ros_dvs_service::GetTime::Request &req,
                    ros_dvs_service::GetTime::Response &res);

  // returns average frequency of all transmitted packets in [Hz]
  bool get_frequency(ros_dvs_service::GetTime::Request &req,
                    ros_dvs_service::GetTime::Response &res);

private:
  ros::NodeHandle nh_;

  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);

  ros::Subscriber event_sub_;

  std::vector<ros::Time> timestamp_now;
  std::vector<ros::Time> timestamp_sent;
  std::vector<int> size_sent;
};

} // namespace

#endif // SUBSCRIBER_H

