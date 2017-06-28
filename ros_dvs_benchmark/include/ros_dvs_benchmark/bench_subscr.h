#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include <ros/ros.h>

#include <ros_dvs_msgs/Event.h>
#include <ros_dvs_msgs/EventArray.h>

// store measurements
#include <vector>

// log data
#include <fstream>      // std::ofstream

// service file
#include "ros_dvs_service/GetTime.h"

namespace ros_dvs_benchmark
{

class BenchSubscr {
public:
  BenchSubscr(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  virtual ~BenchSubscr();

  //****************************************************************
  ///! Service functions
  //****************************************************************
  bool reset_measurements(ros_dvs_service::GetTime::Request &req,
                          ros_dvs_service::GetTime::Response &res);

  // returns average duration of all transmitted packets in [ms]
  bool get_duration(ros_dvs_service::GetTime::Request &req,
                    ros_dvs_service::GetTime::Response &res);

  // returns average frequency of all transmitted packets in [Hz]
  bool get_frequency(ros_dvs_service::GetTime::Request &req,
                    ros_dvs_service::GetTime::Response &res);

  // logs results to a file
  bool log_results(ros_dvs_service::GetTime::Request &req,
                    ros_dvs_service::GetTime::Response &res);

private:
  //****************************************************************
  ///! ROS specific parameters
  //****************************************************************
  ros::NodeHandle nh_;

  uint64_t counterReceived;

  //!
  //! \brief eventsCallback is called whenever an event packet is received
  //! \param msg is a pointer to the received message
  //!
  void eventsCallback(const ros_dvs_msgs::EventArray::ConstPtr& msg);

  ros::Subscriber event_sub_;

  //****************************************************************
  ///! Benchmarking parameters
  //****************************************************************
  //! vectors containing timestamps for each packet received
  //! _now: time of receiving the packet
  std::vector<ros::Time> timestampNow;
  //! _sent: time of last event in packet (closest before being sent)
  std::vector<ros::Time> timestampSent;
  //! vector containing the amount of events sent per packet
  std::vector<int> sizeSent;

  //! last calculated mean frequency
  double meanFrequency;
  double stdDevFrequency;
  //! last calculated mean transmission duration
  double meanDuration;
  double stdDevDuration;
  //! last calculated mean amount of events sent per packet
  double meanPacketSize;
  double stdDevPacketSize;
  //! last amount of iterations of packets received
  double packetIterations;

  //****************************************************************
  ///! Logging parameters
  //****************************************************************
#ifndef NO_IO
  int initPlotting();
  int plot();
  int closePlotting();

  //! directory for data logging
  std::string outputDir;
  //! output stream object for benchmark results logging
  std::ofstream resultsLog;

#endif // NO_IO
};

} // namespace

#endif // SUBSCRIBER_H

