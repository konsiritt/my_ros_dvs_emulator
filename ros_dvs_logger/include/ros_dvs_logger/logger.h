// This file is part of DVS-ROS - the RPG DVS ROS Package
//
// DVS-ROS is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// DVS-ROS is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with DVS-ROS.  If not, see <http://www.gnu.org/licenses/>.

#ifndef DVS_LOGGER_H_
#define DVS_LOGGER_H_

#include <ros/ros.h>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

// store measurements
#include <vector>

// insert current time into logging file
#include <ctime>

// plot/log data
#include <fstream>      // std::ofstream
#define AEDAT3_FILE_VERSION 2.0

// perform big endian small endian conversions:
#include <arpa/inet.h>

// service file
#include "ros_dvs_service/GetTime.h"

namespace ros_dvs_logger
{

class EventLogger {
public:
  EventLogger(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  virtual ~EventLogger();

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

  //!
  //! \brief eventsCallback is called whenever an event packet is received
  //! \param msg is a pointer to the received message
  //!
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);

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
  ///! Logging parameters -> save events in AER-DAT file format
  //****************************************************************
  int initLogging();
  int logAer();
  int closeLogging();

  //! directory for data logging
  std::string outputDir;
  //! file for logging aerdat event data
  std::ofstream eventsLog;


  //****************************************************************
  ///! Plotting parameters
  //****************************************************************
#ifndef NO_IO
  int initPlotting();
  int plot();
  int closePlotting();

  //! output stream object for benchmark results logging
  std::ofstream resultsPlot;

#endif // NO_IO
};

} // namespace

#endif // DVS_LOGGER_H_
