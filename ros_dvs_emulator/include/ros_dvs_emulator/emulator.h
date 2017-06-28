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

#pragma once

#include <ros/ros.h>
#include <string>

// boost
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
// boost shared mem
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>

// messages
#include <ros_dvs_msgs/Event.h>
#include <ros_dvs_msgs/EventArray.h>
#include <std_msgs/Empty.h>

// execution time measurement
#include "ros_dvs_emulator/Timer.h"

// dvs configuration file
#include "ros_dvs_emulator/config_dvs.h"

// plot/log data
#include <fstream>      // std::ofstream

struct shared_mem_emul
{
    shared_mem_emul() :
        timeNew(0),
        timeRef(0),
        imageNew(),
        imageRef(),
        frameUpdated(false),
        mutex()
    {
    }
    double timeNew;
    double timeRef;
    unsigned char imageNew[image_width*image_height*4];
    double imageRef[image_width*image_height];

    //boolean updated when new frame was written
    bool frameUpdated;

    //Mutex to protect access to the queue
    boost::interprocess::interprocess_mutex mutex;

    //Condition to wait when the frame was not updated
    boost::interprocess::interprocess_condition  condNew;
};

namespace ros_dvs_emulator {

class RosDvsEmulator {
public:
    RosDvsEmulator(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  RosDvsEmulator(ros::NodeHandle & nh, ros::NodeHandle nh_private, shared_mem_emul * dataShrd_);
  ~RosDvsEmulator();

  inline double linlog(double arg)
  {
      if (arg>linLogLim)
      {
          return log(arg);
      }
      else
          return log(linLogLim)/linLogLim*arg;
  }

  double linlog(uint16_t arg);


private:

  void readout();

  uint64_t countPackages;

  ros::NodeHandle nh_;
  ros::Publisher event_array_pub_;

  volatile bool running_;

  boost::shared_ptr<boost::thread> readout_thread_;

  unsigned dvsThresh;

  unsigned streamingRate;

  shared_mem_emul *dataShrd;

  Timer t1;
  double tMutex;
  double tWait;
  double tProcess;
  double tPublish;
  unsigned framesCount;
  unsigned eventCount;

  double linLogLim;

  double lookupLinLog[256];

  //****************************************************************
  ///! Plotting parameters
  //****************************************************************
  void logLookupTable ();
  //! directory for data logging
  std::string outputDir;
  //! output stream object for benchmark results logging
  std::ofstream linLogPlot;

};

} // namespace
