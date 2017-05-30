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

// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <std_msgs/Empty.h>

// execution time measurement
#include "ros_dvs_emulator/Timer.h"

struct shared_mem_emul
{
    shared_mem_emul() :
        aIsNew(true),
        timeA(0),
        timeB(0),
        imageA(),
        imageB(),
        mutex()
    {
    }

    bool aIsNew;
    double timeA;
    double timeB;
    unsigned char imageA[640*480*4];
    unsigned char imageB[640*480*4];

    //Mutex to protect access to the queue
    boost::interprocess::interprocess_mutex mutex;

};

namespace ros_dvs_emulator {

class RosDvsEmulator {
public:
    RosDvsEmulator(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  RosDvsEmulator(ros::NodeHandle & nh, ros::NodeHandle nh_private, shared_mem_emul * dataShrd_);
  ~RosDvsEmulator();

private:

  void readout();

  ros::NodeHandle nh_;
  ros::Publisher event_array_pub_;

  volatile bool running_;

  boost::shared_ptr<boost::thread> readout_thread_;

  unsigned dvsThresh;

  shared_mem_emul *dataShrd;

  Timer t1;
  double tProcess;
  double tPublish;
  unsigned framesCount;


};

} // namespace
