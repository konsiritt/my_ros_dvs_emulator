// This file is part of my_ros_dvs_emulator - a DVS emulator implementation
// Copyright (C) 2017 Konstantin Ritt
//
// my_ros_dvs_emulator is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// my_ros_dvs_emulator is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with my_ros_dvs_emulator.  If not, see <http://www.gnu.org/licenses/>.

#ifndef BENCH_EMUL_H
#define BENCH_EMUL_H

#include <ros/ros.h>
#include <string>

// boost
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// messages
#include <ros_dvs_msgs/Event.h>
#include <ros_dvs_msgs/EventArray.h>
#include <std_msgs/Empty.h>

// dvs configuration file
#include "ros_dvs_benchmark/config_dvs.h"

// service file
#include "ros_dvs_service/GetTime.h"


namespace ros_dvs_benchmark {

class BenchEmul {
public:
    BenchEmul(ros::NodeHandle & nh, ros::NodeHandle nh_private);
    ~BenchEmul();

private:

    void runBenchmark ();

    void publishPacket(unsigned packetSize);

    uint64_t countPackages;

    ros::NodeHandle nh_;
    ros::Publisher event_array_pub_;

    volatile bool running_;

    boost::shared_ptr<boost::thread> readout_thread_;

    unsigned iterations;

    unsigned pubFreq;

};

} // namespace


#endif // BENCH_EMUL_H

