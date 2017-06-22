// ros_dvs_benchmark - benchmark for dvs-message types.
// Copyright (C) 2016-2017 Konstantin Ritt
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>


#include "ros_dvs_benchmark/bench_subscr.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ros_dvs_benchmark");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");


  ros_dvs_benchmark::BenchSubscr benchSubscriber(nh, nh_private);

  // setup services
  ros::ServiceServer serviceReset = nh.advertiseService("reset_measurements", &ros_dvs_benchmark::BenchSubscr::reset_measurements, &benchSubscriber);
  ros::ServiceServer serviceDuration = nh.advertiseService("get_duration", &ros_dvs_benchmark::BenchSubscr::get_duration, &benchSubscriber);
  ros::ServiceServer serviceFrequency = nh.advertiseService("get_frequency", &ros_dvs_benchmark::BenchSubscr::get_frequency, &benchSubscriber);
  ros::ServiceServer serviceLog = nh.advertiseService("log_results", &ros_dvs_benchmark::BenchSubscr::log_results, &benchSubscriber);

  ros::spin();

  return 0;
}
