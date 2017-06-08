
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


#include <ros/ros.h>

#define image_width 320 //640
#define image_height 240 //480
#include "ros_dvs_benchmark/bench_emul.h"


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ros_dvs_benchmark");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::cout << "Start the ros_dvs_benchmark object" << std::endl;
    ros_dvs_benchmark::BenchEmul* emulator = new ros_dvs_benchmark::BenchEmul(nh, nh_private);

    ros::spin();
    return 0;
}
