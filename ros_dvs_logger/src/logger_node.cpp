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

#include "ros_dvs_logger/logger.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ros_dvs_logger");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");


    ros_dvs_logger::EventLogger eventLogger(nh, nh_private);

    // setup services
    ros::ServiceServer serviceReset = nh.advertiseService("reset_measurements", &ros_dvs_logger::EventLogger::reset_measurements, &eventLogger);
    ros::ServiceServer serviceDuration = nh.advertiseService("get_duration", &ros_dvs_logger::EventLogger::get_duration, &eventLogger);
    ros::ServiceServer serviceFrequency = nh.advertiseService("get_frequency", &ros_dvs_logger::EventLogger::get_frequency, &eventLogger);
    ros::ServiceServer serviceLog = nh.advertiseService("log_results", &ros_dvs_logger::EventLogger::log_results, &eventLogger);

    ros::spin();

    return 0;
}
