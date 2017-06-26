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
#include <std_msgs/Float32.h>

#include <iostream>

namespace ros_dvs_logger {

EventLogger::EventLogger(ros::NodeHandle & nh, ros::NodeHandle nh_private) :
    nh_(nh),
    outputDir("/home/rittk/devel/catkin_torcs_ros/logs/output/ros_dvs_benchmark"), //2DO: make adaptable
    meanFrequency(0),
    stdDevFrequency(0),
    meanDuration(0),
    stdDevDuration(0),
    meanPacketSize(0),
    stdDevPacketSize(0),
    packetIterations(0)
{
    timestampNow.reserve(1000);
    timestampSent.reserve(1000);
    sizeSent.reserve(1000);

    // setup subscribers and publishers
    event_sub_ = nh_.subscribe("events", 1, &EventLogger::eventsCallback, this,
                               ros::TransportHints().reliable().tcpNoDelay(true)); //.unreliable()
    // unreliable(): UDP
    // reliable().tcpNoDelay(true): TCP without Nagle's algorithm

    initPlotting();
    initLogging();
}

EventLogger::~EventLogger()
{
    closePlotting();
}

bool EventLogger::reset_measurements(ros_dvs_service::GetTime::Request &req,
                                     ros_dvs_service::GetTime::Response &res)
{
    std::cout << "resetting measurements now with current length at " << timestampNow.size() << " elements" << std::endl;
    timestampNow.clear();
    timestampNow.reserve(1000);

    timestampSent.clear();
    timestampSent.reserve(1000);

    sizeSent.clear();
    sizeSent.reserve(1000);

    return true;
}

bool EventLogger::get_duration(ros_dvs_service::GetTime::Request &req,
                               ros_dvs_service::GetTime::Response &res)
{
    ros::Time temp;
    if (timestampNow.size() != timestampSent.size() || timestampNow.size() == 0)
        return false;

    double tempPacketSize = 0;
    packetIterations = timestampNow.size();
    for (int ii = 0; ii < timestampNow.size(); ++ii)
    {
        tempPacketSize += sizeSent[ii];
        temp += timestampNow[ii] - timestampSent[ii];
    }
    meanPacketSize = tempPacketSize/packetIterations;
    meanDuration = 1e3*temp.toSec()/packetIterations;
    res.data = meanDuration;

    std::cout << "average transmission time " << res.data << " ms" << std::endl;
    return true;
}

bool EventLogger::get_frequency (ros_dvs_service::GetTime::Request &req,
                               ros_dvs_service::GetTime::Response &res)
{
    double temp = 0;
    if (timestampNow.size() != timestampSent.size() || timestampNow.size() == 0)
        return false;

    packetIterations = timestampNow.size();
    for (int ii = 1; ii < packetIterations; ++ii)
    {
        temp += 1./(timestampNow[ii] - timestampNow[ii-1]).toSec();
    }
    meanFrequency = temp/(packetIterations-1);
    res.data = meanFrequency;

    std::cout << "average frequency " << res.data << " Hz" << std::endl;
    return true;
}

bool EventLogger::log_results(ros_dvs_service::GetTime::Request &req,
                                     ros_dvs_service::GetTime::Response &res)
{
    double tempTimeTotal = 0;
    double tempTimeDev = 0;
    double tempFreqTotal = 0;
    double tempFreqDev = 0;
    double tempPacketSizeTotal = 0;

    packetIterations = timestampNow.size();
    for (int ii = 0; ii < timestampNow.size(); ++ii)
    {
        // calculate deviation from mean per element
        if (ii != 0 )
        {
            tempFreqDev = (1./(timestampNow[ii] - timestampNow[ii-1]).toSec()) - meanFrequency;
            tempFreqTotal = tempFreqDev*tempFreqDev;
        }

        tempTimeDev = 1e3*(timestampNow[ii] - timestampSent[ii]).toSec() - meanDuration;
        tempTimeTotal += tempTimeDev*tempTimeDev;

        tempPacketSizeTotal += (sizeSent[ii] - meanPacketSize)*(sizeSent[ii] - meanPacketSize);
    }
    stdDevFrequency = sqrt(tempFreqTotal/(packetIterations-1));
    stdDevDuration = sqrt(tempTimeTotal/packetIterations);
    stdDevPacketSize = sqrt(tempPacketSizeTotal/packetIterations);

    plot();
    return true;
}

void EventLogger::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
    timestampNow.push_back(ros::Time::now());
    timestampSent.push_back(msg->events.back().ts);
    sizeSent.push_back(msg->events.size());

    for (int ii = 0; ii < msg->events.size(); ++ii)
    {

    }
}

int EventLogger::initLogging()
{
    if( !eventsLog.is_open() )
    {
        if (!eventsLog)
        {
            std::cerr << "something went wrong when opening the logging file" << std::endl;
        }
        eventsLog.open((outputDir +"/events.aedat").c_str(),std::ofstream::binary);
        eventsLog.write( (const char *) "#!AER-DAT2.0\r\n",14);
        eventsLog.write( (const char *) "# This is a raw AE data file - do not edit\r\n", 44);
        eventsLog.write( (const char *) "# Data format is int32 address, int32 timestamp (8 bytes total), repeated for each event\r\n", 90);
        eventsLog.write( (const char *) "# Timestamps tick is 1 us\r\n", 27);
//        time_t now = time(0);
//        char* currentTime = ctime(&now);
//        eventsLog.write( (const char *) "# created " + currentTime + "\r\n", 12 + strlen(currentTime));
        // First prepend the time.
        time_t currentTimeEpoch = time(NULL);
        // From localtime_r() man-page: "According to POSIX.1-2004, localtime()
        // is required to behave as though tzset(3) was called, while
        // localtime_r() does not have this requirement."
        // So we make sure to call it here, to be portable.
        tzset();

        struct tm currentTime;
        localtime_r(&currentTimeEpoch, &currentTime);

        // Following time format uses exactly 45 characters (26 separators/characters,
        // 4 year, 2 month, 2 day, 2 hours, 2 minutes, 2 seconds, 5 time-zone).
        size_t currentTimeStringLength = 45;
        char currentTimeString[currentTimeStringLength + 1]; // + 1 for terminating NUL byte.
        strftime(currentTimeString, currentTimeStringLength + 1, "# Start-Time: %Y-%m-%d %H:%M:%S (TZ%z)\r\n", &currentTime);
        eventsLog.write( (const char *) currentTimeString, currentTimeStringLength);
        eventsLog.write( (const char *) "#!END-HEADER\r\n", 14);
    }
    else
    {
        std::cerr <<"Could not open file!\n" << std::endl;
        return -1;
    }
    return 1;
}

int EventLogger::logAer()
{

}

int EventLogger::closeLogging()
{
    eventsLog.close();
    return 1;
}


#ifndef NO_IO

int EventLogger::initPlotting()
{
    if( !resultsPlot.is_open() )
    {
        if (!resultsPlot)
        {
            std::cerr << "something went wrong when opening the logging file" << std::endl;
        }
        std::cout << "logging results to " << (outputDir +"/benchmark.out").c_str() << std::endl;
        resultsPlot.open((outputDir +"/benchmark.out").c_str(),std::fstream::out); //std::ios::app);
        resultsPlot << "# avg freq [Hz] | sigma freq | avg duration [ms] | sigma duration "
                   <<"| avg packet size | sigma packet size | packet iterations" << std::endl;
    }
    else
    {
        std::cerr <<"Could not open file!\n" << std::endl;
        return -1;
    }
    return 1;
}

int EventLogger::plot()
{
    resultsPlot << meanFrequency << " " << stdDevFrequency << " " << meanDuration << " "
               << stdDevDuration << " " << meanPacketSize << " " << stdDevPacketSize << " "
               << packetIterations << std::endl;
}

int EventLogger::closePlotting()
{
    resultsPlot.close();
    return 1;
}

#endif // NO_IO

} // namespace
