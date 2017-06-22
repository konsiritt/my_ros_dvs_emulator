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
#include <std_msgs/Float32.h>

#include <iostream>

namespace ros_dvs_benchmark {

BenchSubscr::BenchSubscr(ros::NodeHandle & nh, ros::NodeHandle nh_private) :
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
    event_sub_ = nh_.subscribe("events", 1, &BenchSubscr::eventsCallback, this,
                               ros::TransportHints().reliable().tcpNoDelay(true)); //.unreliable()
    // unreliable(): UDP
    // reliable().tcpNoDelay(true): TCP without Nagle's algorithm

    initPlotting();
}

BenchSubscr::~BenchSubscr()
{
    closePlotting();
}

bool BenchSubscr::reset_measurements(ros_dvs_service::GetTime::Request &req,
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

bool BenchSubscr::get_duration(ros_dvs_service::GetTime::Request &req,
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

bool BenchSubscr::get_frequency (ros_dvs_service::GetTime::Request &req,
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

bool BenchSubscr::log_results(ros_dvs_service::GetTime::Request &req,
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

void BenchSubscr::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
    timestampNow.push_back(ros::Time::now());
    timestampSent.push_back(msg->events.back().ts);
    sizeSent.push_back(msg->events.size());
}

#ifndef NO_IO

int BenchSubscr::initPlotting()
{
    if( !resultsLog.is_open() )
    {
        if (!resultsLog)
        {
            std::cerr << "something went wrong when opening the logging file" << std::endl;
        }
        std::cout << "logging results to " << (outputDir +"/benchmark.out").c_str() << std::endl;
        resultsLog.open((outputDir +"/benchmark.out").c_str(),std::fstream::out); //std::ios::app);
        resultsLog << "# avg freq [Hz] | sigma freq | avg duration [ms] | sigma duration "
                   <<"| avg packet size | sigma packet size | packet iterations" << std::endl;
    }
    else
    {
        std::cerr <<"Could not open file!\n" << std::endl;
        return -1;
    }
    return 1;
}

int BenchSubscr::plot()
{
    resultsLog << meanFrequency << " " << stdDevFrequency << " " << meanDuration << " "
               << stdDevDuration << " " << meanPacketSize << " " << stdDevPacketSize << " "
               << packetIterations << std::endl;
}

int BenchSubscr::closePlotting()
{
    resultsLog.close();
    return 1;
}

#endif // NO_IO

} // namespace
