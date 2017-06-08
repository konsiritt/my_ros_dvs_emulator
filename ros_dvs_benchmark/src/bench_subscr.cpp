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
    nh_(nh)
{
    timestamp_now.reserve(1000);
    timestamp_sent.reserve(1000);
    size_sent.reserve(1000);

    // setup subscribers and publishers
    event_sub_ = nh_.subscribe("events", 1, &BenchSubscr::eventsCallback, this,
                               ros::TransportHints().reliable().tcpNoDelay(true)); //.unreliable()
    // unreliable(): UDP
    // reliable().tcpNoDelay(true): TCP without Nagle's algorithm
}

BenchSubscr::~BenchSubscr()
{
}

bool BenchSubscr::reset_measurements(ros_dvs_service::GetTime::Request &req,
                                     ros_dvs_service::GetTime::Response &res)
{
    std::cout << "resetting measurements now with current length at " << timestamp_now.size() << " elements" << std::endl;
    timestamp_now.clear();
    timestamp_now.reserve(1000);

    timestamp_sent.clear();
    timestamp_sent.reserve(1000);

    size_sent.clear();
    size_sent.reserve(1000);

    return true;
}

bool BenchSubscr::get_duration(ros_dvs_service::GetTime::Request &req,
                               ros_dvs_service::GetTime::Response &res)
{
    ros::Time temp;
    if (timestamp_now.size() != timestamp_sent.size() || timestamp_now.size() == 0)
        return false;

    for (int ii = 0; ii < timestamp_now.size(); ++ii)
    {
        temp += timestamp_now[ii] - timestamp_sent[ii];
    }
    double elements = timestamp_now.size();
    res.data = 1e3*temp.toSec()/elements;

    std::cout << "average transmission time " << res.data << " ms" << std::endl;
    return true;
}

bool BenchSubscr::get_frequency (ros_dvs_service::GetTime::Request &req,
                               ros_dvs_service::GetTime::Response &res)
{
    double temp;
    if (timestamp_now.size() != timestamp_sent.size() || timestamp_now.size() == 0)
        return false;

    double elements = timestamp_now.size();
    for (int ii = 1; ii < elements; ++ii)
    {
        temp += 1./(timestamp_now[ii] - timestamp_now[ii-1]).toSec();
    }
    res.data = temp/(elements-1);

    std::cout << "average frequency " << res.data << " Hz" << std::endl;
    return true;
}

void BenchSubscr::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
    timestamp_now.push_back(ros::Time::now());
    timestamp_sent.push_back(msg->events.back().ts);
    size_sent.push_back(msg->events.size());
}

} // namespace
