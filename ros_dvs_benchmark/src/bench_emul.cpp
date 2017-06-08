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

#include "ros_dvs_benchmark/bench_emul.h"

#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <random>

#include <iostream>

namespace ros_dvs_benchmark {

BenchEmul::BenchEmul(ros::NodeHandle & nh, ros::NodeHandle nh_private) :
    nh_(nh),
    iterations(1000),
    pubFreq(100)
{

    // set namespace
    std::string ns = ros::this_node::getNamespace();
    if (ns == "/")
        ns = "/dvs";
    event_array_pub_ = nh_.advertise<dvs_msgs::EventArray>(ns + "/events", 1);

    // spawn threads
    running_ = true;
    readout_thread_ = boost::shared_ptr< boost::thread >(new boost::thread(boost::bind(&BenchEmul::runBenchmark, this)));
}

BenchEmul::~BenchEmul()
{
    if (running_)
    {
        ROS_INFO("shutting down threads");
        running_ = false;
        readout_thread_->join();
        ROS_INFO("threads stopped");
    }

//    //Erase shared memory 2DO: does it make sense to remove in both classes?
//    bip::shared_memory_object::remove("shared_memory");
}

void BenchEmul::runBenchmark ()
{
    // create service client to get timings from subscriber and reset measurements
    ros::ServiceClient clientReset = nh_.serviceClient<ros_dvs_service::GetTime>("reset_measurements");
    ros::ServiceClient clientDuration = nh_.serviceClient<ros_dvs_service::GetTime>("get_duration");
    ros::ServiceClient clientFrequency = nh_.serviceClient<ros_dvs_service::GetTime>("get_frequency");
    // create service classes to perform service calls with
    ros_dvs_service::GetTime srvReset;
    ros_dvs_service::GetTime srvDuration;
    ros_dvs_service::GetTime srvFrequency;

    for (int packSize = 10; packSize < 40150; packSize+=5000)
    {
        std::cout << "------ publishing packets with " << packSize << " events ------" << std::endl;
        for (unsigned ii=0; ii<iterations; ++ii)
        {
            publishPacket(packSize, pubFreq);
        }
        ros::Duration(1.0).sleep();
        clientFrequency.call(srvFrequency);
        if ( srvFrequency.response.data > pubFreq * 1.5)
        {
            std::cout << "current subscriber frequency too high" << std::endl;
        }
        else if (srvFrequency.response.data < pubFreq * 0.5)
        {
            std::cout << "current subscriber frequency too low" << std::endl;
        }
        clientDuration.call(srvDuration);
        clientReset.call(srvReset);
    }
    ros::shutdown();
}

void BenchEmul::publishPacket(unsigned packetSize, unsigned loopFreq)
{
    ros::Rate loopRate(loopFreq);

    std::random_device rd; // obtain a random number from hardware
    std::mt19937 eng(rd()); // seed the generator
    std::uniform_int_distribution<> distrW(0, image_width-1); // define the range
    std::uniform_int_distribution<> distrH(0, image_height-1); // define the range

    dvs_msgs::EventArrayPtr event_array_msg(new dvs_msgs::EventArray());
    event_array_msg->height = image_height; //2DO: get dynamically
    event_array_msg->width = image_width;

    for (unsigned ii = 0; ii< packetSize; ++ii)
    {
        dvs_msgs::Event e;
        e.y = distrW(eng);
        e.x = distrH(eng);
        e.ts = ros::Time::now();
        e.polarity = true;
        event_array_msg->events.push_back(e);
    }

    event_array_pub_.publish(event_array_msg);  
    event_array_msg->events.clear();

    ros::spinOnce();

    loopRate.sleep();

}

} // namespace
