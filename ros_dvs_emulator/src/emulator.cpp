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

#include "ros_dvs_emulator/emulator.h"

#include <boost/interprocess/sync/interprocess_mutex.hpp>

namespace bip = boost::interprocess;

extern  shared_mem_emul * dataShrdMain;

namespace ros_dvs_emulator {

RosDvsEmulator::RosDvsEmulator(ros::NodeHandle & nh, ros::NodeHandle nh_private) :
    nh_(nh),
    linLogLim(lin_log_lim),
    dvsThresh(dvs_threshold),
    streamingRate(ros_streaming_rate),
#ifdef interp_events
    interpEvents(interp_timeslots), //2DO: make generic?!
#endif
    tProcess(0),
    tPublish(0),
    framesCount(0),
    eventCount(0),
    countPackages(0),
    outputDir("/home/rittk/devel/catkin_torcs_ros/logs/output/ros_dvs_emulator") //2DO: make adaptable
{
    // set namespace
    std::string ns = ros::this_node::getNamespace();
    if (ns == "/")
        ns = "/dvs";
    event_array_pub_ = nh_.advertise<ros_dvs_msgs::EventArray>(ns + "/events", 1);

    dataShrd = dataShrdMain;

    for (uint16_t ii = 0; ii<256; ++ii)
    {
        lookupLinLog[ii] = linlog((double) ii);
    }
    //    logLookupTable();

    // spawn threads
    running_ = true;
    std::cout << "Now spawning threads" << std::endl;
    emulate_thread_ = boost::shared_ptr< boost::thread >(new boost::thread(boost::bind(&RosDvsEmulator::emulateFrame, this)));
}

RosDvsEmulator::RosDvsEmulator(ros::NodeHandle & nh, ros::NodeHandle nh_private, shared_mem_emul * dataShrd_) :    
    nh_(nh),
    linLogLim(lin_log_lim),
    dvsThresh(dvs_threshold),
    streamingRate(ros_streaming_rate),
#ifdef interp_events
    interpEvents(interp_timeslots), //2DO: make generic?!
#endif
    dataShrd(dataShrd_),
    tMutex(0),
    tWait(0),
    tProcess(0),
    tPublish(0),
    framesCount(0),
    eventCount(0),
    countPackages(0),
    outputDir("/home/rittk/devel/catkin_torcs_ros/logs/output/ros_dvs_emulator") //2DO: make adaptable
{
    // set namespace
    std::string ns = ros::this_node::getNamespace();
    if (ns == "/")
        ns = "/dvs";
    event_array_pub_ = nh_.advertise<ros_dvs_msgs::EventArray>(ns + "/events", 1);

    for (uint16_t ii = 0; ii<256; ++ii)
    {
        lookupLinLog[ii] = linlog((double) ii);
    }
    //    logLookupTable();

    // spawn threads
    running_ = true;
    std::cout << "Now spawning threads" << std::endl;
    emulate_thread_ = boost::shared_ptr< boost::thread >(new boost::thread(boost::bind(&RosDvsEmulator::emulateFrame, this)));
}

RosDvsEmulator::~RosDvsEmulator()
{
    if (running_)
    {
        ROS_INFO("shutting down threads");
        running_ = false;
        emulate_thread_->join();
        ROS_INFO("threads stopped");
    }

    //    //Erase shared memory 2DO: does it make sense to remove in both classes?
    //    bip::shared_memory_object::remove("shared_memory");
}

double RosDvsEmulator::linlog(uint16_t arg)
{
    if (arg < 256)
    {
        return lookupLinLog[arg];
    }
    else
    {
        std::cerr << "wrong illuminance value computed" << std::endl;
        return 0;
    }
}

void RosDvsEmulator::emulateFrame()
{
    ros_dvs_msgs::EventArrayPtr event_array_msg(new ros_dvs_msgs::EventArray());
    event_array_msg->height = image_height; //2DO: get dynamically
    event_array_msg->width = image_width;

    boost::posix_time::ptime next_send_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration delta_ = boost::posix_time::microseconds(1e6/streamingRate);

    while (running_)
    {
        try
        {
            t1.start();

            double pixelLuminance = 0;
            int sizePic = event_array_msg->height*event_array_msg->width;
            // compute luminance difference for each pixel
            // to do that: lock mutex while reading
            {
                bip::scoped_lock<bip::interprocess_mutex> lock(dataShrd->mutex);
                t1.stop();
                tMutex += t1.getElapsedTimeInMilliSec();
                t1.start();

                if (!dataShrd->frameUpdated)
                {
                    // wait till notified by frame saving process if no new frame is available to process
                    dataShrd->condNew.wait(lock);
                }
                t1.stop();
                tWait += t1.getElapsedTimeInMilliSec();
                t1.start();

                // iterate through every pixel in the newly obtained frame
                for (int ii=0; ii<sizePic; ++ii)
                {
                    // 2DO: currently colorspace equally, luminance obtained from three colors equally (unlike humans)
                    pixelLuminance = linlog((uint16_t) (1.0/3.0*(dataShrd->imageNew[4*ii] + dataShrd->imageNew[4*ii+1] + dataShrd->imageNew[4*ii+2])) )
                            - dataShrd->imageRef[ii];

                    // determine event polarity
                    bool positiveVal = false;
                    if (pixelLuminance > 0)
                    {
                        positiveVal = true;
                    }
                    pixelLuminance = abs(pixelLuminance);

#ifdef interp_events
                    int magnitude = 0;
                    double tempLum = pixelLuminance;
                    while (tempLum > dvsThresh && magnitude < interp_timeslots)
                    {
                        magnitude++;
                        tempLum = tempLum - dvsThresh;
                    }
                    if (magnitude !=0)
                    {
                        // create ROS event message
                        ros_dvs_msgs::Event e;
                        e.y = (int) ii/event_array_msg->width;
                        e.x = ii%event_array_msg->width;
                        e.polarity = positiveVal;

                        switch (magnitude) {
                        case 1:
                            e.ts = ros::Time(dataShrd->timeRef + (dataShrd->timeNew - dataShrd->timeRef)/2.0);
                            interpEvents[2].push_back(e);
                            break;
                        case 2:
                            e.ts = ros::Time(dataShrd->timeRef + (dataShrd->timeNew - dataShrd->timeRef)*1.0/3.0);
                            interpEvents[1].push_back(e);
                            e.ts = ros::Time(dataShrd->timeRef + (dataShrd->timeNew - dataShrd->timeRef)*2.0/3.0);
                            interpEvents[3].push_back(e);
                            break;
                        case 3:
                            e.ts = ros::Time(dataShrd->timeRef + (dataShrd->timeNew - dataShrd->timeRef)*1.0/3.0);
                            interpEvents[1].push_back(e);
                            e.ts = ros::Time(dataShrd->timeRef + (dataShrd->timeNew - dataShrd->timeRef)/2.0);
                            interpEvents[2].push_back(e);
                            e.ts = ros::Time(dataShrd->timeRef + (dataShrd->timeNew - dataShrd->timeRef)*2.0/3.0);
                            interpEvents[3].push_back(e);
                            break;
                        case 4:
                            e.ts = ros::Time(dataShrd->timeRef + (dataShrd->timeNew - dataShrd->timeRef)*1.0/6.0);
                            interpEvents[0].push_back(e);
                            e.ts = ros::Time(dataShrd->timeRef + (dataShrd->timeNew - dataShrd->timeRef)*1.0/3.0);
                            interpEvents[1].push_back(e);
                            e.ts = ros::Time(dataShrd->timeRef + (dataShrd->timeNew - dataShrd->timeRef)*2.0/3.0);
                            interpEvents[3].push_back(e);
                            e.ts = ros::Time(dataShrd->timeRef + (dataShrd->timeNew - dataShrd->timeRef)*5.0/6.0);
                            interpEvents[4].push_back(e);
                            break;
                        case 5:
                            e.ts = ros::Time(dataShrd->timeRef + (dataShrd->timeNew - dataShrd->timeRef)*1.0/6.0);
                            interpEvents[0].push_back(e);
                            e.ts = ros::Time(dataShrd->timeRef + (dataShrd->timeNew - dataShrd->timeRef)*1.0/3.0);
                            interpEvents[1].push_back(e);
                            e.ts = ros::Time(dataShrd->timeRef + (dataShrd->timeNew - dataShrd->timeRef)/2.0);
                            interpEvents[2].push_back(e);
                            e.ts = ros::Time(dataShrd->timeRef + (dataShrd->timeNew - dataShrd->timeRef)*2.0/3.0);
                            interpEvents[3].push_back(e);
                            e.ts = ros::Time(dataShrd->timeRef + (dataShrd->timeNew - dataShrd->timeRef)*5.0/6.0);
                            interpEvents[4].push_back(e);
                            break;
                        default:
                            std::cout << "undefined threshold change" << std::endl;
                        }

                        // update reference frame according to polarity of event
                        if (positiveVal)
                        {
                            dataShrd->imageRef[ii] = dataShrd->imageRef[ii] + pixelLuminance;
                        }
                        else
                        {
                            dataShrd->imageRef[ii] = dataShrd->imageRef[ii] - pixelLuminance;
                        }

                    }
#else
                    if (pixelLuminance>dvsThresh)
                    {

                        // create ROS event message
                        ros_dvs_msgs::Event e;
                        e.y = (int) ii/event_array_msg->width;
                        e.x = ii%event_array_msg->width;
                        e.ts = ros::Time(dataShrd->timeNew - (dataShrd->timeNew - dataShrd->timeRef)/2);
                        e.polarity = positiveVal;

                        // update reference frame according to polarity of event
                        if (positiveVal)
                        {
                            dataShrd->imageRef[ii] = dataShrd->imageRef[ii] + pixelLuminance;
                        }
                        else
                        {
                            dataShrd->imageRef[ii] = dataShrd->imageRef[ii] - pixelLuminance;
                        }

                        // add event message to message array
                        event_array_msg->events.push_back(e);
                    }
#endif
                }

#ifdef interp_events
//                event_array_msg->events.reserve(event_array_msg->events.size() + interpEvents[0].size()
//                        + interpEvents[1].size() + interpEvents[2].size()
//                        + interpEvents[3].size() + interpEvents[4].size());
                for (int ii = 0; ii < interp_timeslots; ++ii)
                {
                    if (interpEvents[ii].size()!=0)
                    {
                        event_array_msg->events.insert(event_array_msg->events.end(),interpEvents[ii].begin(),
                                                       interpEvents[ii].end());
                        interpEvents[ii].clear();
                    }
                }
#endif
                //flag that the current new frame has been used
                dataShrd->frameUpdated = false;
            }
            t1.stop();
            tProcess += t1.getElapsedTimeInMilliSec();
            t1.start();

            // throttle event messages
            if (boost::posix_time::microsec_clock::local_time() > next_send_time || streamingRate == 0)
            {
                eventCount += event_array_msg->events.size();

                event_array_msg->counter = countPackages++;
                event_array_pub_.publish(event_array_msg);
                event_array_msg->events.clear();
                if (streamingRate > 0)
                {
                    next_send_time += delta_;
                }

                ++framesCount;
                if (framesCount == 60) //((tPublish + tProcess) >= 1000)
                {
                    std::cout << "Amount of events in array: " << eventCount << std::endl;

                    std::cout << " Average times: \n mutex - \t" << tMutex/framesCount
                              << "ms, \n wait - \t"             << tWait/framesCount
                              << "ms, \n process - \t"          << tProcess/framesCount
                              << "ms, \n publish - \t"           << tPublish/framesCount
                              << "ms, \n total - \t"            << (tPublish+tProcess+tWait+tMutex)/framesCount
                              << "ms" << std::endl;
                    tMutex = 0;
                    tWait = 0;
                    tProcess = 0;
                    tPublish = 0;
                    framesCount = 0;
                    eventCount = 0;

                }
            }
            //            event_array_pub_.publish(event_array_msg);
            //            event_array_msg->events.clear();

            t1.stop();
            tPublish += t1.getElapsedTimeInMilliSec();



            //            std::cout << "ros::spinOnce now" << std::endl;
            ros::spinOnce();
        }
        catch (boost::thread_interrupted&)
        {
            std::cout << "thread interrupted" << std::endl;
            return;
        }
    }
}

void RosDvsEmulator::logLookupTable()
{
    if( !linLogPlot.is_open() )
    {
        if (!linLogPlot)
        {
            std::cerr << "something went wrong when opening the logging file" << std::endl;
            return;
        }
        std::cout << "plotting results of linlog mapping to " << (outputDir +"/linlog.out").c_str() << std::endl;
        linLogPlot.open((outputDir +"/linlog.out").c_str(),std::fstream::out); //std::ios::app);
    }
    else
    {
        std::cerr <<"Could not open file!\n" << std::endl;
        return;
    }

    for (uint16_t ii=0; ii<256; ++ii)
    {
        linLogPlot << ii << " " << linlog((double) ii) << std::endl;
    }
}

} // namespace
