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

#include "ros_dvs_emulator/emulator.h"

#include <boost/interprocess/sync/interprocess_mutex.hpp>

namespace bip = boost::interprocess;

extern  shared_mem_emul * dataShrdMain;

namespace ros_dvs_emulator {

RosDvsEmulator::RosDvsEmulator(ros::NodeHandle & nh, ros::NodeHandle nh_private) :
    nh_(nh),
    dvsThresh(dvs_threshold), //2DO: include in config file
    streamingRate(100),
    tProcess(0),
    tPublish(0),
    framesCount(0),
    eventCount(0)
{
    dataShrd = dataShrdMain;

    // set namespace
    std::string ns = ros::this_node::getNamespace();
    if (ns == "/")
        ns = "/dvs";
    event_array_pub_ = nh_.advertise<dvs_msgs::EventArray>(ns + "/events", 1);

    // spawn threads
    running_ = true;
    readout_thread_ = boost::shared_ptr< boost::thread >(new boost::thread(boost::bind(&RosDvsEmulator::readout, this)));
}

RosDvsEmulator::RosDvsEmulator(ros::NodeHandle & nh, ros::NodeHandle nh_private, shared_mem_emul * dataShrd_) :
    nh_(nh),
    dvsThresh(dvs_threshold), //2DO: include in config file
    streamingRate(200),
    dataShrd(dataShrd_),
    tMutex(0),
    tWait(0),
    tProcess(0),
    tPublish(0),
    framesCount(0),
    eventCount(0)
{
//    ROS_INFO( "testing shared mem access in emulator: aIsNew: %i timeA: %0.2f imageA[12]: %i", dataShrd->aIsNew, dataShrd->timeA, (int) dataShrd->imageA[12] );

    // set namespace
    std::string ns = ros::this_node::getNamespace();
    if (ns == "/")
        ns = "/dvs";
    event_array_pub_ = nh_.advertise<dvs_msgs::EventArray>(ns + "/events", 1);

    std::cout << "Spawn threads" << std::endl;
    // spawn threads
    running_ = true;
    readout_thread_ = boost::shared_ptr< boost::thread >(new boost::thread(boost::bind(&RosDvsEmulator::readout, this)));
}

RosDvsEmulator::~RosDvsEmulator()
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

void RosDvsEmulator::readout()
{
    dvs_msgs::EventArrayPtr event_array_msg(new dvs_msgs::EventArray());
    event_array_msg->height = image_height; //2DO: get dynamically
    event_array_msg->width = image_width;

    boost::posix_time::ptime next_send_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration delta_ = boost::posix_time::microseconds(1e6/streamingRate);

    while (running_)
    {
        try
        {

            t1.start();

            int temp_lum = 0;
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

                for (int ii=0; ii<sizePic; ++ii)
                {
                    // currently not rounding correctly, just to get visualization through ROS
                    temp_lum = (int) (0.33*(dataShrd->imageNew[4*ii] + dataShrd->imageNew[4*ii+1] + dataShrd->imageNew[4*ii+2]
                            - dataShrd->imageRef[4*ii] - dataShrd->imageRef[4*ii+1] - dataShrd->imageRef[4*ii+2]));
                    // 2DO: correct handling of luminance values...., proper rounding etc.

                    bool positiveVal = false;
                    if (temp_lum > 0)
                    {
                        positiveVal = true;
                    }
                    temp_lum = abs(temp_lum);

                    if (temp_lum>dvsThresh)
                    {
                        dvs_msgs::Event e;
                        e.y = event_array_msg->height - (int) ii/event_array_msg->width;
                        e.x = ii%event_array_msg->width;
                        e.ts = ros::Time(dataShrd->timeNew - (dataShrd->timeNew - dataShrd->timeRef)/2);
                        e.polarity = positiveVal;

                        dataShrd->imageRef[4*ii] = dataShrd->imageNew[4*ii];
                        dataShrd->imageRef[4*ii+1] = dataShrd->imageNew[4*ii+1];
                        dataShrd->imageRef[4*ii+2] = dataShrd->imageNew[4*ii+2];

                        event_array_msg->events.push_back(e);
                    }
                }
                //flag that the current new frame has been used
                dataShrd->frameUpdated = false;
            }
            t1.stop();
            tProcess += t1.getElapsedTimeInMilliSec();
            t1.start();

            //            eventCount += event_array_msg->events.size();


            // throttle event messages
            if (boost::posix_time::microsec_clock::local_time() > next_send_time || streamingRate == 0)
            {
                eventCount += event_array_msg->events.size();

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
                              << "ms, \n total - \t"            << (tPublish+tProcess)/framesCount
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

} // namespace
