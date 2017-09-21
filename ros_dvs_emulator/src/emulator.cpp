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

namespace ros_dvs_emulator {

RosDvsEmulator::RosDvsEmulator(ros::NodeHandle & nh, ros::NodeHandle nh_private, shared_mem_emul * dataShrd_) :    
    nh_(nh),
    initializedRef(false),
    linLogLim(lin_log_lim),
    dvsThresh(dvs_threshold),
    streamingRate(ros_streaming_rate),
#ifdef interp_events
    interpEvents(interp_timeslots), //TODO: make generic?!
#endif
    dataShrd(dataShrd_),
    lastFrameIndex(0),
    timeNew(0),
    timeRef(0),
    deltaStepTime(0),
    countMag1(0),
    countMag2(0),
    countMag3(0),
    countMag4(0),
    countMag5(0),
    outputDirAEDat("/home/rittk/Documents/torcsGT"),
    tMutex(0),
    tWait(0),
    tProcess(0),
    tPublish(0),
    framesCount(0),
    eventCount(0),
    countPackages(0),
    timeLastEventOut(0),
    timeLastPerfOut(0),
    totalFrames(0),
    outputDir("/home/rittk/devel/catkin_torcs_ros/logs/output/ros_dvs_emulator") //TODO: make adaptable
{
    // set namespace
    std::string ns = ros::this_node::getNamespace();
    if (ns == "/")
        ns = "/dvs";
    if (!save_to_aedat) {
        event_array_pub_ = nh_.advertise<ros_dvs_msgs::EventArray>(ns + "/events", 1);
    }

    initEmulator();

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
    closeLogging();
}

void RosDvsEmulator::initEmulator()
{
    createLookupLinLog();
    createThresholdMismatch();
//    logLookupTable();
    initLogging();
}

void RosDvsEmulator::createLookupLinLog()
{

    for (uint16_t ii = 0; ii<lum_range; ++ii)
    {
        if (use_katz_log_scale)
        {
            lookupLinLog[ii] = linlogKatz((double) ii);
        }
        else
        {
            lookupLinLog[ii] = linlog((double) ii);
        }
    }
}

void RosDvsEmulator::createThresholdMismatch()
{
    if (threshold_mismatch)
    {
        std::random_device rd;
        std::default_random_engine generator(rd());
        double mean = dvs_threshold;
        double stddev = dvs_threshold*threshold_mismatch_sigma;
        std::normal_distribution<double> distribution(mean,stddev);

        for (int ii = 0; ii < width_out*height_out; ++ii)
        {
            deviationThreshold[ii] = distribution(generator);
        }
    }
    else
    {
        for (int ii = 0; ii < width_out*height_out; ++ii)
        {
            deviationThreshold[ii] = dvs_threshold;
        }
    }
}

double RosDvsEmulator::linlog(uint16_t arg)
{
    if (arg < 0)
    {
        arg = 0;
    }
    else if (arg > lum_range)
    {
        arg = lum_range - 1;
    }
    return lookupLinLog[arg];
}

void RosDvsEmulator::emulateFrame()
{
    ros_dvs_msgs::EventArrayPtr event_array_msg(new ros_dvs_msgs::EventArray());    
    event_array_msg->height = height_out;//image_height; //2DO: get dynamically
    event_array_msg->width = width_out;//image_width;
    int sizePic = event_array_msg->height*event_array_msg->width;

#if save_to_aedat
    boost::posix_time::ptime next_send_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration delta_ = boost::posix_time::microseconds(1e6/streamingRate);
#endif

    while (running_)
    {
        try
        {
            if (emulator_io) t1.start();

            double pixelLuminance = 0;
            //****************************************************************
            ///! Emulation of events: through access to shared memory
            //****************************************************************
            // compute luminance difference for each pixel
            {
                totalFrames++;
                // start with: lock mutex while reading
                bip::scoped_lock<bip::interprocess_mutex> lock(dataShrd->mutex);

                if (emulator_io) timePartMid(tMutex);

                // wait till notified by frame saving process if no new frame is available to process
                if (!dataShrd->frameUpdated)
                {                    
                    dataShrd->condNew.wait(lock);
                }

                if (emulator_io) timePartMid(tWait);

                // check for loss of frames in shared memory
                if (!initializedRef)
                {
                    lastFrameIndex = dataShrd->frameIndex;
                }
                else if (++lastFrameIndex != dataShrd->frameIndex)
                {                    
                    lastFrameIndex = dataShrd->frameIndex;
                }

                // update timestamps for currently evaluated frames
                updateTimes(dataShrd->timeNew, dataShrd->timeRef);

                // iterate through every pixel in the newly obtained frame
                for (int ii=0; ii<sizePic; ++ii)
                {
                    // for the very first frame: just save reference brightness values and don't emit any events
                    if (!initializedRef)
                    {
                        if (super_sample)
                        {
                            dataShrd->imageRef[ii] = linlog((uint16_t)
                                    ((lum_b*(dataShrd->imageNew[8*ii+4*image_width*(ii/width_out)] + dataShrd->imageNew[8*ii+4+4*image_width*(ii/width_out)] +
                                    dataShrd->imageNew[8*ii+4*image_width*(ii/width_out+1)] + dataShrd->imageNew[8*ii+4+4*image_width*(ii/width_out+1)])
                                    + lum_g*(dataShrd->imageNew[8*ii+1+4*image_width*(ii/width_out)] + dataShrd->imageNew[8*ii+5+4*image_width*(ii/width_out)] +
                                    dataShrd->imageNew[8*ii+1+4*image_width*(ii/width_out+1)] + dataShrd->imageNew[8*ii+5+4*image_width*(ii/width_out+1)])
                                    + lum_r*(dataShrd->imageNew[8*ii+2+4*image_width*(ii/width_out)] + dataShrd->imageNew[8*ii+6+4*image_width*(ii/width_out)] +
                                    dataShrd->imageNew[8*ii+2+4*image_width*(ii/width_out+1)] + dataShrd->imageNew[8*ii+6+4*image_width*(ii/width_out+1)]) )/4.0) );
                        }
                        else
                        {
                            dataShrd->imageRef[ii] = linlog((uint16_t) (lum_b*dataShrd->imageNew[4*ii] + lum_g*dataShrd->imageNew[4*ii+1] + lum_r*dataShrd->imageNew[4*ii+2]) );
                        }
                        continue;
                    }

                    if (super_sample)
                    {
                    //TODO: currently colorspace equally, luminance obtained from three colors equally (unlike humans)
                        pixelLuminance = linlog((uint16_t) (
                                                (lum_b*(dataShrd->imageNew[8*ii+4*image_width*(ii/width_out)] + dataShrd->imageNew[8*ii+4+4*image_width*(ii/width_out)] +
                                                dataShrd->imageNew[8*ii+4*image_width*(ii/width_out+1)] + dataShrd->imageNew[8*ii+4+4*image_width*(ii/width_out+1)])
                                               + lum_g*(dataShrd->imageNew[8*ii+1+4*image_width*(ii/width_out)] + dataShrd->imageNew[8*ii+5+4*image_width*(ii/width_out)] +
                                                dataShrd->imageNew[8*ii+1+4*image_width*(ii/width_out+1)] + dataShrd->imageNew[8*ii+5+4*image_width*(ii/width_out+1)])
                                               + lum_r*(dataShrd->imageNew[8*ii+2+4*image_width*(ii/width_out)] + dataShrd->imageNew[8*ii+6+4*image_width*(ii/width_out)] +
                                                dataShrd->imageNew[8*ii+2+4*image_width*(ii/width_out+1)] + dataShrd->imageNew[8*ii+6+4*image_width*(ii/width_out+1)]) )/4.0) )
                                               - dataShrd->imageRef[ii];
                    }
                    else
                    {
                        pixelLuminance = linlog((uint16_t) (lum_b*dataShrd->imageNew[4*ii] + lum_g*dataShrd->imageNew[4*ii+1] + lum_r*dataShrd->imageNew[4*ii+2]) )
                                - dataShrd->imageRef[ii];
                    }

                    // determine event polarity
                    bool positiveVal = false;
                    if (pixelLuminance > 0)
                    {
                        positiveVal = true;
                    }
                    pixelLuminance = abs(pixelLuminance);

#ifdef interp_events
                    if (pixelLuminance>deviationThreshold[ii])
                    {
                        determineEvent(ii, ii%event_array_msg->width, (int) ii/event_array_msg->width,
                                       pixelLuminance, positiveVal);
                    }
#else
                    if (pixelLuminance>deviationThreshold[ii])
                    {
                        determineEvent(ii, ii%event_array_msg->width, (int) ii/event_array_msg->width,
                                       pixelLuminance, positiveVal, event_array_msg);
                    }
#endif
                } // end iterate through every pixel in the newly obtained frame                

                //flag that the current new frame has been used
                dataShrd->frameUpdated = false;
#ifdef no_frame_loss_emulation
                // notify the frame creating process to continue with next
                dataShrd->condProcess.notify_one();
#endif // no_frame_loss_emulation

            } // end scope with lock on mutex of shared memory

            // for first run of emulator: mark reference frame as initialized
            if (!initializedRef)
            {
                initializedRef = true;
                ros::spinOnce();
                continue;
            }

#ifdef interp_events
            sortEvents(event_array_msg);
#endif

            // obtain time of last generated event for periodic performance output
            if (event_array_msg->events.size()>0)
            {
                timeLastEventOut = event_array_msg->events.back().ts;
            }
            //****************************************************************
            if(emulator_io) timePartMid(tProcess);

            //****************************************************************
            ///! Output of Events: Interface to ROS or logging to .aedat file
            //****************************************************************
            // save emulated events directly to file
            if (save_to_aedat && event_array_msg->events.size() > save_to_aedat_min_size)
            {
                eventCount += event_array_msg->events.size();

                if (logAer(event_array_msg))
                {
                    // clean up after events were saved
                    event_array_msg->events.clear();
                }
            }
            // throttle event messages if sent via ROS
            else if (!save_to_aedat && (boost::posix_time::microsec_clock::local_time() > next_send_time ||
                    streamingRate == 0 || event_array_msg->events.size() > ros_max_package) )
            {
                //count new events before they are published and then discarded
                eventCount += event_array_msg->events.size();

                event_array_msg->counter = countPackages++;
                event_array_pub_.publish(event_array_msg);
                event_array_msg->events.clear();
                if (streamingRate > 0)
                {
                    next_send_time += delta_;
                }
            }
            //****************************************************************
            if(emulator_io) timePartEnd(tPublish);

            ros::spinOnce();

        } //end try
        catch (boost::thread_interrupted&)
        {
            std::cout << "thread interrupted" << std::endl;
            return;
        }
    } //end while(_running)
}
#ifdef interp_events
int RosDvsEmulator::determineEvent(const int index, const int xCoord, const int yCoord,
                                   const double deltaLum, const bool pol)
{
    int magnitude = 0;
    double tempLum = deltaLum;
    while (tempLum > deviationThreshold[index] && magnitude < interp_timeslots)
    {
        magnitude++;
        tempLum = tempLum - deviationThreshold[index];
    }
    if (magnitude !=0)
    {
        // create ROS event message
        ros_dvs_msgs::Event e;
        e.x = xCoord;
        e.y = yCoord;
        e.polarity = pol;

        // switch amount of events per intraframe period and synthetically produce according events
        switch (magnitude) {
        case 1:
            e.ts = tSlot[2];
            interpEvents[2].push_back(e);
            countMag1++;
            break;
        case 2:
            e.ts = tSlot[1];
            interpEvents[1].push_back(e);
            e.ts = tSlot[3];
            interpEvents[3].push_back(e);
            countMag2++;
            break;
        case 3:
            e.ts = tSlot[1];
            interpEvents[1].push_back(e);
            e.ts = tSlot[2];
            interpEvents[2].push_back(e);
            e.ts = tSlot[3];
            interpEvents[3].push_back(e);
            countMag3++;
            break;
        case 4:
            e.ts = tSlot[0];
            interpEvents[0].push_back(e);
            e.ts = tSlot[1];
            interpEvents[1].push_back(e);
            e.ts = tSlot[3];
            interpEvents[3].push_back(e);
            e.ts = tSlot[4];
            interpEvents[4].push_back(e);
            countMag4++;
            break;
        case 5:
            e.ts = tSlot[0];
            interpEvents[0].push_back(e);
            e.ts = tSlot[1];
            interpEvents[1].push_back(e);
            e.ts = tSlot[2];
            interpEvents[2].push_back(e);
            e.ts = tSlot[3];
            interpEvents[3].push_back(e);
            e.ts = tSlot[4];
            interpEvents[4].push_back(e);
            countMag5++;
            break;
        default:
            std::cout << "undefined threshold change" << std::endl;
        }

        // update reference frame according to polarity of event and amount of events generated
        if (pol)
        {
            if (ref_update_full)
            {
                dataShrd->imageRef[index] = dataShrd->imageRef[index] + deltaLum;
            }
            else
            {
                dataShrd->imageRef[index] = dataShrd->imageRef[index] + magnitude*deviationThreshold[index];
            }
        }
        else
        {
            if (ref_update_full)
            {
                dataShrd->imageRef[index] = dataShrd->imageRef[index] - deltaLum;
            }
            else
            {
                dataShrd->imageRef[index] = dataShrd->imageRef[index] - magnitude*deviationThreshold[index];
            }
        }

    }

}

int RosDvsEmulator::sortEvents(ros_dvs_msgs::EventArrayPtr& outEvents)
{
    double checkTotal = 0;
    double initialEventAmount = outEvents->events.size();
    // sort different timeslot event packets to achieve monotonous timestamps in the overall sent package
    for (int ii = 0; ii < interp_timeslots; ++ii)
    {

        if (interpEvents[ii].size()!=0)
        {
            checkTotal += interpEvents[ii].size();
            outEvents->events.insert(outEvents->events.end(),interpEvents[ii].begin(),
                                           interpEvents[ii].end());
            interpEvents[ii].clear();
        }
    }
    if (checkTotal != outEvents->events.size()-initialEventAmount)
    {
        std::cerr << "sth went wrong concatenating the timeslot events, "<< checkTotal <<
                     " should be in it, " << outEvents->events.size() << " are" << std::endl;
        return 0;
    }
    return 1;
}
#else
int RosDvsEmulator::determineEvent(const int index, const int xCoord, const int yCoord,
                                   const double deltaLum, const bool pol,
                                   ros_dvs_msgs::EventArrayPtr& outEvents)
{
    // create ROS event message
    ros_dvs_msgs::Event e;
    e.x = xCoord;
    e.y = yCoord;
    e.ts = tStamp;
    e.polarity = pol;

    // TODO: currently only updated by difference, not by event represented difference
    // update reference frame according to polarity of event
    if (pol)
    {
        dataShrd->imageRef[index] = dataShrd->imageRef[index] + deltaLum;
    }
    else
    {
        dataShrd->imageRef[index] = dataShrd->imageRef[index] - deltaLum;
    }

    // add event message to message array
    outEvents->events.push_back(e);
}
#endif

int RosDvsEmulator::initLogging()
{
    if( !eventsLog.is_open() )
    {
        if (!eventsLog)
        {
            std::cerr << "something went wrong when opening the logging file" << std::endl;
        }
        std::cout << "opening file to log results to " << (outputDirAEDat +"/eventLog.aedat").c_str() << std::endl;
        eventsLog.open((outputDirAEDat +"/eventLog.aedat").c_str(),std::ofstream::binary | std::ios::out | std::ios::trunc);
    }
    else
    {
        std::cerr <<"Could not open file!\n" << std::endl;
        return -1;
    }
    eventsLog.write( (const char *) "#!AER-DAT2.0\r\n",14);
    eventsLog.write( (const char *) "# This is a raw AE data file - do not edit\r\n", 44);
    eventsLog.write( (const char *) "# Data format is int32 address, int32 timestamp (8 bytes total), repeated for each event\r\n", 90);
    eventsLog.write( (const char *) "# Timestamps tick is 1 us\r\n", 27);

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

return 1;
}

int RosDvsEmulator::logAer(const ros_dvs_msgs::EventArrayPtr& outEvents)
{
    int32_t writeDataLittle = 0;
    int32_t xScreen = 0;
    int32_t yScreen = 0;
    int32_t onPolarity = 2048; // bit at Position 11 is 1 -> 2^11 = 2048

    int32_t timeStamp = 0;

    int32_t imageWidth = outEvents->width;

    // perform conversion to 32 bit signed integers for each event created
    for (int ii = 0; ii < outEvents->events.size(); ++ii)
    {
        xScreen = (imageWidth - 1 - outEvents->events[ii].x) << 12;
        yScreen = outEvents->events[ii].y << 22;

        writeDataLittle = xScreen | yScreen;

        if (outEvents->events[ii].polarity)
        {
            writeDataLittle = writeDataLittle | onPolarity;
        }

        timeStamp = (int32_t) outEvents->events[ii].ts.sec*1000000 + (int32_t) outEvents->events[ii].ts.nsec*1e-3 ;
        if (timeStamp < 0 )
        {
            std::cout<<"Attention, timestamp is " << timeStamp << " but should not be negative" <<std::endl;
            std::cout<<"sec: " << outEvents->events[ii].ts.sec << "nsec: " << outEvents->events[ii].ts.nsec <<std::endl;
        }

        // write ints to file
        logBinaryAer(writeDataLittle);
        logBinaryAer(timeStamp);
    }
    return 1;
}
int RosDvsEmulator::logBinaryAer(const int32_t writeDataLittle)
{
    int32_t writeDataBig = __builtin_bswap32(writeDataLittle);
    eventsLog.write((char *) (&writeDataBig), 4);
    return 1;
}

int RosDvsEmulator::closeLogging()
{
    eventsLog.close();
    return 1;
}

void RosDvsEmulator::timePartMid(double& timeIt)
{
    t1.stop();
    timeIt += t1.getElapsedTimeInMilliSec();
    t1.start();
}
void RosDvsEmulator::timePartEnd(double& timeIt)
{
    t1.stop();
    timeIt += t1.getElapsedTimeInMilliSec();

    //****************************************************************
    ///! Status output: logging of stats to terminal every second (simulation time)
    //****************************************************************
    ++framesCount;
    // perform performance output:
    if (emulator_io && timeLastEventOut - timeLastPerfOut > ros::Duration(1) )
    {
        double deltaTimeOut = (timeLastEventOut - timeLastPerfOut).toSec();
        timeLastPerfOut = timeLastEventOut;
        std::cout << " " << std::endl;
        std::cout << "current event rate: " << eventCount/deltaTimeOut/1000 << "[keps]" << std::endl;

        std::cout << " Average times: \n mutex - \t" << tMutex/framesCount
                  << "ms, \n wait - \t"             << tWait/framesCount
                  << "ms, \n process - \t"          << tProcess/framesCount
                  << "ms, \n process/e - \t"          << tProcess/eventCount
                  << "ms, \n publish - \t"           << tPublish/framesCount
                  << "ms, \n total - \t"            << (tPublish+tProcess+tWait+tMutex)/framesCount
                  << "ms" << std::endl;

        std::cout << "\n magnitude 1 events: " << countMag1
                  << "\n magnitude 2 events: " << countMag2
                  << "\n magnitude 3 events: " << countMag3
                  << "\n magnitude 4 events: " << countMag4
                  << "\n magnitude 5 events: " << countMag5 << std::endl;

        std::cout << "\n total frames emulated: " << totalFrames << std::endl;
        tMutex = 0;
        tWait = 0;
        tProcess = 0;
        tPublish = 0;
        framesCount = 0;
        eventCount = 0;
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

    for (uint16_t ii=0; ii<lum_range; ++ii)
    {
        linLogPlot << ii << " " << linlog((double) ii) << " "
                   << ii << " " << linlogKatz((double) ii) <<  std::endl;
    }

    linLogPlot.close();
}

} // namespace
