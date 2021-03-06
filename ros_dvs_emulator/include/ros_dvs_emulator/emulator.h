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

#pragma once

#include <ros/ros.h>
#include <string>

// threshold mismatch generation
#include <random>

// boost
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
// boost shared memory access
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>

// insert current time into logging file
#include <ctime>

// plot/log data
#include <cstdint>      // fixed width integer types
#include <fstream>      // std::ofstream
#include <iostream>
#define AEDAT3_FILE_VERSION 2.0

// perform big endian small endian conversions:
#include <arpa/inet.h>

// messages
#include <ros_dvs_msgs/Event.h>
#include <ros_dvs_msgs/EventArray.h>
#include <std_msgs/Empty.h>

// execution time measurement
#include "ros_dvs_emulator/Timer.h"

// dvs configuration file
#include "ros_dvs_emulator/config_dvs.h"

// plot/log data
#include <fstream>      // std::ofstream

//****************************************************************
///! Structs
//****************************************************************

//! struct that is kept in shared memory
//! ATTENTION: changes here need to be performed in TORCS main.cpp
//! as well, where frames are saved to shared memory
struct shared_mem_emul
{
    shared_mem_emul() :
        timeNew(0),
        timeRef(0),
        imageNew(),
        imageRef(),
        frameUpdated(false),
        frameIndex(0),
        mutex()
    {
    }
    //! time stamp of newly acquired frame
    double timeNew;
    //! time stamp of last frame
    double timeRef;
    //! RGBA values of newly acquired frame
    unsigned char imageNew[image_width*image_height*4];
    //! log(luminance) value of reference frame (last event generated)
    double imageRef[width_out*height_out];

    //! true when new frame was written
    bool frameUpdated;
    //! frame index to keep track of loss of frames saved to memory
    double frameIndex;

    //! Mutex to protect access to the queue
    boost::interprocess::interprocess_mutex mutex;

    //! Condition to wait on when the frame was not updated
    boost::interprocess::interprocess_condition  condNew;
    //! Condition to wait on when the frame was not processed
    boost::interprocess::interprocess_condition  condProcess;
};


namespace ros_dvs_emulator {

//! A class that generates address event representation from frames in shared memory
/*!
 * \brief The RosDvsEmulator class
 * this class handles frames written to shared memory by e.g. the car racing simulation
 * TORCS.
 * Through access to the struct shared_mem_emul also other sources for frames can be
 * used to perform a DVS emulation on
 */
class RosDvsEmulator {
public:
    RosDvsEmulator(ros::NodeHandle & nh, ros::NodeHandle nh_private, shared_mem_emul * dataShrd_);
    ~RosDvsEmulator();

private:

    //****************************************************************
    ///! Emulator functions
    //****************************************************************
    //! initializes all necessary fields and lookup tables
    void initEmulator();

    //! perform logarithmic scaling with linear scaling for values
    //! close to zero (expansive for log)
    inline double linlog(double arg)
    {
        if (arg>linLogLim)
        {
            return log(arg);
        }
        else
            return log(linLogLim)/linLogLim*arg;
    }

    //! perform logarithmic scaling with linear scaling for values
    //! close to zero (expansive for log), map to range 0 to 255
    inline double linlogKatz(double arg)
    {
        if (arg>linLogLim)
        {
            double a = ((double) lum_range - linLogLim) / (log(lum_range) - log(linLogLim));
            double b = lum_range - a *log(lum_range);


            return (a * log(arg) + b);
        }
        else
            return arg;
    }

    //! updates locally used times
    inline void updateTimes(double timeNew_, double timeRef_)
    {
        timeNew = timeNew_;
        timeRef = timeRef_;
        deltaStepTime = timeNew_ - timeRef_;
#ifdef interp_events
        //TODO: make emulation (sorting etc.) adaptable to different amount of slots
        for (int i = 0; i < interp_timeslots; ++i)
        {
            tSlot [i] = ros::Time(timeRef + deltaStepTime*(i+1)/(interp_timeslots+1));
        }
#else
        tStamp = ros::Time(timeRef + deltaStepTime/2);
#endif
    }

    //! create lookup table
    void createLookupLinLog();

    //! obtain threshold mismatches per pixel, with our without std deviation
    void createThresholdMismatch();

    //! access linear/logarithmic scaling via lookup table
    double linlog(uint16_t arg);

    //! performs emulation operation when new frame is available
    //! in shared memory, access managed via mutex and conditions
    void emulateFrame();
#ifdef interp_events
    //! writes event structure to array of created events
    int determineEvent(const int index, const int xCoord, const int yCoord,
                       const double deltaLum, const bool pol);
    int sortEvents(ros_dvs_msgs::EventArrayPtr& outEvents);
#else
    //! overloaded for when timeslots are not used -> directly written to the eventArray
    int determineEvent(const int index, const int xCoord, const int yCoord,
                       const double deltaLum, const bool pol,
                       ros_dvs_msgs::EventArrayPtr& outEvents);
#endif
    //****************************************************************
    ///! ROS related runtime variables
    //****************************************************************
    //! ROS node handle
    ros::NodeHandle nh_;    

    volatile bool running_;

    boost::shared_ptr<boost::thread> emulate_thread_;

    //****************************************************************
    ///! DVS emulation related variables
    //****************************************************************
    //! reference frame initialized yet?
    bool initializedRef;
    //! limit to which value linear scale will be applied
    //! out of range [0,255]
    double linLogLim;
    //! lookup table for linlog scaling of luminance
    double lookupLinLog[lum_range];
    //! event generation threshold value per pixel including std deviation
    //! (on/off) regarding threshold_mismatch
    double deviationThreshold[width_out*height_out];
    //! log(illumination) difference threshold,
    //! 2DO: make variable
    unsigned dvsThresh;
    //! rate at which event packages will be published
    //! 2DO: determine preference
    unsigned streamingRate;
#ifdef interp_events
    //! vectors for different timeslots for interpolating between frames
    std::vector<std::vector<ros_dvs_msgs::Event>> interpEvents;
    //! quantized timeslots for the amount of interpolated events
    ros::Time tSlot [interp_timeslots];
#else
    ros::Time tStamp;
#endif
    //! struct accessed in shared memory
    shared_mem_emul *dataShrd;
    //! counter of last frame index received
    double lastFrameIndex;
    //! locally saves currently used timestamps of frames and relative time
    double timeNew, timeRef, deltaStepTime;
    //! counters to assess applicability of timeslot choice
    double countMag1;
    double countMag2;
    double countMag3;
    double countMag4;
    double countMag5;

    //****************************************************************
    ///! DVS event output related
    //****************************************************************
    //! ROS publisher construct
    ros::Publisher event_array_pub_;

    //! Logging functions -> save events in AER-DAT file format
    int initLogging();
    int logAer(const ros_dvs_msgs::EventArrayPtr& outEvents);
    int logBinaryAer(const int32_t writeDataLittle);
    int closeLogging();

    //! directory for data logging
    std::string outputDirAEDat;
    //! file for logging aerdat event data
    std::ofstream eventsLog;


    //****************************************************************
    ///! Performance evaluation variables
    //****************************************************************
    //! timing helper functions that log timing
    void timePartMid(double& timeIt);
    void timePartEnd(double& timeIt);
    //! timer object taking timing measurements
    Timer t1;
    //! time for each part of the processing pipeline
    double tMutex;
    double tWait;
    double tProcess;
    double tPublish;
    //! evaluated frames, to perform averaging for timings
    unsigned framesCount;
    //! amount of events evaluated
    unsigned eventCount;
    //! counter that keeps track of generated packages
    //! receiving side can detect loss of packages
    uint64_t countPackages;
    //! time of last event in current package
    ros::Time timeLastEventOut;
    //! time of last performance output
    ros::Time timeLastPerfOut;

    //! total frames, since start of logging
    unsigned totalFrames;

    //****************************************************************
    ///! Plotting parameters
    //****************************************************************
    //! logs the used lookup table of logarithmic luminance intensity
    void logLookupTable ();
    //! directory for data logging
    std::string outputDir;
    //! output stream object for benchmark results logging
    std::ofstream linLogPlot;

};

} // namespace
