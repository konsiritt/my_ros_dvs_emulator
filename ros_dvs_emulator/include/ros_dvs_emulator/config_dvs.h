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

#ifndef CONFIG_DVS_H
#define CONFIG_DVS_H

//! DVS Emulation from TORCS configuration file:

//****************************************************************
///! FRAME INTERFACE SETTINGS (needs to be adapted in torcs/src/
///  libs/dvs/config_dvs.h as well, where frames are saved)
//****************************************************************

// game resolution used: TODO: obtain from shared memory
#define image_width 640 //320 //
#define image_height 480 //240 //
// BGRA: Blue Green Red Alpha channels
#define channel_size 4

// define frames_float if the color values should be obtained in
// GL_FLOAT [0,1] format from GPU instead of GL_UNSIGNED_BYTE [0,255]
#define frames_float //TODO: NOT IMPLEMENTED

//****************************************************************
///! EMULATOR SETTINGS
//****************************************************************

// use condition for processing frames without loss of potential frames
// not fit for real-time emulation
#define no_loss_frame_emulation
// use logarithmic scaling as in Katz_2012 in jAER implementation
#define use_katz_log_scale true //0 //
// pixel firing threshold
#define dvs_threshold 30.0 // 0.1// 0.22 vs Katz (10 for a range to 255)
// turn on/off use of standard deviation for pixel threshold values
#define threshold_mismatch true
// 1-sigma deviation in percent (DVS: 2.1% of contrast )
// from https://inilabs.com/products/dynamic-vision-sensors/specifications/
#define threshold_mismatch_sigma 0.02
// define to what value on linear rgb scale the log should be linear
#define lin_log_lim 15
// define the luminance range, here 0 to 255
#define lum_range 256

// define if interpolation between frames is allowed -> multiple events per pixel per frame
#define interp_events //#undef interp_events //
// defines the amount of timeslots for interframe interpolation
#define interp_timeslots 5

// set relative luminance contribution of rgb components https://en.wikipedia.org/wiki/Relative_luminance
#define lum_r 0.2126
#define lum_g 0.7152
#define lum_b 0.0732

//****************************************************************
///! EVENT OUTPUT INTERFACE
//****************************************************************

// save to aedat directly 1; publish via ROS topics 0
#define save_to_aedat true
// minimum packet size to save to aedat file after frame processing
#define save_to_aedat_min_size 1000
// define streaming rate of event packages in as ROS topic in [Hz]
#define ros_streaming_rate 200
// define maximum event package size
#define ros_max_package 30000

//****************************************************************
///! I/O BEHAVIOUR
//****************************************************************

// log performance and event creation frequency stats
#define emulator_io true

#endif // CONFIG_DVS_H

