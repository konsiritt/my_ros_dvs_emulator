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

// define dvs variables here

#define image_width 640 //320 //
#define image_height 480 //240 //

#define pbo_count 2

// configure pixelBuffer and dvsEmulator:
#define pixel_format GL_BGRA //Fastest option for map from vram to ram
#define channel_size 4
#define dvs_threshold 1.0 //1.3 // 60 for lin values

// define to what value on linear rgb scale the log should be linear
#define lin_log_lim 20

// define streaming rate of event packages in as ROS topic in [Hz]
#define ros_streaming_rate 200

// define if interpolation between frames is allowed -> multiple events per pixel per frame
#define interp_events //#undef interp_events //
// defines the amount of timeslots for interframe interpolation
#define interp_timeslots 5

// define frames_float if the color values should be obtained in
// GL_FLOAT [0,1] format from GPU instead of GL_UNSIGNED_BYTE [0,255]
#define frames_float

#endif // CONFIG_DVS_H

