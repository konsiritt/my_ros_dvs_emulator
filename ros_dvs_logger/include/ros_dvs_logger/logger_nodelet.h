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

#ifndef DVS_LOGGER_NODELET_H_
#define DVS_LOGGER_NODELET_H_

#include <nodelet/nodelet.h>

#include "ros_dvs_logger/logger.h"

namespace ros_dvs_logger {

class DvsLoggerNodelet : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  ros_dvs_logger::EventLogger* eventLogger;
};

}

#endif // DVS_LOGGER_NODELET_H_
