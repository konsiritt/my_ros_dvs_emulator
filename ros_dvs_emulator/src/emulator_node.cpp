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

// boost shared mem
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

#include <ros/ros.h>

#include "ros_dvs_emulator/emulator.h"

namespace bip = boost::interprocess;

shared_mem_emul * dataShrdMain = NULL;

//shared_mem_emul * dataShrdMain;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ros_dvs_emulator");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // ---- memory sharing using boost library to use in emulator to ros interface ---//
    //Remove shared memory on destruction
    struct shm_remove
    {
        ~shm_remove(){ bip::shared_memory_object::remove("shared_memory"); }
    } remover;

    // initialize shared memory
    //Open the shared memory object.
    bip::shared_memory_object shm
            (bip::open_or_create       //open_only or open_or_create
             ,"shared_memory"           //name
             ,bip::read_write   //read-write mode
             );

    //Map the whole shared memory in this process
    bip::mapped_region region
            (shm                       //What to map
             ,bip::read_write   //Map it as read-only
             );

    //Get the address of the mapped region
    void * addr       = region.get_address();

    //Construct the shared structure in memory
    dataShrdMain = static_cast <shared_mem_emul*>(addr);

    {
        bip::scoped_lock<bip::interprocess_mutex> lock(dataShrdMain->mutex);
        ROS_INFO ( "testing shared mem access in emulator: timeNew: %0.2f imageNew[12]: %i", dataShrdMain->timeNew, (int) dataShrdMain->imageNew[12] );
    }

    std::cout << "Start the RosDvsEmulator object" << std::endl;
    ros_dvs_emulator::RosDvsEmulator* emulator = new ros_dvs_emulator::RosDvsEmulator(nh, nh_private, dataShrdMain);

    ros::spin();
    return 0;
}
