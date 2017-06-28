#ifndef BENCH_EMUL_H
#define BENCH_EMUL_H

#include <ros/ros.h>
#include <string>

// boost
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// messages
#include <ros_dvs_msgs/Event.h>
#include <ros_dvs_msgs/EventArray.h>
#include <std_msgs/Empty.h>

// dvs configuration file
#include "ros_dvs_benchmark/config_dvs.h"

// service file
#include "ros_dvs_service/GetTime.h"


namespace ros_dvs_benchmark {

class BenchEmul {
public:
    BenchEmul(ros::NodeHandle & nh, ros::NodeHandle nh_private);
    ~BenchEmul();

private:

    void runBenchmark ();

    void publishPacket(unsigned packetSize);

    uint64_t countPackages;

    ros::NodeHandle nh_;
    ros::Publisher event_array_pub_;

    volatile bool running_;

    boost::shared_ptr<boost::thread> readout_thread_;

    unsigned iterations;

    unsigned pubFreq;

};

} // namespace


#endif // BENCH_EMUL_H

