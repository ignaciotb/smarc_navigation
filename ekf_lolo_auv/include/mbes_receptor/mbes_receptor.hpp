#ifndef MBES_RECEPTOR_HPP
#define MBES_RECEPTOR_HPP

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/LaserScan.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MyTimerPolicy;

class MBESReceptor{

public:
    MBESReceptor(std::string node_name, ros::NodeHandle &nh);
    ~MBESReceptor();

private:
    ros::NodeHandle* nh_;
    std::string node_name_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* mbes_l_subs_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* mbes_r_subs_;
    message_filters::Synchronizer<MyTimerPolicy>* mbes_synch_;


    // Methods
    void mbesReadingsCB(const sensor_msgs::LaserScanConstPtr & mbes_l_msg, const sensor_msgs::LaserScanConstPtr &mbes_r_msg );

};


#endif // MBES_RECEPTOR_HPP