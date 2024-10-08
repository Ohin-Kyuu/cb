#ifndef SIXREGION_INFO_H
#define SIXREGION_INFO_H

#include "ros/ros.h"
#include "std_msgs/Int8MultiArray.h"
#include <vector>

class SixRegion {
public:
    SixRegion();

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_six_region_count;
    ros::Publisher pub_global_info;
    std::vector<std::vector<int>> received_arrays;
    
    const int min_plant_count = 2;  // Minimum number of plants in a region to be considered in the region
    const int window_size = 5;  // Size of the sliding window

    void sixRegionCountCallback(const std_msgs::Int8MultiArray::ConstPtr& msg);
    void slidingWindow();
};

#endif // SIXREGION_INFO_H
