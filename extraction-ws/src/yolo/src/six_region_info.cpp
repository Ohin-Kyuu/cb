#include "six_region_info.h"

SixRegion::SixRegion() {
    sub_six_region_count = nh.subscribe("/cbcam/objects/six_region_counts", 10, &SixRegion::sixRegionCountCallback, this);
    pub_global_info = nh.advertise<std_msgs::Int8MultiArray>("/robot/objects/global_info", 10);
}

void SixRegion::slidingWindow() {
    std_msgs::Int8MultiArray processed_averages;
    processed_averages.data.resize(received_arrays[0].size());
    
    for (size_t i = 0; i < received_arrays[0].size(); ++i) {
        int sum = 0;
        for (size_t j = 0; j < received_arrays.size(); ++j) {
            sum += received_arrays[j][i];
        }
        processed_averages.data[i] = sum / received_arrays.size();
    }
    
    pub_global_info.publish(processed_averages);
}

void SixRegion::sixRegionCountCallback(const std_msgs::Int8MultiArray::ConstPtr& msg) {
    std::vector<int> region_info;
    for (size_t i = 0; i < msg->data.size(); ++i) {
        if (msg->data[i] < min_plant_count) {
            region_info.push_back(0);
        } else {
            region_info.push_back(1);
        }
    }

    received_arrays.push_back(region_info);

    if (received_arrays.size() > window_size) {
        received_arrays.erase(received_arrays.begin());
    }
    
    slidingWindow();
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "six_region_info");
    SixRegion six_region;
    ros::spin();

    return 0;
}