#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/Float32MultiArray.h"
#include <cmath>
#include <vector>
#include <std_msgs/Int8MultiArray.h>

class CameraPointListener {
public:
    CameraPointListener() {
        sub = nh.subscribe("/cbcam/objects/world_point_array", 10, &CameraPointListener::pointCallback, this);
        pub = nh.advertise<visualization_msgs::MarkerArray>("/cbcam/objects/six_region_marker", 10);
        region_pub = nh.advertise<std_msgs::Int8MultiArray>("/cbcam/objects/six_region_counts", 10);
    }

    void pointCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        markerArray.markers.clear();
        
        std_msgs::Int8MultiArray region_counts;
        region_counts.data.resize(6, 0); // Initialize the region count array with zeros

        std::vector<geometry_msgs::Point> world_points;
        for (size_t i = 0; i < msg->data.size(); i += 3) {
            geometry_msgs::Point point;
            point.x = msg->data[i];
            point.y = msg->data[i + 1];
            point.z = msg->data[i + 2];
            world_points.push_back(point);
            
            // Determine the region for each point and update the region count
            int region = getRegion(point);
            if (region >= 0 && region < 6) {
                region_counts.data[region]++;
                addMarker(point, i / 3, true); // True for red
            } else {
                addMarker(point, i / 3, false); // False for white
            }
        }

        // Publish the region counts
        region_pub.publish(region_counts);
        // Publish the markers for visualization
        pub.publish(markerArray);
    }

    void addMarker(const geometry_msgs::Point& point, int id, bool isRed) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "world_points";
        marker.id = id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = point;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
        if (isRed) {
            marker.color.r = 1.0;
            marker.color.a = 1.0;
        } else {
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
        }
        markerArray.markers.push_back(marker);
    }

    int getRegion(const geometry_msgs::Point& point) {
        // Define the regions as circles and check if the point is inside any of them
        std::vector<geometry_msgs::Point> region_centers;
        geometry_msgs::Point p1, p2, p3, p4, p5, p6;
        p1.x = -0.5; p1.y = -0.3; p1.z = 0.0; 
        p2.x = -0.5; p2.y = 0.3; p2.z = 0.0;
        p3.x = 0.0; p3.y = -0.5; p3.z = 0.0;
        p4.x = 0.0; p4.y = 0.5; p4.z = 0.0;
        p5.x = 0.5; p5.y = -0.3; p5.z = 0.0;
        p6.x = 0.5; p6.y = 0.3; p6.z = 0.0;

        region_centers.push_back(p1);
        region_centers.push_back(p2);
        region_centers.push_back(p3);
        region_centers.push_back(p4);
        region_centers.push_back(p5);
        region_centers.push_back(p6);

        const double radius = 0.15; // Define the radius of the circular regions

        for (size_t i = 0; i < region_centers.size(); ++i) {
            double distance_squared = pow(point.x - region_centers[i].x, 2) + pow(point.y - region_centers[i].y, 2);
            if (distance_squared <= pow(radius, 2)) {
                return i; // Return the index of the region if the point is inside
            }
        }

        return -1; // Return -1 if the point is not inside any region
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher region_pub;
    visualization_msgs::MarkerArray markerArray;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "six_region_marker");
    CameraPointListener listener;
    ros::spin();

    return 0;
}
