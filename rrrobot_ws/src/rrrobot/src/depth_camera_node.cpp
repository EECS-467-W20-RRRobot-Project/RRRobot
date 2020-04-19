// depth_camera_node.cpp

#include <algorithm>
#include <vector>
#include <string>

#include <ros/ros.h>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "message_filters/subscriber.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>


pcl::PointCloud<pcl::PointXYZ> convert_pc(const sensor_msgs::PointCloud::ConstPtr & msg) {
    // Convert from sensor_msgs/PointCloud.msg to necessary format
    // Convert to PointCloud2
    sensor_msgs::PointCloud2 cloud2 = new sensor_msgs::PointCloud2;
    sensor_msgs::PointCloudToPointCloud2(msg, cloud2); 

    // Convert to PCL data type
    pcl::PointCloud<pcl::PointXYZ> output = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::fromROSMsg(cloud2, output);
}

void depth_camera_callback(const sensor_msgs::PointCloud::ConstPtr & msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud = convert_pc(msg);

    // extract clusters

}

int main(int argc, char ** argv) {
    // TODO: Subscribe to depth camera topic
    // Last argument is the default name of the node.
    ros::init(argc, argv, "depth_camera_node");

    ros::NodeHandle node;
    
    ros::Subscriber sub = node.subscribe("/ariac/depth_camera_1", 1, depth_camera_callback);

    
    
    // TODO: When item is in view, work with point cloud to get location (in world frame) for arm to reach to pickup item
    

    // TODO: Publish object's current location to topic for RRRobot node to listen to
    
}