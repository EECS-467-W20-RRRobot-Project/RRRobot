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


void depth_camera_callback(const sensor_msgs::PointCloud::ConstPtr & cloud_msg) {
    // convert from PointCloud to PointCloud2
	sensor_msgs::PointCloud2 cloud2;
	sensor_msgs::convertPointCloudToPointCloud2(*cloud_msg, cloud2);	

	// Container for original & filtered data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// Convert to PCL data type
	pcl_conversions::toPCL(cloud2, *cloud);

	// Perform the actual filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (0.02, 0.02, 0.02);
	sor.filter (cloud_filtered);

	pcl::PointCloud<pcl::PointXYZ> point_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2( cloud_filtered, point_cloud);
	pcl::copyPointCloud(point_cloud, *point_cloudPtr);

    // Create the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(point_cloudPtr);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.1); // 2cm
	ec.setMinClusterSize(10); //100
	ec.setMaxClusterSize(99000000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(point_cloudPtr);
	ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_segmented(new pcl::PointCloud<pcl::PointXYZRGB>);

  int j= 0;
 
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	  {
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		    {
                pcl::PointXYZRGB point;
                point.x = point_cloudPtr->points[*pit].x;
                point.y = point_cloudPtr->points[*pit].y;
                point.z = point_cloudPtr->points[*pit].z;

                if (j == 0) //Red	#FF0000	(255,0,0)
			     {
				      point.r = 0;
				      point.g = 0;
				      point.b = 255;
			     }
			    else if (j == 1) //Lime	#00FF00	(0,255,0)
			     {
				      point.r = 0;
				      point.g = 255;
				      point.b = 0;
			     }
			    else if (j == 2) // Blue	#0000FF	(0,0,255)
			     {
				      point.r = 255;
				      point.g = 0;
				      point.b = 0;
			     }
			    else
		   	     {
				      if (j % 2 == 0)
				       {
					        point.r = 255 * j / (cluster_indices.size());
					        point.g = 128;
					        point.b = 50;
				       }
				      else
				       {
					        point.r = 0;
					        point.g = 255 * j / (cluster_indices.size());
					        point.b = 128;
				       }
                 }
                point_cloud_segmented->push_back(point);
			
            }
        j++;
    }
  
    // Convert to ROS data type
    point_cloud_segmented->header.frame_id = point_cloudPtr->header.frame_id;
    if(point_cloud_segmented->size()) pcl::toPCLPointCloud2(*point_cloud_segmented, cloud_filtered);
    else pcl::toPCLPointCloud2(*point_cloudPtr, cloud_filtered);
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);

    // Publish the data
    pub.publish (output);
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