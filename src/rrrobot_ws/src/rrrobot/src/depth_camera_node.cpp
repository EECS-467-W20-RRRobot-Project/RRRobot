// depth_camera_node.cpp

#include <algorithm>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include "topic_names.h"

ros::Publisher pub;

void depth_camera_callback(const sensor_msgs::PointCloud::ConstPtr &cloud_msg)
{
	// define camera pose
	float cam_x = 1.0;
	float cam_y = 1.2;
	float cam_z = 1.4;
	// rpy = 0 1.21 0

	// convert from PointCloud to PointCloud2
	sensor_msgs::PointCloud2 cloud2;
	sensor_msgs::convertPointCloudToPointCloud2(*cloud_msg, cloud2);

	// transform to world frame
	sensor_msgs::PointCloud2 worldCloud;

	tf::Transform xform;
	xform.setOrigin(tf::Vector3(cam_x, cam_y, cam_z));
	xform.setRotation(tf::Quaternion(0, 0.5697, 0, 0.82185));

	pcl_ros::transformPointCloud("world", xform, cloud2, worldCloud);

	// Container for original & filtered data
	pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// Convert to PCL data type
	pcl_conversions::toPCL(worldCloud, *cloud);

	// Perform the actual filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloudPtr);
	sor.setLeafSize(0.001, 0.001, 0.001);
	sor.filter(cloud_filtered);

	pcl::PointCloud<pcl::PointXYZ> point_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(cloud_filtered, point_cloud);
	pcl::copyPointCloud(point_cloud, *point_cloudPtr);

	// Filter out points on surface of belt
	for (std::size_t i = 0; i < point_cloudPtr->points.size(); i++)
	{
		//std::cout << point_cloudPtr->points[i].x << ", " << point_cloudPtr->points[i].y << ", " << point_cloudPtr->points[i].z << std::endl;
		if (point_cloudPtr->points[i].z < 0.93)
		{
			point_cloudPtr->points[i].x = 0;
			point_cloudPtr->points[i].y = 0;
			point_cloudPtr->points[i].z = 0;
		}
	}

	// Create the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(point_cloudPtr);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.05); // 5cm
	ec.setMinClusterSize(10);	  //10
	ec.setMaxClusterSize(99000000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(point_cloudPtr);
	ec.extract(cluster_indices);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_segmented(new pcl::PointCloud<pcl::PointXYZRGB>);

	int j = 0;

	// store index of object cluster
	int obj_idx = -1;

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		// average all points
		int s = 0;
		float x = 0;
		float y = 0;
		float z = 0;
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			// convert
			pcl::PointXYZRGB point;
			point.x = point_cloudPtr->points[*pit].x;
			point.y = point_cloudPtr->points[*pit].y;
			point.z = point_cloudPtr->points[*pit].z;

			// add to running totals
			s++;
			x += point.x;
			y += point.y;
			z += point.z;

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
		// calculate center of cluster
		x /= s;
		y /= s;
		z /= s;

		// print cluster center
		std::cout << "cluster at: " << x << ", " << y << ", " << z << std::endl;

		// check if center is within box
		if (x < 1.25 && x > 1.15 && y < cam_y + .05 && y > cam_y - .05)
		{
			// object detected
			std::cout << "object in position" << std::endl;
			obj_idx = j;
		}

		j++;
	}

	// identify surfaces of object cluster
	if (obj_idx != -1)
	{
		// create new cloud containing only object cluster
		pcl::PointCloud<pcl::PointXYZ> obj_cloud;
		std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin() + obj_idx;
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			pcl::PointXYZ point;
			point.x = point_cloudPtr->points[*pit].x;
			point.y = point_cloudPtr->points[*pit].y;
			point.z = point_cloudPtr->points[*pit].z;
			obj_cloud.push_back(point);
		}

		std::cout << "object cluster size: " << obj_cloud.points.size() << std::endl;

		// create 3D bounding box.
		float xmin = 10.0;
		float ymin = 10.0;
		float zmin = 0.92;
		float xmax = 0;
		float ymax = 0;
		float zmax = 0;

		for (int i = 0; i < obj_cloud.points.size(); ++i)
		{
			xmin = xmin < obj_cloud.points[i].x ? xmin : obj_cloud.points[i].x;
			ymin = ymin < obj_cloud.points[i].y ? ymin : obj_cloud.points[i].y;
			//zmin = zmin < obj_cloud.points[i].z ? zmin : obj_cloud.points[i].z;
			xmax = xmax > obj_cloud.points[i].x ? xmax : obj_cloud.points[i].x;
			ymax = ymax > obj_cloud.points[i].y ? ymax : obj_cloud.points[i].y;
			zmax = zmax > obj_cloud.points[i].z ? zmax : obj_cloud.points[i].z;
		}

		// determine target pose
		float area_top = (ymax - ymin) * (xmax - xmin);
		float area_side = (zmax - zmin) * (ymax - ymin);

		std::cout << "object top area: " << area_top << std::endl;
		std::cout << "object side area: " << area_side << std::endl;

		geometry_msgs::Pose pose;
		geometry_msgs::Point p;
		geometry_msgs::Quaternion q;

		// pick up from top (-z direction)
		if (area_top > .002)
		{
			std::cout << "picking up object from top" << std::endl;
			p.x = (xmin + xmax) / 2;
			p.y = (ymin + ymax) / 2;
			p.z = zmax;

			// rpy = 0 0 0
			q.x = 0;
			q.y = 0.707;
			q.z = 0;
			q.w = 0.707;
		}
		else
		{
			// pick up from front (+x direction)
			std::cout << "picking up object from front" << std::endl;
			p.x = xmin;
			p.y = (ymin + ymax) / 2;
			p.z = (zmax + zmin) / 2;

			// rpy = -pi/2 0 pi/2
			q.x = 0.707;
			q.y = 0.0;
			q.z = 0.0;
			q.w = 0.707;
		}

		pose.position = p;
		pose.orientation = q;

		// publish pose
		pub.publish(pose);
	}
}

int main(int argc, char **argv)
{
	// Last argument is the default name of the node.
	ros::init(argc, argv, "depth_camera_node");

	ros::NodeHandle node;

	// Subscribe to depth camera topic
	ros::Subscriber sub = node.subscribe("/ariac/depth_camera_1", 1, depth_camera_callback);

	// Publish object's current location to topic for RRRobot node to listen to
	pub = node.advertise<geometry_msgs::Pose>(DESIRED_GRASP_POSE_CHANNEL, 1);

	ros::spin();
}
