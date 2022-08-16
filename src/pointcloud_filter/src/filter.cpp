
#include "pcl_ros/point_cloud.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <limits>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


using namespace std;
ros::Publisher markerPub;
ros::Publisher Pub;

// Subscriber callback for the incoming pointcloud
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
	
	
	// convert pointcloud from message to pcl
	pcl::PointCloud<pcl::PointXYZ>::Ptr input1_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*input, *input1_cloud);
	// Transforming pointcloud to correct orientation (orientation is wrong due to unknown reason)
	Eigen::Matrix4f transform = Eigen::Matrix4f::Zero();
	transform (0,2) = 1;
  	transform (1,0) = -1;
  	transform (2,1) = -1;
  	transform (3,3) = 1;
	pcl::transformPointCloud (*input1_cloud, *transform_cloud, transform);

	// Create voxel filter object to reduce number of points in pointcloud to reduce computational load
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::VoxelGrid<pcl::PointXYZ> sor;
  	sor.setInputCloud (transform_cloud);
	sor.setLeafSize (0.03f, 0.03f, 0.03f);
  	sor.filter (*input_cloud);

	// Filter out the ground floor
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	for (int i = 0; i < (*input_cloud).size(); i++)
	{
		pcl::PointXYZ pt(input_cloud->points[i].x, input_cloud->points[i].y, input_cloud->points[i].z);
		float zAvg = 0.1f;
		if (pt.z < zAvg) // e.g. remove all pts below zAvg
		{
			inliers->indices.push_back(i);
		}
	}
	extract.setInputCloud(input_cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*input_cloud);
	
	
	// Creating the KdTree from input point cloud
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(input_cloud);

	// Setup to extract clusters from pointcloud
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.2);
	ec.setMinClusterSize(30);
	ec.setMaxClusterSize(100000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(input_cloud);

	// Extract the clusters out of point cloud and save indices in cluster_indices.
	ec.extract(cluster_indices);

	// Iterators for two loops
	std::vector<pcl::PointIndices>::const_iterator it;
	std::vector<int>::const_iterator pit;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
	
	// Setup Marker array for the detections
	visualization_msgs::MarkerArray clusterMarkers;

	// Start iterating through clusters to find the best fit for apriltag cluster
	int i = 0;
	for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) 
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		float x = 0.0;
		float y = 0.0;
		float z = 0.0;
		int numPts = 0;
		for (pit = it->indices.begin(); pit != it->indices.end(); pit++) 
		{
		cloud_cluster->points.push_back(input_cloud->points[*pit]);
		x += input_cloud->points[*pit].x;
		y += input_cloud->points[*pit].y;
		z += input_cloud->points[*pit].z;
		numPts++;
		}

		// Create the marker for the object
		visualization_msgs::Marker m;
		m.id = i;
		i++;
		m.type = visualization_msgs::Marker::CUBE;
		m.header.frame_id = "base_link";
		m.action = visualization_msgs::Marker::ADD;
		m.color.a = 1.0;
		m.color.r = i % 2 ? 1 : 0;
		m.color.g = i % 3 ? 1 : 0;
		m.color.b = i % 4 ? 1 : 0;
		m.pose.position.x = x / numPts;
		m.pose.position.y = y / numPts;
		m.pose.position.z = z / numPts;
		m.scale.x = 0.2;
		m.scale.y = 0.2;
		m.scale.z = 0.2;

		clusterMarkers.markers.push_back(m);
		
	}

	// Publish the markers for the objects
	markerPub.publish(clusterMarkers);
	
	/*
	// For debugging purposes
	sensor_msgs::PointCloud2::Ptr clustermsg(new sensor_msgs::PointCloud2);
	pcl::toROSMsg(*input_cloud, *clustermsg);
	clustermsg->header.frame_id = "base_link";
	clustermsg->header.stamp = ros::Time::now();
	Pub.publish(clustermsg);
	*/
	
}


int main(int argc, char **argv) {
	// ROS init
	ros::init(argc, argv, "pointcloud_filter");
	ros::NodeHandle nh;

	cout << "Setup Callback\n";

	// Create a ROS publisher for the detected objects as Markers
	markerPub = nh.advertise<visualization_msgs::MarkerArray>("filtered_markers", 1);
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe("camera/depth/color/points", 1, cloud_cb);

	// For debugging the pointcloud
	Pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered",1);

	ros::spin();
}
