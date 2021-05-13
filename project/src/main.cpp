#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// from website
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>

// from reys website
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// from harts website
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub;

void 
cloud_cb (const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
    pcl::PCLPointCloud2 cloud;
    pcl_conversions::toPCL(*input, cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud,*temp_cloud);


    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::SACSegmentation<pcl::PointXYZ> segCyl;
    // Optional
    seg.setOptimizeCoefficients (true);
    segCyl.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    segCyl.setModelType (pcl::SACMODEL_CYLINDER);
    segCyl.setMethodType (pcl::SAC_RANSAC);
    segCyl.setDistanceThreshold (0.01)

    seg.setInputCloud (temp_cloud);
    seg.segment (*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    pcl::PointCloud<pcl::PointXYZ>::Ptr table_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    extract.setInputCloud (temp_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false); 
    extract.filter (*table_cloud);

    // Create a container for the data.
    pcl::PCLPointCloud2 outputInProgress;
    sensor_msgs::PointCloud2 output;
    
    pcl::toPCLPointCloud2(*table_cloud, outputInProgress);
    pcl_conversions::fromPCL(outputInProgress, output);
    // pcl_conversions::moveFromPCL(table_cloud, output);


  // Do data processing here...
  //   output = *input;
  ROS_INFO("Cloud callback received");

  // Publish the data.
  pub.publish (output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1, cloud_cb);
  
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("table", 1);

  // Spin
  ros::spin ();
}

