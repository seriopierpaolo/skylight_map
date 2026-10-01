#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/common/io.h> // for concatenateFields
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
 

ros::Publisher pub;

pcl::PCLPointCloud2 out1;
pcl::PCLPointCloud2 out2;
pcl::PCLPointCloud2 mc;

sensor_msgs::PointCloud2 tbp;

// Callback for pointcloud from depth camera 1
void
cloud_cb1 (const sensor_msgs::PointCloud2ConstPtr& input)
{  
  pcl_conversions::toPCL(*input,::out1);
}

// Callback for pointcloud from depth camera 1
void
cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr&  input)
{
  pcl_conversions::toPCL(*input,::out2);
}

//Concatenate the two pointclouds
sensor_msgs::PointCloud2
pub_mm ()
{
  pcl::concatenate(::out1, ::out2, ::mc);
  
  pcl_conversions::fromPCL(::mc,::tbp);

  return ::tbp;
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "conc2pcbm");
  ros::NodeHandle nh;

// Create a ROS subscriber for the rotated depthcamera 1 (dc1rot node must be running)
  ros::Subscriber sub1 = nh.subscribe ("pc1rotated", 1, cloud_cb1);
///d435_1/depth/color/points for raw dc data, pc1rotated for rotated pc

// Create a ROS subscriber for the rotated depthcamera 2 (dc2rot node must be running)
  ros::Subscriber sub2 = nh.subscribe ("pc2rotated", 1, cloud_cb2);
///d435_2/depth/color/points for raw dc data, pc2rotated for rotated pc

// Create a ROS publisher for the merged pointcloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("merged_pc", 1);
 
  ros::Rate r(1000);
  while (ros::ok()){
    tbp = pub_mm();
    pub.publish(tbp);
    ros::spinOnce();
    r.sleep();
  }

}