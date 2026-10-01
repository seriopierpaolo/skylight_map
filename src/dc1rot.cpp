#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>

#include <vector>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>

// PCL specific includes

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/crop_box.h>

Eigen::Vector3f translvec(0, 0, 0); //translation

std::vector<float> rotzyx {0, 3.14159265359, -2.35619449019};     //Euler angles for rotation (ZYX)

using std::vector;

ros::Publisher pub;

//From Euler angles to Rotation Matrix
Eigen::Matrix3f eulAng2Matrix (std::vector<float> angles){

Eigen::AngleAxisf rollAngle(angles[0], Eigen::Vector3f::UnitZ());
Eigen::AngleAxisf yawAngle(angles[1], Eigen::Vector3f::UnitY());
Eigen::AngleAxisf pitchAngle(angles[2], Eigen::Vector3f::UnitX());

Eigen::Quaternion<float> q = rollAngle * yawAngle * pitchAngle;

Eigen::Matrix3f rotationMatrix = q.matrix();

return rotationMatrix;

}



Eigen::Matrix4f getOmogMat (Eigen::Matrix3f rotm, Eigen::Vector3f traslm){

Eigen::Matrix4f Trans; // Your Transformation Matrix
Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
Trans.block<3,3>(0,0) = rotm;
Trans.block<3,1>(0,3) = traslm;
return Trans;

}

sensor_msgs::PointCloud2
cropBox(const sensor_msgs::PointCloud2ConstPtr& msg)
{

sensor_msgs::PointCloud2 out_msg;
pcl::PCLPointCloud2 pcl_pc2;

    pcl_conversions::toPCL(*msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*pcl_input);

    // set up cropbox


    // apply cropping
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(-5, -5, -5, 1.0));
    boxFilter.setMax(Eigen::Vector4f(5, 5, 5, 1.0));
    boxFilter.setInputCloud(pcl_input);
    //boxFilter.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cropped(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl_cropped = msg->header.frame_id;
    boxFilter.filter(*pcl_input);


    // publish sensor_msgs pcl 2
    sensor_msgs::PointCloud2 msg_pcl2_cropped;
     msg_pcl2_cropped.header = msg->header;
    pcl::toROSMsg(*pcl_input, out_msg);
    out_msg.header = msg->header;
    std::cout << out_msg.header<< std::endl;
    //pub.publish(msg_pcl2_cropped);


  return out_msg;
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 out_crop;
  sensor_msgs::PointCloud2* cloudrotated = new sensor_msgs::PointCloud2;
  
  //Homogenenous transformation matrix
  Eigen::Matrix3f rotmat = eulAng2Matrix(rotzyx);
  Eigen::Matrix4f hotr = getOmogMat(rotmat, translvec);

  out_crop = cropBox(input);
  //Applying transformation
   pcl_ros::transformPointCloud(hotr, out_crop, *cloudrotated);
  // Publish the data.
   pub.publish (*cloudrotated);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "dc1rot");
  ros::NodeHandle nh;


// Create a ROS subscriber for the depthcamera 1
  ros::Subscriber sub = nh.subscribe ("/d435_1/depth/color/points", 100, cloud_cb);



  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("pc1rotated", 100);

  // Spin
  ros::spin ();
}