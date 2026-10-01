//#include <cube_dc_map/lbmap.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>

// PCL filters includes
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>


tf2_ros::Buffer tf_buffer;

geometry_msgs::TransformStamped transform;

pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> tot_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr tot_cloud_ptr (&tot_cloud);
sensor_msgs::PointCloud2 object_msg;

bool is_initial = true;

ros::Publisher pub;

//tf::StampedTransform transform;


//-----------------------------------------FILTERS--------------------------------------------//
void 
cropBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZ> out_cloud)
{
 Eigen::Affine3d croprot;
 geometry_msgs::Transform trasfs;
 trasfs = ::transform.transform;

//tf::transformMsgToEigen(trasfs, croprot);
 
 pcl::CropBox<pcl::PointXYZ> cropBoxFilter (true);
 cropBoxFilter.setInputCloud (in_cloud);
 Eigen::Vector4f min_pt (-3.0, -3.0, -3.0, 1);
 Eigen::Vector4f max_pt (3.0, 3.0, 3.0, 1);

 // Cropbox slighlty bigger then bounding box of points
 cropBoxFilter.setMin (min_pt);
 cropBoxFilter.setMax (max_pt);
 //tf::transformMsgToEigen(trasfs, croprot);
 //cropBoxFilter.setTransform(croprot.);
 
 // Cloud
 cropBoxFilter.filter (out_cloud);
  return;
}


void
statisticalOutlierRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZ> out_cloud)
{
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (in_cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (out_cloud);
}

void 
downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    //cout<<"-------Downsampling cloud---------"<<endl;
    
    double leaf_size = 0.09;
    
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approx_vg;
    approx_vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    approx_vg.setInputCloud(in_cloud_ptr);
    approx_vg.filter(*out_cloud_ptr);

    //cout<<"DS Input: "<<in_cloud_ptr->size()<<" pts, DS output: "<<out_cloud_ptr->size()<<" pts"<<endl;

    return;
}


void
radiusOutlierRemoval (const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ> cloud_out)
{
 pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud_in);
    outrem.setRadiusSearch(0.3);
    outrem.setMinNeighborsInRadius (5);
    outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter (cloud_out); 
}
//--------------------------------------END OF FILTERS-------------------------------------------//


pcl::PointCloud<pcl::PointXYZ>

rotPC ( pcl::PointCloud<pcl::PointXYZ> pc, geometry_msgs::TransformStamped trnsf)
{
    
  pcl::PointCloud<pcl::PointXYZ> tmp;

      
      pcl_ros::transformPointCloud(pc,tmp, trnsf.transform);
    

      ROS_INFO("-------Entered rotpcl---------");
      return tmp;
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ> temp_pc;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_dwn_pc_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> rotatedPC;

    pcl::fromROSMsg(*input,*tmp_dwn_pc_ptr);
//---------------------------------FILTERING PHASE---------------------------------//

//tmp_dwn_pc_ptr->points.resize (tmp_dwn_pc_ptr->width * tmp_dwn_pc_ptr->height);
    //passThroughFilter (tmp_dwn_pc_ptr,*tmp_dwn_pc_ptr);
    
    //cropBox(tmp_dwn_pc_ptr,*tmp_dwn_pc_ptr);
    downsampleCloud(tmp_dwn_pc_ptr,tmp_dwn_pc_ptr);
    radiusOutlierRemoval(tmp_dwn_pc_ptr,*tmp_dwn_pc_ptr);
    
    //statisticalOutlierRemoval (tmp_dwn_pc_ptr,*tmp_dwn_pc_ptr);
//------------------------------END OF FILTERING PHASE------------------------------//

    ::tot_cloud.header.frame_id = "map";
    temp_pc = *tmp_dwn_pc_ptr;

    rotatedPC = rotPC(temp_pc,::transform);

    ::tot_cloud = rotatedPC;

    //cropBox(::tot_cloud_ptr,::tot_cloud);
    //std::cout << tot_cloud.size() << std::endl;
    //pcl::concatenate(temp_pc_ptr, *::tot_cloud_ptr, *::tot_cloud_tr);
    
   
    pcl::toROSMsg(::tot_cloud,::object_msg);
}


 int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "lbmerge");
  ros::NodeHandle node;

  //tf::TransformListener *tf_ = new tf::TransformListener;
  
  ros::Subscriber sub = node.subscribe ("merged_pc", 1, cloud_cb);
  pub = node.advertise<sensor_msgs::PointCloud2> ("totmaprot", 1);
  //pub = node.advertise<sensor_msgs::PointCloud2> ("totmapnotrot", 1);

  tf2_ros::TransformListener tf2_listener(tf_buffer);
  //t265_to_map_tf = tf_buffer.lookupTransform(<origin_frame>, <target_frame>, ros::Time(0));
  ros::Time t = ros::Time(0);
  
 
  
  //pub.publish(::object_msg);

    ros::Rate loop_rate(1000);

    while(ros::ok())
    {
        ros::Time t = ros::Time(0);
        //tf_->waitForTransform("map", "t265_pose_frame",  t, ros::Duration(10.0));
        //tf_->lookupTransform("map", "t265_pose_frame",  t, transform);
        
        //std::cout << transform << std::endl;
        
         tf_buffer.canTransform("map","t265_pose_frame", t, ros::Duration(10.0));
        ::transform = tf_buffer.lookupTransform("map","t265_pose_frame",  ros::Time(0));
        pub.publish(::object_msg);

        ros::spinOnce(); //invokes callback
        loop_rate.sleep();
    }

    //ros::spin();

    return 0;
}