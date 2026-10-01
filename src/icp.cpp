#include "cube_dc_map/icp.h"
#include "cube_dc_map/icpresult.h"

#include <iostream>
#include <chrono>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Geometry>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/conversions.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/registration/icp.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>


#include <pcl/filters/approximate_voxel_grid.h>

using namespace std;

cube_dc_map::icpresult out;

cube_dc_map::icpresult
getMsgFromM4(Eigen::Matrix4f  m4, double score)
{
    Eigen::Affine3d mer;
    geometry_msgs::TransformStamped transform_msg;
    Eigen::Matrix4d m4d;
    cube_dc_map::icpresult tbp;
    
    m4d = m4.cast<double>();
    mer = m4d;
    transform_msg = tf2::eigenToTransform(mer);
    tbp.header= transform_msg.header;
    tbp.icpscore = score;
    tbp.rotation= transform_msg.transform.rotation;
    tbp.translation= transform_msg.transform.translation;


    return tbp;
}

void icpc::downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    //cout<<"-------Downsampling cloud---------"<<endl;

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approx_vg;
    approx_vg.setLeafSize(_leaf_size, _leaf_size, _leaf_size);
    approx_vg.setInputCloud(in_cloud_ptr);
    approx_vg.filter(*out_cloud_ptr);

    //cout<<"DS Input: "<<in_cloud_ptr->size()<<" pts, DS output: "<<out_cloud_ptr->size()<<" pts"<<endl;

    return;
}


/* @brief Constructor */
icpc::icpc(ros::NodeHandle node, ros::NodeHandle private_nh)
{
    
    icpc::pc_pub = node.advertise<cube_dc_map::icpresult>("icp_transformation",50);
    /*
    private_nh.param("leaf_size", _leaf_size, 0.002);
    private_nh.param("eps_angle", _eps_angle,2);    
    private_nh.param("mean_k", _mean_k, 50);
    private_nh.param("std_mul", _std_mul, 0.01);
    */
    /* Loading parameters */
     private_nh.param("leaf_size", _leaf_size, 0.05);

    private_nh.param("transformation_epsilon", _transformation_epsilon, 1e-8);
    private_nh.param("max_iterations", _max_iters, 100);
    private_nh.param("euclidean_fitness_epsilon", _euclidean_fitness_epsilon, 0.001);
    private_nh.param("max_correspondence_distance", _max_correspondence_distance, 1.0);
 
    private_nh.param<std::string>("totmaprot", _point_cloud_topic, "totmaprot");
    ROS_INFO("point_cloud_topic: %s", _point_cloud_topic.c_str());
    
    pc_sub = node.subscribe("totmaprot", 1, &icpc::cloudCallback,this);


    is_initial = true;
    //icp::pc_pub =  node.advertise<sensor_msgs::PointCloud> ("pctot", 1);
}

/* void icpc::downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    //cout<<"-------Downsampling cloud---------"<<endl;

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approx_vg;
    approx_vg.setLeafSize(_leaf_size, _leaf_size, _leaf_size);
    approx_vg.setInputCloud(in_cloud_ptr);
    approx_vg.filter(*out_cloud_ptr);

    //cout<<"DS Input: "<<in_cloud_ptr->size()<<" pts, DS output: "<<out_cloud_ptr->size()<<" pts"<<endl;

    return;
}
*/
void icpc::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    //DEBUG=ROS_INFO("-------Entered callback---------");
    
    if(is_initial)
    {
	    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
      ROS_INFO("Hi");
        pcl::fromROSMsg(*msg, *prev_cloud_ptr);

        //downsampleCloud(prev_cloud_ptr,filtered_cloud_ptr);
        
        icpc::_prev_cloud = *prev_cloud_ptr;
        cout << "The is_initial bool is " << is_initial << endl;

        is_initial = false;
    }
    else
    {
        Eigen::Matrix4f final_icp_transform;
	    //geometry_msgs::TransformStamped object_msg;
        //sensor_msgs::PointCloud2 object_msg;
        
        //ROS_INFO("");
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>(_prev_cloud));
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        tf::Transform transftf;
        chrono::time_point<chrono::system_clock> start, end;

        pcl::fromROSMsg(*msg, *current_cloud_ptr);

        //downsampleCloud(current_cloud_ptr,current_cloud_ptr);

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setTransformationEpsilon(_transformation_epsilon);
        icp.setMaximumIterations(_max_iters);
        icp.setMaxCorrespondenceDistance(_max_correspondence_distance);
        icp.setEuclideanFitnessEpsilon(_euclidean_fitness_epsilon);

        icp.setInputSource(current_cloud_ptr);
        icp.setInputTarget(prev_cloud_ptr);



        icp.align(*icp_cloud_ptr);

        end = chrono::system_clock::now();

        //cout<<"-------Matching done---------"<<endl;

        cout << "ICP has converged:" << icp.hasConverged()
            << " score: " << icp.getFitnessScore() << endl;
       //icp.getFinalTransformation
       _prev_cloud = *icp_cloud_ptr;
       final_icp_transform = icp.getFinalTransformation();

        ::out = getMsgFromM4(final_icp_transform,icp.getFitnessScore());
        
        
       pc_pub.publish(::out); //publishing the current pose

    }

    return;
}
 