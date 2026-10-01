#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pc_filters/pc_filtering.h>
#include <string>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/common/projection_matrix.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pc_filtering.h>
void 
downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    //cout<<"-------Downsampling cloud---------"<<endl;
    
    double leaf_size = 0.3;
    
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approx_vg;
    approx_vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    approx_vg.setInputCloud(in_cloud_ptr);
    approx_vg.filter(*out_cloud_ptr);

    //cout<<"DS Input: "<<in_cloud_ptr->size()<<" pts, DS output: "<<out_cloud_ptr->size()<<" pts"<<endl;

    return;
}


void 
passThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
    //cout<<"-------Downsampling cloud---------"<<endl;
    
 pcl::PassThrough< pcl::PointXYZ > ptfilter (true); // Initializing with true will allow us to extract the removed indices
ptfilter.setInputCloud (cloud_in);
ptfilter.setFilterFieldName ("x");
ptfilter.setFilterLimits (0.0, 1000.0);

// The indices_rem array indexes all points of cloud_in that have x smaller than 0.0 or larger than 1000.0
// and also indexes all non-finite points of cloud_in

ptfilter.setFilterFieldName ("z");
ptfilter.setFilterLimits (-10.0, 10.0);
ptfilter.setNegative (true);

// The indices_xz array indexes all points of cloud_in that have x between 0.0 and 1000.0 and z larger than 10.0 or smaller than -10.0

ptfilter.setFilterFieldName ("intensity");
ptfilter.setFilterLimits (std::numeric_limits<float>::lowest(), 0.5);
ptfilter.setNegative (false);
ptfilter.filter (*cloud_out);
// The resulting cloud_out contains all points of cloud_in that are finite and have:
// x between 0.0 and 1000.0, z larger than 10.0 or smaller than -10.0 and intensity smaller than 0.5.
    return;
}

void
radiusOutlierRemoval (const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ> cloud_out)
{
 pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud_in);
    outrem.setRadiusSearch(0.3);
    outrem.setMinNeighborsInRadius (10);
    outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter (cloud_out); 
}

