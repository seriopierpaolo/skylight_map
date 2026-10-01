#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <string>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

class icpc
{
    public:
        icpc(ros::NodeHandle node, ros::NodeHandle private_nh);
        ~icpc() {};

    private:
        void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg); //point cloud callback
        
        
        void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr); //downsampling the point cloud using Voxelgrid
        
        ros::Subscriber pc_sub; //point cloud subscriber
        ros::Publisher pc_pub;
        

        pcl::PointCloud<pcl::PointXYZ> _prev_cloud; //point cloud at the previous time instance
        
    
        /*---------sub-pub parameters----------*/
        
        std::string _point_cloud_topic; //point cloud ros topic to subscribe to
        bool is_initial;

        /*----------ICP parameters------------*/

        //double _minX, _maxX, _minY, _maxY, _minZ, _maxZ; //min and max pts for box filter
        double _leaf_size; //leaf size for voxel grid
        //int _mean_k; //number of neighbors to analyze for each point 
        //double _std_mul; //standard deviation multiplication threshold 
        //int _eps_angle; //allowed difference of angles in degrees for perpendicular plane model
        double _transformation_epsilon; //minimum transformation difference for termination condition
        int _max_iters; //max number of registration iterations
        double _euclidean_fitness_epsilon; //maximum allowed Euclidean error between two consecutive steps in the ICP loop
        double _max_correspondence_distance; //correspondences with higher distances will be ignored

};
