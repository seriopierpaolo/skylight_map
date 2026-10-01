#include "cube_dc_map/icp.h"
#include <ros/ros.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "icp_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    icpc icp(node, private_nh); //instance of icp class

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        
        ros::spinOnce(); //invokes callback
        loop_rate.sleep();
    }

    //ros::spin();

    return 0;
}