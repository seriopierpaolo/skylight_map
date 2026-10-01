#include <ros/ros.h>
#include "cube_dc_map/icpresult.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <vector>
//#include <Eigen/Geometry>
//#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
//#include <Eigen/Core>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher pub;

tf2_ros::Buffer tf_buffer;

geometry_msgs::TransformStamped vslam_transform;

geometry_msgs::TransformStamped res;

float t;

geometry_msgs::Quaternion
interpolatevSlamIcp(cube_dc_map::icpresult icpt)
{
 ::t = 0.5;
  
 geometry_msgs::Quaternion out, qgslam;
 Eigen::Quaterniond qevslam;
 geometry_msgs::Quaternion qgvicp = icpt.rotation;
 Eigen::Quaterniond qevicp;
 //Eigen::Quaterniond qevicpd;
 tf2::fromMsg(qgvicp, qevicp);
 //Eigen::Quaternionf qevicp(icp_transform.block<3,3>(0,0)); //This extracts rotation matrix
  
qevicp.slerp(t,qevslam);
//qevicpd = qevicp.cast<double>();
tf::quaternionEigenToMsg (qevicp, out);


return out;

 
}

geometry_msgs::Vector3  
combinetransl(cube_dc_map::icpresult icpt)
{
  Eigen::Vector3d sum;
  Eigen::Vector3d sumd;
  geometry_msgs::Vector3 comb_transl;
  float t = 0.5;
  Eigen::Vector3d icpet, vslamet; 
  tf2::fromMsg(icpt.translation, icpet);
  tf2::fromMsg(::vslam_transform.transform.translation, vslamet); 

  sum = (t*icpet) + ((1-t)*vslamet);

  //sumd = sum.cast<double>();
  tf2::toMsg(sum,comb_transl);


  return comb_transl;
}

void
icpTransformCB(cube_dc_map::icpresult data)
{
  geometry_msgs::Quaternion quat; 
  geometry_msgs::Vector3 transl;

  quat = interpolatevSlamIcp(data);
  transl = combinetransl(data);
  ::res.header = data.header;
  ::res.transform.rotation = quat;
  ::res.transform.translation = transl;
  return;

}
int main(int argc, char **argv){

  ros::init(argc, argv, "tf_blending");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Subscriber icp_sub = nh.subscribe("icp_transformation", 10, icpTransformCB); 
  pub = nh.advertise<geometry_msgs::TransformStamped> ("transform_slerp", 1);
  
  tf2_ros::TransformListener tf2_listener(tf_buffer);

  ros::Rate loop_rate(10);

  while(ros::ok())
    {
        ros::Time t = ros::Time(0);
        tf_buffer.setUsingDedicatedThread(true);
         tf_buffer.canTransform("map","t265_pose_frame", t, ros::Duration(10.0));
        ::vslam_transform = tf_buffer.lookupTransform("map","t265_pose_frame",  ros::Time(0));

        ::pub.publish(::res);
        ros::spinOnce(); //invokes callback
        loop_rate.sleep();
    }
  return 0;
}
