#include <ros/ros.h>
#include <ros/console.h>

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <fstream>      // std::ofstream
#include <ctime>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 

#include <eigen_conversions/eigen_msg.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

using namespace std;

// TODO: this shouldn't be defined here:
namespace pcl
{
  // Euclidean Velodyne coordinate, including intensity and ring number. 
  struct PointXYZIR
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    std::uint16_t ring;                 ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

};

class Pass{
  public:
    Pass(ros::NodeHandle node_);
    
    ~Pass(){
    }    
  private:
    ros::NodeHandle node_;

    ros::Subscriber collectSub_;
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& msg);

    double voxelGridResolution_;
};


Pass::Pass(ros::NodeHandle node_){

  std::string node_name = ros::this_node::getName();
  ROS_INFO("Node name: %s", node_name.c_str() );

  collectSub_     = node_.subscribe(std::string("/lidar/point_cloud"), 100, &Pass::cloudHandler, this);

  voxelGridResolution_ = 0.01;
  if (!node_.getParam("voxel_grid_resolution", voxelGridResolution_)) {
    ROS_ERROR("Could not read parameter `voxel_grid_resolution`.");
    exit(-1);
  }
  ROS_INFO("Voxel Grid resolution: %f", voxelGridResolution_);



}


// x, y, z, intensity, ring
void Pass::cloudHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
  double time_stamp = double(msg->header.stamp.sec) + double(msg->header.stamp.nsec)*1e-9;


  // 1. Collect the Scan
  ROS_INFO("Collect map command received. Collected seconds.");

  pcl::PCLPointCloud2* pcl_pc2 = new pcl::PCLPointCloud2; 
  pcl_conversions::toPCL(*msg,*pcl_pc2);

  for (size_t i=0 ; i < pcl_pc2->fields.size() ; i++)
    std::cout << pcl_pc2->fields[i].name << ", ";
  std::cout << " are the "<< pcl_pc2->fields.size() <<" fields\n";


  //pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZIR>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(*pcl_pc2,*cloud_filtered);


  // 5 Write point cloud to file
  std::stringstream ss_pcd_filename;
  ss_pcd_filename << "cloud_" << time_stamp << ".pcd";
  pcl::io::savePCDFileASCII (ss_pcd_filename.str(), *cloud_filtered);
  std::cout << "Saved " << cloud_filtered->points.size () << " data points to "<< ss_pcd_filename.str() << std::endl;


  // 3 Voxel Filter
  // ROS_INFO("Voxel Filter (%fcm)...", voxelGridResolution_);
  // pcl::VoxelGrid<pcl::PointXYZ> sor;
  // sor.setInputCloud (temp_cloud);
  // sor.setLeafSize (voxelGridResolution_, voxelGridResolution_, voxelGridResolution_);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  // sor.filter (*cloud_filtered);


}


int main( int argc, char** argv ){
  ros::init(argc, argv, "point_cloud_collector");
 
  ros::NodeHandle nh("~");

  Pass app(nh);
  cout << "Ready with PointCloudCollector" << endl << "============================" << endl;
  ROS_INFO_STREAM("PointCloudCollector ready");
  ROS_ERROR_STREAM("PointCloudCollector ready");
  ros::spin();
  return 0;
}
