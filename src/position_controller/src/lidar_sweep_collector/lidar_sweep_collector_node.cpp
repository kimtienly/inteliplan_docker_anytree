#include <ros/ros.h>
#include <ros/console.h>

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <fstream>      // std::ofstream
#include <ctime>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

#include <eigen_conversions/eigen_msg.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>


#include <laser_assembler/AssembleScans2.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

using namespace laser_assembler;

using namespace std;

class Pass{
  public:
    Pass(ros::NodeHandle node_);
    
    ~Pass(){
    }    
  private:
    ros::NodeHandle node_;

    ros::Subscriber collectSub_;
    void collectNewSweepHandler(const std_msgs::Int16ConstPtr& msg);
    std::string outputDirectory_;

    tf::TransformListener* listener_;

    bool initialized_;
    Eigen::Isometry3d prev_pose_robot_;
    std::ofstream outputPosesFile_;

    double voxelGridResolution_;
};


std::string getTimeAsString(){
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];
  time (&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(buffer,sizeof(buffer),"%Y-%m-%d-%H-%M-%S",timeinfo);
  std::string str(buffer);
  return str;
}


Eigen::Quaterniond euler_to_quat(double roll, double pitch, double yaw) {
  
  // This conversion function introduces a NaN in Eigen Rotations when:
  // roll == pi , pitch,yaw =0    ... or other combinations.
  // cos(pi) ~=0 but not exactly 0 
  // Post DRC Trails: replace these with Eigen's own conversions
  if ( ((roll==M_PI) && (pitch ==0)) && (yaw ==0)){
    return  Eigen::Quaterniond(0,1,0,0);
  }else if( ((pitch==M_PI) && (roll ==0)) && (yaw ==0)){
    return  Eigen::Quaterniond(0,0,1,0);
  }else if( ((yaw==M_PI) && (roll ==0)) && (pitch ==0)){
    return  Eigen::Quaterniond(0,0,0,1);
  }
  
  double sy = sin(yaw*0.5);
  double cy = cos(yaw*0.5);
  double sp = sin(pitch*0.5);
  double cp = cos(pitch*0.5);
  double sr = sin(roll*0.5);
  double cr = cos(roll*0.5);
  double w = cr*cp*cy + sr*sp*sy;
  double x = sr*cp*cy - cr*sp*sy;
  double y = cr*sp*cy + sr*cp*sy;
  double z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaterniond(w,x,y,z);
}



// http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1301
void quat_to_euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}



Pass::Pass(ros::NodeHandle node_){

  std::string node_name = ros::this_node::getName();
  ROS_INFO("Node name: %s", node_name.c_str() );

  collectSub_     = node_.subscribe(std::string("/collect_lidar_sweep_command"), 100, &Pass::collectNewSweepHandler, this);

  voxelGridResolution_ = 0.01;
  if (!node_.getParam("voxel_grid_resolution", voxelGridResolution_)) {
    ROS_ERROR("Could not read parameter `voxel_grid_resolution`.");
    exit(-1);
  }
  ROS_INFO("Voxel Grid resolution: %f", voxelGridResolution_);

  listener_ = new tf::TransformListener();

  char const* home = getenv("HOME");
  std::stringstream ss;
  ss << home << "/drs_maps/" << getTimeAsString();
  outputDirectory_ = ss.str();

  std::cout << "Create output Directory: " << outputDirectory_ << "\n";
  bool success = boost::filesystem::create_directories(outputDirectory_);
  std::cout << "Success? " << success << "\n";
  if (!success)
    exit(-1);

  initialized_ = false;
  std::stringstream ss2;
  ss2 << outputDirectory_ << "/relative_poses.csv";
  std::string pose_filename = ss2.str();

  outputPosesFile_.open(pose_filename.c_str() );
  outputPosesFile_ << "# clouds are relative to pose. row 0 is in odom frame. others are relative to it. format: cloudid,x,y,z,r,p,y (in m,rads)\n";
  outputPosesFile_.flush();
}


void Pass::collectNewSweepHandler(const std_msgs::Int16ConstPtr& msg){

  // 1. Collect the Scan
  ROS_INFO("Collect map command received. Collect %d seconds.", msg->data);
  ROS_INFO("Collect Data...");
  ros::Duration(msg->data).sleep(); 
  ROS_INFO("Finished collecting. Requesting sweep from assember");

  ros::Time time_now = ros::Time::now();
  ros::Time time_before = time_now - ros::Duration( (double)msg->data );

  ros::service::waitForService("assemble_scans2");
  ros::ServiceClient client = node_.serviceClient<AssembleScans2>("assemble_scans2");
  AssembleScans2 srv;
  srv.request.begin = time_before;
  srv.request.end   = time_now;

  printf("Get cloud from %6.4f to %6.4f (%d seconds)\n", time_before.toSec(), time_now.toSec(), msg->data);

  if (client.call(srv)){
    // Number of points, if uncompressed
    double numberPoints = srv.response.cloud.row_step / srv.response.cloud.point_step;
    printf("Got cloud with %.0f points (if uncompressed)\n", numberPoints);
  }else{
    printf("Service call failed\n");
  }


  // 2 Get Cloud and transform it relative to the robot
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(srv.response.cloud,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  std::cout << temp_cloud->width << " x " << temp_cloud->height << " | width x height\n";


  Eigen::Isometry3d pose_robot = Eigen::Isometry3d::Identity();
  tf::StampedTransform transform;
  try {
      ros::Time time_now = ros::Time::now();
      listener_->waitForTransform("/odom", "/base", time_now, ros::Duration(2.0) );
      listener_->lookupTransform("/odom", "/base", time_now, transform);
  } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
  }
  tf::transformTFToEigen (transform, pose_robot);

  Eigen::Isometry3d pose_robot_inv = pose_robot.inverse();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud4 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::transformPointCloud (*temp_cloud, *temp_cloud,
                           pose_robot_inv.translation().cast<float>(), Eigen::Quaternionf(pose_robot_inv.rotation().cast<float>()));

  // 3 Voxel Filter
  ROS_INFO("Voxel Filter (%fcm)...", voxelGridResolution_);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (temp_cloud);
  sor.setLeafSize (voxelGridResolution_, voxelGridResolution_, voxelGridResolution_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  sor.filter (*cloud_filtered);

  // 4 Get relative transform and write to file
  Eigen::Isometry3d output_pose;
  if (!initialized_){
    output_pose = pose_robot;
    initialized_ = true;
  }else{
    output_pose = prev_pose_robot_.inverse() * pose_robot;
  }
  // NB: keep the pose for late
  prev_pose_robot_ = pose_robot;

  std::string cloud_name = getTimeAsString();
  Eigen::Quaterniond quat(output_pose.rotation());
  double rpy[3];
  quat_to_euler(quat, rpy[0], rpy[1], rpy[2]);
  outputPosesFile_ << cloud_name << ", " 
      << output_pose.translation().x() << ", "
      << output_pose.translation().y() << ", "
      << output_pose.translation().z() << ", "
      << rpy[0] << ", "
      << rpy[1] << ", "
      << rpy[2] << "\n";
  outputPosesFile_.flush();

  std::cout << cloud_name << ", " 
      << output_pose.translation().x() << ", "
      << output_pose.translation().y() << ", "
      << output_pose.translation().z() << ", "
      << rpy[0] << ", "
      << rpy[1] << ", "
      << rpy[2] << " - output\n";


  // 5 Write point cloud to file
  std::stringstream ss_pcd_filename;
  ss_pcd_filename << outputDirectory_ << "/" << cloud_name << ".pcd";
  pcl::io::savePCDFileASCII (ss_pcd_filename.str(), *cloud_filtered);
  std::cout << "Saved " << cloud_filtered->points.size () << " data points to "<< ss_pcd_filename.str() << std::endl;

}


int main( int argc, char** argv ){
  ros::init(argc, argv, "lidar_sweep_collector");
 
  ros::NodeHandle nh("~");

  Pass app(nh);
  cout << "Ready with LidarSweepCollector" << endl << "============================" << endl;
  ROS_INFO_STREAM("LidarSweepCollector ready");
  ROS_ERROR_STREAM("LidarSweepCollector ready");
  ros::spin();
  return 0;
}