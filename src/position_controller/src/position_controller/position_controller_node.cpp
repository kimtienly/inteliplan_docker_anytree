#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <algorithm>

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <cstdlib> // abs

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

#include "std_srvs/SetBool.h"

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <dynamic_reconfigure/server.h>
#include <position_controller/PositionControllerConfig.h>
#include <position_controller/position_controller.hpp>


#include <actionlib/server/simple_action_server.h>
#include <position_controller/PositionControllerAction.h>

using namespace std;

class PositionControllerNode{
  public:
    PositionControllerNode(ros::NodeHandle node_, std::string name);
    
    ~PositionControllerNode(){
    }    

    void dynCallback(position_controller::PositionControllerConfig &config, uint32_t level);
    void readParamFromLaunch();

  private:
    ros::NodeHandle node_;

    actionlib::SimpleActionServer<position_controller::PositionControllerAction> as_;
    position_controller::PositionControllerResult result_;
    position_controller::PositionControllerFeedback feedback_;
    std::string action_name_;
    void newGoalRequestActionHandler();
    void preemptActionHandler();

    PositionController* positionController_;

    ros::Subscriber stopSub_, enableSub_, poseSub_, poseHuskySub_, drivingSub_, drivingRvizSub_, drivingRviz2Sub_, goalSub_, goalListSub_;
    void stopWalkingHandler(const std_msgs::Int16ConstPtr& msg);
    void enableHandler(const std_msgs::StringConstPtr& msg);
    void poseHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void newDrivingGoalHandler(const geometry_msgs::PoseStampedConstPtr& msg);
    void newDrivingGoalRvizHandler(const geometry_msgs::PoseStampedConstPtr& msg);
    void newGoalRequestHandler(const geometry_msgs::PoseStampedConstPtr& msg);
    void setNewGoal(const geometry_msgs::PoseStampedConstPtr& msg);
    void newGoalListRequestHandler(const geometry_msgs::PoseArrayConstPtr& msg);

    void computeControlCommand(Eigen::Isometry3d msg_pose, int64_t msg_utime, std_msgs::Header msg_header);
    void outputVisualization(int64_t msg_utime, std_msgs::Header msg_header);

    ros::Publisher positionControllerPub_;
    ros::Publisher visualizeCurrentGoalPub_, visualizeRemainingGoalsPub_, visualizeStartingGoalPub_;
    ros::Publisher visualizeVelocityCombinedPub_, visualizeVelocityTracklinePub_, visualizeVelocityGoalPub_, markerPub_;
    ros::Publisher positionControllerStatusPub_;

    ros::ServiceServer pauseService;
    bool pauseServiceHandler(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res);
    bool operationPaused_;

    // Function to publish the controller status (inactive/0 or active/1)
    void publishControllerStatus(int64_t msg_utime, FOLLOWER_OUTPUT output_mode);
    int64_t previousStatusPublishUtime_;
    int64_t previousVisualizationPublishUtime_;
    int64_t previousCommandUtime_;
    std_msgs::Header previousPoseHeader_;
    Eigen::Isometry3d previousPose_;
    std::vector<std::string> validGoalFrames_;
    float publishIntervalMicrosec_;
    bool differentialDrive_;
    float minLinearVelocity_;
    float minAngularVelocity_;
    std::string positionControllerTwistType_;

    dynamic_reconfigure::Server<position_controller::PositionControllerConfig> server;
    dynamic_reconfigure::Server<position_controller::PositionControllerConfig>::CallbackType f;

};



PositionControllerNode::PositionControllerNode(ros::NodeHandle node_, std::string action_name_):
    node_(node_), action_name_(action_name_), as_(node_, action_name_, false){

  positionController_ = new PositionController();

  // Create ActionLib Server
  as_.registerGoalCallback(boost::bind(&PositionControllerNode::newGoalRequestActionHandler, this));
  as_.registerPreemptCallback(boost::bind(&PositionControllerNode::preemptActionHandler, this));
  as_.start();

  // Dynamic Reconfigure
  f = boost::bind(&PositionControllerNode::dynCallback, this, _1, _2);
  server.setCallback(f);

  std::string poseInputTopic;
  if (!node_.getParam("pose_input_topic", poseInputTopic)) {
    ROS_ERROR("Could not read parameter `pose_input_topic`.");
    exit(-1);
  }
  ROS_INFO("Receive pelvis/base pose from %s", poseInputTopic.c_str());

  float maximum_control_command_rate;
  float micros_in_second = 1000000;
  node_.param<float>("maximum_control_command_rate_hz", maximum_control_command_rate, 0);
  if (maximum_control_command_rate > 0) {
    ROS_INFO("Applying maximum control signal rate of %f Hz", maximum_control_command_rate);
    publishIntervalMicrosec_ = (1.0 / maximum_control_command_rate) * micros_in_second;
  } else {
    ROS_INFO("No maximum control signal rate specified, control signals will be published based on the frequency of the pose input topic");
  }

  node_.param<bool>("differential_drive", differentialDrive_, false);
  if (differentialDrive_) {
    ROS_INFO("Using differential drive mode, twist command values on y axis will be zeroed.");
  }

  node_.param<float>("min_linear_velocity", minLinearVelocity_, 0);
  if (minLinearVelocity_ != 0) {
    ROS_INFO("Minimum linear velocity of controller set to %f", minLinearVelocity_);
  }
  node_.param<float>("min_angular_velocity", minAngularVelocity_, 0);
  if (minAngularVelocity_ != 0) {
    ROS_INFO("Minimum angular velocity of controller set to %f", minAngularVelocity_);
  }

  poseSub_ = node_.subscribe(poseInputTopic, 100, &PositionControllerNode::poseHandler, this);

  stopSub_     = node_.subscribe(std::string("/stop_walking_cmd"), 100, &PositionControllerNode::stopWalkingHandler, this);
  enableSub_   = node_.subscribe(std::string("/enable_position_controller"), 100, &PositionControllerNode::enableHandler, this);
  drivingSub_  = node_.subscribe(std::string("/driving_plan_request"), 100, &PositionControllerNode::newDrivingGoalHandler, this);
  drivingRvizSub_  = node_.subscribe(std::string("/goal"), 100, &PositionControllerNode::newDrivingGoalRvizHandler, this);
  drivingRviz2Sub_  = node_.subscribe(std::string("/move_base_simple/goal"), 100, &PositionControllerNode::newDrivingGoalRvizHandler, this); // rviz in certain configurations
  goalSub_ = node_.subscribe(std::string("/position_controller/goal_pose"), 100, &PositionControllerNode::newGoalRequestHandler, this);
  goalListSub_ = node_.subscribe(std::string("/position_controller/goal_pose_list"), 100, &PositionControllerNode::newGoalListRequestHandler, this);

  pauseService = node_.advertiseService("/position_controller/pause_execution", &PositionControllerNode::pauseServiceHandler, this);
  operationPaused_ = false;


  std::string positionControllerTopic;
  if (!node_.getParam("output_topic", positionControllerTopic)) {
    ROS_ERROR("Could not read parameter `output_topic`.");
    exit(-1);
  }
  ROS_INFO("Publish commands to %s", positionControllerTopic.c_str());

  node_.param<std::string>("output_type", positionControllerTwistType_, "twist_stamped");
  if (positionControllerTwistType_ == "twist") {
    positionControllerPub_ = node_.advertise<geometry_msgs::Twist>(  positionControllerTopic, 10);
  } else if (positionControllerTwistType_ == "twist_stamped") {
    positionControllerPub_ = node_.advertise<geometry_msgs::TwistStamped>(  positionControllerTopic, 10);
  } else {
    ROS_ERROR("Invalid twist type %s, valid options are 'twist' or 'twist_stamped'", positionControllerTwistType_.c_str());
    exit(1);
  }
  ROS_INFO("Publishing commands with type %s", positionControllerTwistType_.c_str());


  positionControllerStatusPub_ = node_.advertise<std_msgs::Bool>(  "/position_controller/controller_status", 10); // I don't think this is needed or used anymore

  // diagnostics:
  visualizeCurrentGoalPub_ = node_.advertise<geometry_msgs::PoseStamped>("/position_controller/current_goal", 10);
  visualizeRemainingGoalsPub_ = node_.advertise<geometry_msgs::PoseArray>("/position_controller/remaining_goals", 10);
  visualizeStartingGoalPub_ = node_.advertise<geometry_msgs::PoseStamped>("/position_controller/starting_pose", 10);

  visualizeVelocityCombinedPub_ = node_.advertise<geometry_msgs::WrenchStamped>("/position_controller/velocity_combined", 10);
  visualizeVelocityGoalPub_ = node_.advertise<geometry_msgs::WrenchStamped>("/position_controller/velocity_goal", 10);
  visualizeVelocityTracklinePub_ = node_.advertise<geometry_msgs::WrenchStamped>("/position_controller/velocity_trackline", 10);
  markerPub_ = node_.advertise<visualization_msgs::Marker>("/position_controller/visualization_marker", 10);


  previousStatusPublishUtime_ = 0;
  previousVisualizationPublishUtime_ = 0;
  previousCommandUtime_ = 0;
  previousPoseHeader_.frame_id = "base";
  previousPoseHeader_.stamp.sec = 0;
  previousPoseHeader_.stamp.nsec = 0;

  previousPose_ = Eigen::Isometry3d::Identity();

}


void PositionControllerNode::readParamFromLaunch(){
  double max_forward_linear_velocity = 0;
  if (!node_.getParam("max_forward_linear_velocity", max_forward_linear_velocity)) {
    ROS_ERROR("Could not read parameter `max_forward_linear_velocity`.");
    exit(-1);
  }
  double max_lateral_linear_velocity = 0;
  if (!node_.getParam("max_lateral_linear_velocity", max_lateral_linear_velocity)) {
    ROS_ERROR("Could not read parameter `max_lateral_linear_velocity`.");
    exit(-1);
  }
  double max_angular_velocity = 0;
  if (!node_.getParam("max_angular_velocity", max_angular_velocity)) {
    ROS_ERROR("Could not read parameter `max_angular_velocity`.");
    exit(-1);
  }
  double max_turning_linear_velocity = 0;
  if (!node_.getParam("max_turning_linear_velocity", max_turning_linear_velocity)) {
    ROS_ERROR("Could not read parameter `max_turning_linear_velocity`.");
    exit(-1);
  }
  double angular_gain_p = 0;
  if (!node_.getParam("angular_gain_p", angular_gain_p)) {
    ROS_ERROR("Could not read parameter `angular_gain_p`.");
    exit(-1);
  }
  double goal_distance_threshold = 0;
  if (!node_.getParam("goal_distance_threshold", goal_distance_threshold)) {
    ROS_ERROR("Could not read parameter `goal_distance_threshold`.");
    exit(-1);
  }
  double goal_heading_threshold = 0;
  if (!node_.getParam("goal_heading_threshold", goal_heading_threshold)) {
    ROS_ERROR("Could not read parameter `goal_heading_threshold`.");
    exit(-1);
  }
  double turn_to_face_heading_threshold = 0;
  if (!node_.getParam("turn_to_face_heading_threshold", turn_to_face_heading_threshold)) {
    ROS_ERROR("Could not read parameter `turn_to_face_heading_threshold`.");
    exit(-1);
  }

  int goal_behind_mode = 1; // 0 ignore, 1 backwards, 2 turnaround
  if (!node_.getParam("goal_behind_mode", goal_behind_mode)) {
    ROS_ERROR("Could not read parameter `goal_behind_mode`.");
    exit(-1);
  }

  int motion_mode = 0; // 0 turnwalkturn. 1 shuffle
  if (!node_.getParam("motion_mode", motion_mode)) {
    ROS_ERROR("Could not read parameter `motion_mode`.");
    exit(-1);
  }
  ROS_INFO("motion_mode is %d", motion_mode);

  double trackline_gain_p = 0; // 0 turnwalkturn. 1 shuffle
  if (!node_.getParam("trackline_gain_p", trackline_gain_p)) {
    ROS_ERROR("Could not read parameter `trackline_gain_p`.");
    exit(-1);
  }
  ROS_INFO("trackline_gain_p is %f", trackline_gain_p);

  std::string fixed_frame;
  node_.param("fixed_frame", fixed_frame, std::string("odom"));

  XmlRpc::XmlRpcValue valid_goal_frames;
  node_.getParam("valid_goal_frames", valid_goal_frames);
  if (valid_goal_frames.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("valid_goal_frames parameter must be a list.");
    exit(-1);
  } else {
    for(int i =0; i < valid_goal_frames.size(); i++)
    {
      validGoalFrames_.push_back(valid_goal_frames[i]);
    }
}

  bool back_is_front = false;
  if (!node_.getParam("back_is_front", back_is_front)) {
    ROS_ERROR("Could not read parameter `back_is_front`.");
    exit(-1);
  }
  ROS_INFO("back_is_front is %d", back_is_front);

  bool yaw_exclusive_turns = false;
  if (!node_.getParam("yaw_exclusive_turns", yaw_exclusive_turns)) {
    ROS_ERROR("Could not read parameter `yaw_exclusive_turns`.");
    exit(-1);
  }
  ROS_INFO("yaw_exclusive_turns is %d", yaw_exclusive_turns);

  bool ignore_intermediate_goal_heading = false;
  if (!node_.getParam("ignore_intermediate_goal_heading", ignore_intermediate_goal_heading)) {
    ROS_ERROR("Could not read parameter `ignore_intermediate_goal_heading`.");
    exit(-1);
  }
  ROS_INFO("ignore_intermediate_goal_heading is %d", ignore_intermediate_goal_heading);  

  ROS_INFO("Ros Param from launch: %f %f %f %f %f %f %f %f %d %d %f %s %d %d %d",
            max_forward_linear_velocity,
            max_lateral_linear_velocity,
            max_angular_velocity,
            max_turning_linear_velocity,
            angular_gain_p,
            goal_distance_threshold,
            goal_heading_threshold,
            turn_to_face_heading_threshold,
            goal_behind_mode,
            motion_mode,
            trackline_gain_p,
            fixed_frame.c_str(),
            back_is_front,
            yaw_exclusive_turns,
            ignore_intermediate_goal_heading);
  positionController_->setMaxForwardLinearVelocity(max_forward_linear_velocity);
  positionController_->setMaxLateralLinearVelocity(max_lateral_linear_velocity);
  positionController_->setMaxAngularVelocity(max_angular_velocity);
  positionController_->setMaxTurningLinearVelocity(max_turning_linear_velocity);
  positionController_->setAngularGainP(angular_gain_p);
  positionController_->setGoalDistanceThreshold(goal_distance_threshold);
  positionController_->setGoalHeadingThreshold(goal_heading_threshold);
  positionController_->setTurnToFaceHeadingThreshold(turn_to_face_heading_threshold);
  positionController_->setGoalBehindMode(goal_behind_mode);
  positionController_->setMotionMode(motion_mode);
  positionController_->setTracklineGainP(trackline_gain_p);
  positionController_->setFixedFrame(fixed_frame);
  positionController_->setBackIsFront(back_is_front);
  positionController_->setYawExclusiveTurns(yaw_exclusive_turns);
  positionController_->setIgnoreIntermediateGoalHeading(ignore_intermediate_goal_heading);

}


void PositionControllerNode::dynCallback(position_controller::PositionControllerConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f %f %d %d %f %d %d %d",
            config.max_forward_linear_velocity,
            config.max_lateral_linear_velocity,
            config.max_angular_velocity,
            config.max_turning_linear_velocity,
            config.angular_gain_p,
            config.goal_distance_threshold,
            config.goal_heading_threshold,
            config.turn_to_face_heading_threshold,
            config.goal_behind_mode,
            config.motion_mode,
            config.trackline_gain_p,
            config.back_is_front,
            config.yaw_exclusive_turns,
            config.ignore_intermediate_goal_heading);

  positionController_->setMaxForwardLinearVelocity(config.max_forward_linear_velocity);
  positionController_->setMaxLateralLinearVelocity(config.max_lateral_linear_velocity);
  positionController_->setMaxAngularVelocity(config.max_angular_velocity);
  positionController_->setMaxTurningLinearVelocity(config.max_turning_linear_velocity);
  positionController_->setAngularGainP(config.angular_gain_p);
  positionController_->setGoalDistanceThreshold(config.goal_distance_threshold);
  positionController_->setGoalHeadingThreshold(config.goal_heading_threshold);
  positionController_->setTurnToFaceHeadingThreshold(config.turn_to_face_heading_threshold);
  positionController_->setGoalBehindMode(config.goal_behind_mode);
  positionController_->setMotionMode(config.motion_mode);
  positionController_->setTracklineGainP(config.trackline_gain_p);
  positionController_->setBackIsFront(config.back_is_front);
  positionController_->setYawExclusiveTurns(config.yaw_exclusive_turns);
  positionController_->setIgnoreIntermediateGoalHeading(config.ignore_intermediate_goal_heading);
}


// Functions for newer ActionLib approach:
void PositionControllerNode::newGoalRequestActionHandler(){
  ROS_INFO_STREAM("PositionController: New goal received - actionlib");
  geometry_msgs::PoseStamped goal= as_.acceptNewGoal()->goal;
  geometry_msgs::PoseStampedConstPtr goal_pointer( new geometry_msgs::PoseStamped(goal) );
  setNewGoal( goal_pointer);
}

void PositionControllerNode::preemptActionHandler(){
  ROS_INFO_STREAM("PositionController: preemptActionHandler - stop walking");
  as_.setPreempted();
  std_msgs::Int16ConstPtr your_const_pointer( new std_msgs::Int16() );
  stopWalkingHandler(your_const_pointer);
}
///////////////////////////////////


void PositionControllerNode::stopWalkingHandler(const std_msgs::Int16ConstPtr& msg){
  ROS_INFO("STOP_WALKING received. Following disabled and command zero velocity");

  geometry_msgs::TwistStamped cmd;
  cmd.header = previousPoseHeader_;
  cmd.twist.linear.x = 0;
  cmd.twist.linear.y = 0;
  cmd.twist.linear.z = 0;
  cmd.twist.angular.x = 0;
  cmd.twist.angular.y = 0;
  cmd.twist.angular.z = 0;
  if (positionControllerTwistType_ == "twist") {
    positionControllerPub_.publish(cmd.twist);
  } else {
    positionControllerPub_.publish(cmd);
  }

  positionController_->stopWalking();
}

// Load a trajectory and get going:
void PositionControllerNode::enableHandler(const std_msgs::StringConstPtr& msg){
  std::cout << "ENABLE_PATH_FOLLOWER received. Not re-implemented yet within ROS launch/cfg\n";
  //std::cout << "ENABLE_PATH_FOLLOWER received. Following enabled\n";
  //positionController_->prepareGoalTrajectory();
}


void PositionControllerNode::newDrivingGoalHandler(const geometry_msgs::PoseStampedConstPtr& msg){
  ROS_INFO_STREAM("New driving goal received");
  setNewGoal(msg);
}

void PositionControllerNode::newDrivingGoalRvizHandler(const geometry_msgs::PoseStampedConstPtr& msg){
  ROS_INFO_STREAM("New Rviz goal received");
  setNewGoal(msg);
}

void PositionControllerNode::newGoalRequestHandler(const geometry_msgs::PoseStampedConstPtr& msg){
  ROS_INFO_STREAM("GOAL_REQUEST goal received");
  setNewGoal(msg);
}


void PositionControllerNode::setNewGoal(const geometry_msgs::PoseStampedConstPtr& msg){
  ROS_INFO_STREAM("New goal with frame_id: " << msg->header.frame_id );

  if (std::find(validGoalFrames_.begin(), validGoalFrames_.end(), msg->header.frame_id) == validGoalFrames_.end()) {
    ROS_INFO("Goal frame '%s' is not in the list of valid goal frames. Rejected", msg->header.frame_id.c_str());
    as_.setAborted();
    return;
  }

  Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
  tf::poseMsgToEigen(msg->pose, goal);

  PositionGoal position_goal = PositionGoal(msg->header.frame_id, goal);
  positionController_->setGoalAndEnable( position_goal, previousPose_ );
}


void PositionControllerNode::newGoalListRequestHandler(const geometry_msgs::PoseArrayConstPtr& msg){
  ROS_INFO_STREAM("GOAL_REQUEST goal list received");
  std::deque<PositionGoal> goals;
  for (size_t i =0 ; i < msg->poses.size() ; i ++){
    Eigen::Isometry3d msg_pose = Eigen::Isometry3d::Identity();
    tf::poseMsgToEigen(msg->poses[i], msg_pose);
    PositionGoal position_goal = PositionGoal(msg->header.frame_id, msg_pose);
    goals.push_back(position_goal);
  }

  positionController_->setGoalListAndEnable( goals, previousPose_ );


}


void PositionControllerNode::poseHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  previousPoseHeader_ = msg->header;
  tf::poseMsgToEigen(msg->pose.pose, previousPose_);
  ROS_INFO_ONCE("poseHandler - got robot's base pose");
  int64_t msg_utime = (int64_t)floor(msg->header.stamp.toNSec() / 1000);

  if ((msg_utime - previousCommandUtime_) < publishIntervalMicrosec_)
    return;
  previousCommandUtime_ = msg_utime;


  if (operationPaused_){
    ROS_INFO_THROTTLE(1,"positionController is paused. Not outputing any commands");
    return;
  }

  computeControlCommand(previousPose_, msg_utime, msg->header);

}


void PositionControllerNode::outputVisualization(int64_t msg_utime, std_msgs::Header msg_header){
  if ((msg_utime - previousVisualizationPublishUtime_) < publishIntervalMicrosec_)
    return;
  previousVisualizationPublishUtime_ = msg_utime;  

  Eigen::Isometry3d current_goal_in_fixed;
  if (!positionController_->getCurrentGoalInFixed(current_goal_in_fixed)){
    ROS_INFO_THROTTLE(1,"positionController: cannot publish visualization");
    return;
  }


  // Visualize the starting pose
  geometry_msgs::PoseStamped m_starting;
  m_starting.header = msg_header;
  Eigen::Isometry3d starting_pose = positionController_->getStartingPose();
  tf::poseEigenToMsg (starting_pose, m_starting.pose);
  visualizeStartingGoalPub_.publish(m_starting);

  // Visualize the current goal
  geometry_msgs::PoseStamped m;
  m.header = msg_header;
  tf::poseEigenToMsg (current_goal_in_fixed, m.pose);
  visualizeCurrentGoalPub_.publish(m);

  // output velocity - as a wrench for convenience
  {
    geometry_msgs::WrenchStamped ws;
    ws.header = msg_header;
    ws.header.frame_id = "base";
    Eigen::Vector3d output_linear_velocity;
    Eigen::Vector3d output_angular_velocity;
    positionController_->getOutputVelocity(output_linear_velocity, output_angular_velocity);
    ws.wrench.force.x = output_linear_velocity[0];
    ws.wrench.force.y = output_linear_velocity[1];
    ws.wrench.force.z = output_linear_velocity[2];
    ws.wrench.torque.x = output_angular_velocity[0];
    ws.wrench.torque.y = output_angular_velocity[1];
    ws.wrench.torque.z = fabs( output_angular_velocity[2]); // for some reason rviz wont visualise this if nexative
    visualizeVelocityCombinedPub_.publish(ws);
  }

  // goal-seeking velocity - as a wrench for convenience
  {
  geometry_msgs::WrenchStamped ws;
  ws.header = msg_header;
  ws.header.frame_id = "base";
  ws.wrench.force.x = positionController_->goal_linvel_x_;
  ws.wrench.force.y = positionController_->goal_linvel_y_;
  ws.wrench.torque.z = fabs( positionController_->goal_angvel_z_ ); // for some reason rviz wont visualise this if nexative
  visualizeVelocityGoalPub_.publish(ws);
  }

  // trackline velocity - as a wrench for convenience
  geometry_msgs::WrenchStamped ws;
  ws.header = msg_header;
  ws.header.frame_id = "base";
  ws.wrench.force.x = positionController_->trackline_linvel_x_;
  ws.wrench.force.y = positionController_->trackline_linvel_y_;
  visualizeVelocityTracklinePub_.publish(ws);

  // the line from the starting point to the goal
  visualization_msgs::Marker line_list;
  line_list.header = msg_header;
  line_list.header.frame_id = positionController_->fixed_frame_;
  line_list.ns = "points_and_lines";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;

  line_list.id = 2;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.05;
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;

  geometry_msgs::Point p;
  p.x = starting_pose.translation().x();
  p.y = starting_pose.translation().y();
  p.z = starting_pose.translation().z();
  line_list.points.push_back(p);

  geometry_msgs::Point p2;
  p2.x = current_goal_in_fixed.translation().x();
  p2.y = current_goal_in_fixed.translation().y();
  p2.z = current_goal_in_fixed.translation().z();
  line_list.points.push_back(p2);
  markerPub_.publish(line_list);

}

void PositionControllerNode::publishControllerStatus(int64_t msg_utime, FOLLOWER_OUTPUT output_mode){
  // only publish at 10Hz or less
  if ((msg_utime - previousStatusPublishUtime_) < publishIntervalMicrosec_)
    return;

  std_msgs::Bool msg;
  msg.data = false; // inactive
  if (output_mode == SEND_COMMAND)
    msg.data = true; // active

  positionControllerStatusPub_.publish(msg);
  previousStatusPublishUtime_ = msg_utime;


  // Sent to the action client
  if (as_.isActive()){
    feedback_.distance_to_goal =  positionController_->getDistanceToGoal();
    as_.publishFeedback(feedback_);
  }

}


void PositionControllerNode::computeControlCommand(Eigen::Isometry3d msg_pose, int64_t msg_utime, std_msgs::Header msg_header){

  FOLLOWER_OUTPUT output_mode = positionController_->computeControlCommand( msg_pose );
  publishControllerStatus(msg_utime, output_mode);

  if (output_mode == SEND_STOP_WALKING){
    as_.setSucceeded(result_);
  }


  if (output_mode == SEND_NOTHING){
    ROS_INFO_THROTTLE(1,"SEND_NOTHING - positionController disabled");
  }else if (output_mode == SEND_STOP_WALKING){
    ROS_INFO("SEND_STOP_WALKING and command zero velocity");
    geometry_msgs::TwistStamped cmd;
    cmd.header = previousPoseHeader_;
    cmd.twist.linear.x = 0;
    cmd.twist.linear.y = 0;
    cmd.twist.linear.z = 0;
    cmd.twist.angular.x = 0;
    cmd.twist.angular.y = 0;
    cmd.twist.angular.z = 0;

    if (positionControllerTwistType_ == "twist") {
      positionControllerPub_.publish(cmd.twist);
    } else {
      positionControllerPub_.publish(cmd);
    }

    // Depreciated. Happens automatically as of Jun 2018
    //std_msgs::Int16 stop_msg;
    //stop_msg.data = 2; // Terminate (TODO: use enum message)
    //stopWalkingPub_.publish(stop_msg);

  }else if (output_mode == SEND_COMMAND){
    Eigen::Vector3d output_linear_velocity;
    Eigen::Vector3d output_angular_velocity;
    positionController_->getOutputVelocity(output_linear_velocity, output_angular_velocity);

    geometry_msgs::TwistStamped cmd;
    cmd.header = previousPoseHeader_;
    cmd.twist.linear.x = output_linear_velocity(0);
    cmd.twist.linear.y = output_linear_velocity(1);
    cmd.twist.linear.z = output_linear_velocity(2);
    cmd.twist.angular.x = output_angular_velocity(0);
    cmd.twist.angular.y = output_angular_velocity(1);
    cmd.twist.angular.z = output_angular_velocity(2);

    // Differential drive robots can only apply twist commands to yaw and x axis
    if (differentialDrive_) {
      cmd.twist.linear.y = 0;  
    }

    // Force the minimum velocities to the specified value if the absolute value
    // output by the position controller is below it.
    if (minLinearVelocity_ != 0) {
      if (cmd.twist.linear.x != 0 && abs(cmd.twist.linear.x) < minLinearVelocity_) {
        ROS_DEBUG("x linear twist value changed from %f to the specified minimum of %f (with the same sign)", cmd.twist.linear.x, minLinearVelocity_);
        if (cmd.twist.linear.x < 0) {
          cmd.twist.linear.x = -minLinearVelocity_;
        } else {
          cmd.twist.linear.x = minLinearVelocity_;  
        }
      }
    
      if (cmd.twist.linear.y != 0 && abs(cmd.twist.linear.y) < minLinearVelocity_) {
        ROS_DEBUG("y linear twist value changed from %f to the specified minimum of %f (with the same sign)", cmd.twist.linear.y, minLinearVelocity_);  
        if (cmd.twist.linear.y < 0) {
          cmd.twist.linear.y = -minLinearVelocity_;
        } else {
          cmd.twist.linear.y = minLinearVelocity_;
        }
      }      
    }

    if (minAngularVelocity_ != 0 && cmd.twist.angular.z != 0 && abs(cmd.twist.angular.z) < minAngularVelocity_) {
      ROS_DEBUG("z angular twist value changed from %f to the specified minimum of %f (with the same sign)", cmd.twist.angular.y, minAngularVelocity_);
      if (cmd.twist.angular.z < 0) {
        cmd.twist.angular.z = -minAngularVelocity_;
      } else {
        cmd.twist.angular.z = minAngularVelocity_;  
      }
    }
    if (positionControllerTwistType_ == "twist") {
      positionControllerPub_.publish(cmd.twist);
    } else {
      positionControllerPub_.publish(cmd);
    }

    outputVisualization(msg_utime, msg_header);


    // The following isn't fully debugged:
    std::deque<PositionGoal> remainingGoals;
    positionController_->getGoals(remainingGoals);
    if (remainingGoals.size() > 0){
      //std::cout << "got " << remainingGoals.size() << " old poses\n";
      geometry_msgs::PoseArray mArray;
      mArray.header = msg_header;
      mArray.header.frame_id = remainingGoals[0].frame_id; // assumes all frame_id are the same
      for (size_t i=0 ; i < remainingGoals.size(); i++){
        geometry_msgs::Pose p;
        tf::poseEigenToMsg ( remainingGoals[i].pose, p);
        mArray.poses.push_back( p );
      }
      visualizeRemainingGoalsPub_.publish(mArray);
    }

  }
}


bool PositionControllerNode::pauseServiceHandler(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res)
{
  std::string message = "PositionController pausing operation";
  if (!req.data)
    message = "PositionController unpausing operation";
    
  ROS_INFO_STREAM(message);

  res.success = true;
  res.message = message;
  operationPaused_ = req.data;

  if (operationPaused_){ 
    // I think this doesn't work - the message never makes it out. perhaps there service handler blocks publishing?
    ROS_INFO_STREAM("Sending zero velocity command to pause robot");
    // If pausing also send a command to stop the robot
    // Otherwise it will trot along until the last command times out
    geometry_msgs::TwistStamped cmd;
    cmd.header = previousPoseHeader_;
    cmd.twist.linear.x = 0;
    cmd.twist.linear.y = 0;
    cmd.twist.linear.z = 0;
    cmd.twist.angular.x = 0;
    cmd.twist.angular.y = 0;
    cmd.twist.angular.z = 0;
    if (positionControllerTwistType_ == "twist") {
      positionControllerPub_.publish(cmd.twist);
    } else {
      positionControllerPub_.publish(cmd);
    }
    
  }

  return true;
}



int main( int argc, char** argv ){
  ros::init(argc, argv, "position_controller");
 
  ros::NodeHandle nh("~");

  static PositionControllerNode *app = new PositionControllerNode(nh, ros::this_node::getName());

  ROS_INFO_STREAM("Ready to follow position goal");
  ROS_INFO_STREAM("=============================");
  app->readParamFromLaunch();


  ROS_INFO_STREAM("positionController ros ready");
  ros::spin();
  return 0;
}
