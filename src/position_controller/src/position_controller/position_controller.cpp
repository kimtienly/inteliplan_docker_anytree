#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <position_controller/position_controller.hpp>

enum class State {
  // UNKNOWN only occurs when starting
  // FINISHED robot at goal. Do nothing
  // TURN_TO_DESTINATION robot at goal but with wrong heading. turn in place to des heading
  // FORWARD robot far from goal but facing it. walk forward
  // TURN_TO_GOAL robot far from goal and not facing it. turn in place to face des position
  // GOAL_BEHIND goal is behind the robot. do nothing
  UNKNOWN = -1, FINISHED = 0, TURN_TO_DESTINATION = 1, FORWARD = 2, TURN_TO_GOAL = 3, GOAL_BEHIND = 4
};

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




PositionController::PositionController(){

  tfListener_ = boost::make_shared<tf::TransformListener>();

  current_goal_.pose.translation()[0] = std::numeric_limits<double>::infinity();
  have_got_near_to_current_goal_ = false;

  ROS_INFO("PositionController set up");
  fixed_frame_ = "odom";

}


bool PositionController::getNextGoal(Eigen::Isometry3d current_pose){
  if (goals_.size() == 0){
    // this is to try to break things which use this value
    current_goal_.pose.translation()[0] = std::numeric_limits<double>::infinity();
    return false;
  }

  // Get the next goal
  ROS_INFO("Goals size: %lu (before)", goals_.size());
  current_goal_ = goals_.front();
  have_got_near_to_current_goal_ = false;
  goals_.pop_front();
  starting_pose_ = current_pose;
  ROS_INFO("Goals size: %lu (after)", goals_.size());

  return true;
}


// clip to be in range [-max_value:max_value]
void clipValue(double &value, double max_value){
  if (value > max_value)
    value = max_value;
  else if (value < -max_value)
    value = -max_value;
}


// constrain angle to be -180:180 in radians
double constrainAngle(double x){
    x = fmod(x + M_PI,2.0*M_PI);
    if (x < 0)
        x += 2.0*M_PI;
    return x - M_PI;
}


void PositionController::computeTracklineVelocityCommand(Eigen::Isometry3d goal_pose, Eigen::Isometry3d current_pose){
  // How far is the robot from the trackline?
  // Create an x/y base velocity term proportional to that distance 

  // signed perpendicular distance between pt0 and line (between pt1 and pt2)
  // point: 0  current_pose
  // line: 1  starting_pose and  2  goal_pose
	// distance is negative on LHS of line. positive on RHS of line
  double x0, y0, x1, y1, x2, y2;
  x0 = current_pose.translation().x();
  y0 = current_pose.translation().y();
  x1 = starting_pose_.translation().x();
  y1 = starting_pose_.translation().y();
  x2 = goal_pose.translation().x();
  y2 = goal_pose.translation().y();
  double numer = (x2-x1)*(y1-y0) - (x1-x0)*(y2-y1);
  double denom = sqrt( (y2-y1)*(y2-y1) + (x2-x1)*(x2-x1) );
  double tracklineDistanceError = numer/denom;
  double slope = atan2((y2 - y1),(x2 - x1));
  //ROS_INFO_THROTTLE(1,"        TRACKLN: %f slope of trackline", slope*57);

  Eigen::Quaterniond q(current_pose.rotation());
  double current_roll, current_pitch, current_yaw;
  quat_to_euler(q, current_roll, current_pitch, current_yaw);
  //ROS_INFO_THROTTLE(1,"        TRACKLN: %f current_yaw of robot", current_yaw*57);

  // This is the angle between the trackline and the direction the robot is facing
  // this is zero if the robot is on trackline and facing goal
  // it is negative on LHS of line. positive on RHS of line
  double projection_angle = current_yaw - slope;
  // ROS_INFO_THROTTLE(1,"        TRACKLN: %f projection_angle", projection_angle*57);
  

  // Create velocity command proportional to error
  double tracklineVelocity = tracklineDistanceError * trackline_gain_p_;
  if (tracklineVelocity > max_forward_linear_velocity_)
    tracklineVelocity = max_forward_linear_velocity_;
  if (tracklineVelocity < -max_forward_linear_velocity_)
    tracklineVelocity = -max_forward_linear_velocity_;

  trackline_linvel_x_ =  tracklineVelocity * sin(projection_angle);
  trackline_linvel_y_ =     tracklineVelocity * cos(projection_angle);
  //ROS_INFO_THROTTLE(1,"        TRACKLN: %f      Velocity norm", tracklineVelocity);
  //ROS_INFO_THROTTLE(1,"        TRACKLN: %f  Forward  %f  Left", trackline_linvel_x_, trackline_linvel_y_);

}


bool PositionController::getCurrentGoalInFixed(Eigen::Isometry3d& current_goal_in_fixed){

  if (current_goal_.frame_id == fixed_frame_){ // directly execute, as controller works in the fixed frame
    current_goal_in_fixed = current_goal_.pose;
    ROS_DEBUG("Goal is already in our fixed frame");
    return true;
  }else { // transform goal into fixed frame
    ROS_DEBUG("Transforming goal into fixed frame");
    tf::StampedTransform fixed_to_goal_transform;
    tfListener_->waitForTransform(fixed_frame_, current_goal_.frame_id, ros::Time(0), ros::Duration(2.0));
    try {
      tfListener_->lookupTransform(fixed_frame_, current_goal_.frame_id, ros::Time(0), fixed_to_goal_transform);

      Eigen::Isometry3d fixed_to_goal = Eigen::Isometry3d::Identity();
      tf::transformTFToEigen (fixed_to_goal_transform, fixed_to_goal);
      //std::cout << "odom_to_map: " << odom_to_map.translation().transpose() << "\n";
      current_goal_in_fixed = fixed_to_goal * current_goal_.pose;
      return true;
    }
    catch (tf::TransformException& ex){
      ROS_ERROR("%s",ex.what());
      return false;
    }
  }

  return false;
}


FOLLOWER_OUTPUT PositionController::computeControlCommand(Eigen::Isometry3d current_pose){
  FOLLOWER_OUTPUT follower_output = SEND_COMMAND;

  // if there is a goal try to follow it, otherwise return.
  if (current_goal_.pose.translation()[0] == std::numeric_limits<double>::infinity()){
    have_got_near_to_current_goal_ = false;
    return SEND_NOTHING;
  }

  Eigen::Isometry3d current_goal_in_fixed;
  if (!getCurrentGoalInFixed(current_goal_in_fixed)){
    ROS_INFO_STREAM("Error: could not transform goal!");
    return SEND_NOTHING;
  }

  ROS_DEBUG("Goal in fixed frame: (%f %f %f)", current_goal_in_fixed.translation().x(), current_goal_in_fixed.translation().y(), current_goal_in_fixed.translation().z());
  if (back_is_front_) {
    Eigen::Quaterniond flip_front_direction =
        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
    current_pose = current_pose * flip_front_direction;
  }

  // outputs
  goal_linvel_x_ = 0;
  goal_linvel_y_ = 0;
  goal_angvel_z_ = 0;
  bool finishedWithThisGoal = false;

  // how far are we away from the goal
  Eigen::Isometry3d deltaPose = current_pose.inverse() * current_goal_in_fixed;
  deltaPose.translation()[2] = 0; // not interested in z error
  dist_to_goal_in_2d_ = hypot( deltaPose.translation().x() , deltaPose.translation().y());


  ////////////// If we want to track position to the goal pose
  // This is the angle, in the robot base frame from the current position to the goal position
  // We want to publish twist vectors which point in this direction
  double headingAngleToGoal = atan2(deltaPose.translation().y(), deltaPose.translation().x());
  // if the goal is BEHIND, then turn to face AWAY from the goal (if enabled)
  if ((deltaPose.translation().x()<0) && (goal_behind_mode_ == 1 )){
    headingAngleToGoal = constrainAngle(headingAngleToGoal + M_PI);
  }

  ////////////// If we want to track position back to the starting pose
  // how far are we away from the start
  Eigen::Isometry3d deltaPoseToStart = current_pose.inverse() * starting_pose_ ;
  deltaPoseToStart.translation()[2] = 0; // not interested in z error
  // This is the angle, in the robot base frame from the current position to the starting position
  // We want to publish twist vectors which point in this direction
  double headingAngleToStart = atan2(deltaPoseToStart.translation().y(), deltaPoseToStart.translation().x());


  // are we looking in the final desired direction
  Eigen::Quaterniond q(current_pose.rotation());
  double current_roll, current_pitch, current_yaw;
  quat_to_euler(q, current_roll, current_pitch, current_yaw);
  Eigen::Quaterniond q_des(current_goal_in_fixed.rotation());
  double des_roll, des_pitch, des_yaw;
  quat_to_euler(q_des, des_roll, des_pitch, des_yaw);
  double desHeadingError = constrainAngle(current_yaw - des_yaw);


  //-1 UNKNOWN only occurs when starting
  // 0 FINISHD robot at goal. Do nothing
  // 1 TRN2DES robot at goal but with wrong heading. turn in place to des heading
  // 2 FORWARD robot far from goal but facing it. walk forward
  // 3 TRN2GOL robot far from goal and not facing it. turn in place to face des position
  // 4 GOALBND goal is behind the robot. do nothing
  State state = State::UNKNOWN;
  if (dist_to_goal_in_2d_ < goal_distance_threshold_){
    have_got_near_to_current_goal_ = true;
    ROS_INFO_THROTTLE(1,"TRN2DES: close to goal now. Only TRN2DES possible now"); // comment this after testing
    if ( fabs(desHeadingError) < goal_heading_threshold_ || (ignore_intermediate_goal_heading_ && !goals_.empty())){
      state = State::FINISHED;
    }else{
      state = State::TURN_TO_DESTINATION;
    }
  }else if ((deltaPose.translation().x()<0) && (goal_behind_mode_ == 0)){
    state = State::GOAL_BEHIND;
  }else{
    if (have_got_near_to_current_goal_){
      // if the robot has gotten near to the goal previously, then only allow it
      // to track position and orientation for final alignment: disable the FORWARD and TRN2GOL
      state = State::TURN_TO_DESTINATION;
      ROS_INFO_THROTTLE(1,"TRN2DES: Have drifted outside goal_distance_threshold. Insisting on TRN2DES"); // comment this after testing

      ///////////////////////// NEW JAN 2021 /////////////////////////////////
      if(dist_to_goal_in_2d_ > goal_distance_threshold_*1.75){ // robot has drifted outside the goal threshod ... try again. This is needed for wheeled platforms
        state = State::TURN_TO_GOAL;
        have_got_near_to_current_goal_ = false;
        ROS_INFO_THROTTLE(1,"DRIFTED: %f to goal in m (%f thresh). %f in rad (%f thresh).", dist_to_goal_in_2d_, goal_distance_threshold_, desHeadingError, goal_heading_threshold_);
        ROS_INFO_THROTTLE(1,"DRIFTED: Position > 1.75*goal_heading_threshold. Returning to TRN2GOL!!!!"); // comment this after testing
        ROS_INFO_THROTTLE(1,"DRIFTED: MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM"); // comment this after testing
      }else{ // if position is near-ish the goal and heading is good, then finish (this is a hystersis fix)
        if ( fabs(desHeadingError) < goal_heading_threshold_ || (ignore_intermediate_goal_heading_ && !goals_.empty())){
          state = State::FINISHED;
          ROS_INFO_THROTTLE(1,"DRIFTED: %f to goal in m (%f thresh). %f in rad (%f thresh).", dist_to_goal_in_2d_, goal_distance_threshold_, desHeadingError, goal_heading_threshold_);
          ROS_INFO_THROTTLE(1,"DRIFTED: Position < 1.75*goal_heading_threshold. Finishing because heading is close enough!!!"); // comment this after testing
          ROS_INFO_THROTTLE(1,"DRIFTED: MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM"); // comment this after testing
        }
      }
      //////////////////////////////////////////////////////////
    }else if (fabs(headingAngleToGoal) < turn_to_face_heading_threshold_) {
      state = State::FORWARD;
    }else{
      state = State::TURN_TO_GOAL;
    }
  }


  if (motion_mode_ == 1){ // 1 is 'shuffle' mode:
    // when in the other walking states, switch to state 3 (track goal position and orientation)
    if ((state==State::FORWARD)||(state==State::TURN_TO_GOAL)){
      state = State::TURN_TO_DESTINATION;
    }
  }


  if (state == State::GOAL_BEHIND){
    goal_angvel_z_ = 0;
    goal_linvel_x_ = 0;
    goal_linvel_y_ = 0;
    finishedWithThisGoal = true;
    ROS_INFO("GOALBHD: %fm to goal | target behind robot (%f, %f, %f), do nothing. cancel goal.\n", dist_to_goal_in_2d_, deltaPose.translation().x(), deltaPose.translation().y(), deltaPose.translation().z());
  }else if(state == State::TURN_TO_GOAL){
    goal_angvel_z_ = headingAngleToGoal * angular_gain_p_;
    clipValue(goal_angvel_z_, max_angular_velocity_);
    goal_linvel_x_ = cos(headingAngleToStart)* max_forward_linear_velocity_; // m/s
    goal_linvel_y_ = sin(headingAngleToStart)* max_forward_linear_velocity_; // m/s

    if (yaw_exclusive_turns_) {
      goal_linvel_x_ = 0;
      goal_linvel_y_ = 0;
    }

    ROS_INFO_THROTTLE(1, "TRN2GOL: %f to face goal in rad (%f thresh). %f des ang vel", headingAngleToGoal, turn_to_face_heading_threshold_, goal_angvel_z_);
  }else if(state == State::FORWARD){
    goal_angvel_z_ = headingAngleToGoal * angular_gain_p_;
    clipValue(goal_angvel_z_, max_angular_velocity_);
    goal_linvel_x_ = cos(headingAngleToGoal)* max_forward_linear_velocity_; // m/s
    goal_linvel_y_ = sin(headingAngleToGoal)* max_forward_linear_velocity_; // m/s

    // if the goal is BEHIND, then walk BACKWARDS towards the goal (if enabled)
    if ((deltaPose.translation().x()<0) && (goal_behind_mode_ == 1 )){
      goal_linvel_x_ = -goal_linvel_x_;
      goal_linvel_y_ = -goal_linvel_y_;
    }

    ROS_INFO_THROTTLE(1, "FORWARD: %f to goal in m (%f thresh). (error x: %f, y: %f, z: %f)", dist_to_goal_in_2d_, goal_distance_threshold_, deltaPose.translation().x(), deltaPose.translation().y(), deltaPose.translation().z());   
  }else if(state == State::TURN_TO_DESTINATION){
    goal_angvel_z_ = -desHeadingError * angular_gain_p_;
    clipValue(goal_angvel_z_, max_angular_velocity_);
    goal_linvel_x_ = cos(headingAngleToGoal)* max_forward_linear_velocity_; // m/s
    goal_linvel_y_ = sin(headingAngleToGoal)* max_forward_linear_velocity_; // m/s

    // if the goal is BEHIND, then walk BACKWARDS towards the goal (if enabled)
    if ((deltaPose.translation().x()<0) && (goal_behind_mode_ == 1 )){
      goal_linvel_x_ = -goal_linvel_x_;
      goal_linvel_y_ = -goal_linvel_y_;
    }

    if (yaw_exclusive_turns_) {
      goal_linvel_x_ = 0;
      goal_linvel_y_ = 0;
    }

    ROS_INFO_THROTTLE(1, "TRN2DES: %f to goal in m (%f thresh). %f in rad (%f thresh).", dist_to_goal_in_2d_, goal_distance_threshold_, desHeadingError, goal_heading_threshold_);
  }else if (state == State::FINISHED){
    if (goals_.size() >0){
      ROS_INFO("Continuing to walk without stopping");
      follower_output = SEND_NOTHING;
    }

    goal_angvel_z_ = 0;
    goal_linvel_x_ = 0;
    goal_linvel_y_ = 0;
    finishedWithThisGoal = true;
    ROS_INFO_THROTTLE(1, "FINISHED %f to goal in rad. (des ang vel: %f)",desHeadingError, goal_angvel_z_);
  }else{
    ROS_INFO_THROTTLE(1, "ERROR UNKOWN STATE %d", static_cast<int>(state));
  }

  if (finishedWithThisGoal){
    ROS_INFO("Get next goal");
    bool continueCommanding = getNextGoal(current_pose);
    if (!continueCommanding){
      ROS_INFO("No new goal. Finished following goal");
      return SEND_STOP_WALKING;
      // used to send stop walking command there
    }
  }

  // Only use trackline objective when walking forward
  if (state== State::FORWARD)
    computeTracklineVelocityCommand(current_goal_in_fixed, current_pose);
  else{
    trackline_linvel_x_ = 0;
    trackline_linvel_y_ = 0;
  }

  // Velocity Limiting: scale x and y velocity proportionally to be below each threshold
  // Two objectives are combined equally at present. Introduce weighting here if needed
  double output_linvel_x = goal_linvel_x_ + trackline_linvel_x_;
  double output_linvel_y = goal_linvel_y_ + trackline_linvel_y_;

  if (max_forward_linear_velocity_ < 0.01){
    // avoid dividing by zero - causes trot controller to crash
    output_linear_velocity_ = Eigen::Vector3d(0, 0, 0);
    output_angular_velocity_ = Eigen::Vector3d(0, 0, 0) ;
    return SEND_COMMAND;
  }

  double max_forward_linear_velocity_now = 0;
  double max_lateral_linear_velocity_now = 0;
  if ((state == State::FORWARD) || (motion_mode_ == 1)){  // if walking straight or shuffling
    max_forward_linear_velocity_now = max_forward_linear_velocity_;
    max_lateral_linear_velocity_now = max_lateral_linear_velocity_;
  }else{
    max_forward_linear_velocity_now = max_turning_linear_velocity_;
    max_lateral_linear_velocity_now = max_turning_linear_velocity_;
  }

  double ratio_x = 1.0;
  // double check that the max linear velocity is not zero
  if (max_forward_linear_velocity_now != 0.0) 
  {
    ratio_x = fabs(output_linvel_x) / max_forward_linear_velocity_now;
  }
  else
  {
    ratio_x = 1.0;
  }

  if (ratio_x < 1)
    ratio_x = 1;

  output_linvel_x = output_linvel_x/ratio_x;
  output_linvel_y = output_linvel_y/ratio_x;


  double ratio_y = 1.0;
  // double check that the max lateral velocity is not zero
  if (max_lateral_linear_velocity_now != 0.0)
  {
    ratio_y = fabs(output_linvel_y) / max_lateral_linear_velocity_now;
  }
  else
  {
    ratio_y = 1.0;
  }

  if (ratio_y < 1)
    ratio_y = 1;
  output_linvel_y = output_linvel_y/ratio_y;
  output_linvel_x = output_linvel_x/ratio_y;

  // if we are considering the robot's front as the back, need to invert the
  // control commands on the x and y axes for it to move correctly.
  if (back_is_front_) {
    output_linear_velocity_ = Eigen::Vector3d(-output_linvel_x, -output_linvel_y, 0);
  } else {
    output_linear_velocity_ = Eigen::Vector3d(output_linvel_x, output_linvel_y, 0);
  }
  output_angular_velocity_ = Eigen::Vector3d(0, 0, goal_angvel_z_);
  return follower_output; 
}
