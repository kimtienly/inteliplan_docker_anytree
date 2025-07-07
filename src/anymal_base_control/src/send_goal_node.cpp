#include <anymal_base_control/navigate_to_goal.hpp>
#include <anymal_base_control/motion_transition.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "navigate_to_goal_node");

//     MotionTransitionerAnymal motion;

// //   motion.sendGoal("stand", "height:0.3;orientation:neutral");
//   motion.sendGoal("walk");
//   if (motion.waitForResult()) {
//     if (motion.getResult()) {
//       ROS_INFO("Motion transition succeeded.");
      NavigateToGoalAnymal nav_client;
        //   nav_client.waitForServer();

        geometry_msgs::Pose pose;
        pose.position.x = 0.5;
        pose.position.y = 0.0;
        pose.position.z = 0.57;
        pose.orientation.w = 1.0;  // Identity quaternion

        nav_client.sendGoal(pose, "odom");



        if (nav_client.waitForResult()) {
            if (nav_client.getResult()) {
            ROS_INFO("Successfully reached goal!");
            } else {
            ROS_WARN("Failed to reach goal.");
            }
          } else {
            ROS_ERROR("Timed out waiting for result.");
          }
//     } else {
//       ROS_WARN("Motion transition failed.");
//     }
//   } else {
//     ROS_ERROR("Timeout waiting for motion transition result.");
//   }



  

  return 0;
}
