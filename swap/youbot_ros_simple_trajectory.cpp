#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include <vector>

// Function to create and send a trajectory goal
void sendTrajectoryGoal(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &ac, 
                        const std::vector<double> &positions, 
                        double wait_time) {
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectoryPoint point;

    // Define joint names
    for (int i = 0; i < 5; i++) {
        std::stringstream jointName;
        jointName << "arm_joint_" << (i + 1);
        goal.trajectory.joint_names.push_back(jointName.str());
    }

    // Define joint positions
    for (const double &pos : positions) {
        point.positions.push_back(pos);
        point.velocities.push_back(0.0);  // Can remain at 0.0 as time controls speed
        point.accelerations.push_back(0.0);
    }

    point.time_from_start = ros::Duration(wait_time);  // Larger value = slower movement
    goal.trajectory.points.push_back(point);
    goal.trajectory.header.frame_id = "arm_link_0";
    goal.trajectory.header.stamp = ros::Time::now();

    ac.sendGoal(goal);
    ac.waitForResult(ros::Duration(wait_time + 5)); // Buffer time

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Target position reached successfully.");
    else
        ROS_WARN("Failed to reach the target position within the allotted time.");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "youbot_go_to_position_1");
    ros::NodeHandle n;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("arm_1/arm_controller/follow_joint_trajectory", true);

    ROS_INFO("Waiting for the action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started.");

    // Target position 1
    std::vector<double> position1 = {1.2, 2.7, -3, 0.8, 0.5}; 
    double wait_time = 5.0; // Extended wait time for slower movement

    // Send the goal
    sendTrajectoryGoal(ac, position1, wait_time);

    return 0;
}
