#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include <vector>
#include <iostream>

// Function to create and send a trajectory goal
void sendTrajectoryGoal(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &ac, 
                        const std::vector<double> &positions, 
                        double wait_time) {
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectoryPoint point;

    // Define joint names
    for (int i = 0; i < positions.size(); i++) {
        std::stringstream jointName;
        jointName << "arm_joint_" << (i + 1);
        goal.trajectory.joint_names.push_back(jointName.str());
    }

    // Define joint positions
    point.positions = positions;
    point.velocities.assign(positions.size(), 0.0);  // Set velocities to 0.0
    point.accelerations.assign(positions.size(), 0.0); // Set accelerations to 0.0
    point.time_from_start = ros::Duration(wait_time);  // Larger value = slower movement
    goal.trajectory.points.push_back(point);
    goal.trajectory.header.frame_id = "arm_link_0";
    goal.trajectory.header.stamp = ros::Time::now();

    // Send the goal and wait for the result
    ac.sendGoal(goal);
    ac.waitForResult(ros::Duration(wait_time + 5)); // Buffer time

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Target position reached successfully.");
    else
        ROS_WARN("Failed to reach the target position within the allotted time.");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "youbot_select_position");
    ros::NodeHandle n;

    // Connect to the action server
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("arm_1/arm_controller/follow_joint_trajectory", true);

    ROS_INFO("Waiting for the action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started.");

    // Define the positions
    std::vector<std::vector<double>> positions = {
        {4.4, 2.1, -0.9, 2.2},  // Position 1
        {4.45, 2.7, -2.0, 2.8}, // Position 2
        {5.17, 2.7, -2.0, 2.7}, // Position 3
        {5.2, 2.2, -1.1, 2.1}   // Position 4
    };

    std::vector<double> neutral_position = {0.0, 0.0, 0.0, 0.0}; // Neutral position
    double neutral_wait_time = 0.5; // Half a second for neutral position
    double wait_time = 5.0; // Adjust wait time for slower movements

    while (ros::ok()) {
        int input;
        std::cout << "Enter the target position (1, 2, 3, 4) or 0 to exit: ";
        std::cin >> input;

        if (input == 0) {
            std::cout << "Exiting the program." << std::endl;
            break;
        }

        if (input >= 1 && input <= 4) {
            // Move to neutral position first
            ROS_INFO("Moving to neutral position.");
            sendTrajectoryGoal(ac, neutral_position, neutral_wait_time);

            // Move to the selected position
            ROS_INFO("Moving to position %d", input);
            sendTrajectoryGoal(ac, positions[input - 1], wait_time);
        } else {
            std::cout << "Invalid input. Please enter a number between 1 and 4." << std::endl;
        }
    }

    return 0;
}
