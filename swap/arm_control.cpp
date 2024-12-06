#include "ros/ros.h"
#include "brics_actuator/JointPositions.h"
#include <iostream>
#include <vector>
#include <termios.h>
#include <unistd.h>

ros::Publisher armPublisher;

// Function to read keyboard input without waiting for user to press enter
char getKey() {
    struct termios oldt, newt;
    char c;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    c = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return c;
}

// Create a command message to position a specific joint
brics_actuator::JointPositions createArmPositionCommand(int jointIndex, double position) {
    brics_actuator::JointPositions msg;
    brics_actuator::JointValue joint;
    
    joint.timeStamp = ros::Time::now();
    joint.value = position;
    joint.unit = "rad"; // Use radians for angles

    // Define the joint name
    std::stringstream jointName;
    jointName << "arm_joint_" << jointIndex;
    joint.joint_uri = jointName.str();
    
    msg.positions.push_back(joint);
    return msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "youbot_arm_keyboard_control");
    ros::NodeHandle n;

    armPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);

    std::vector<double> jointPositions(5, 0.0); // Initialize all positions to 0
    int selectedJoint = 1; // Start with the first joint selected (arm_joint_1)
    double step = 0.1; // Increment by 0.1 radian for each key press

    std::cout << "Control the YouBot arm using the keyboard!" << std::endl;
    std::cout << "Press 1-5 to select a motor" << std::endl;
    std::cout << "Press 'q' to decrease the angle, 'd' to increase it" << std::endl;
    std::cout << "Press 's' to stop the selected joint's movement" << std::endl;
    std::cout << "Press 'x' to exit" << std::endl;

    while (ros::ok()) {
        char key = getKey();

        // Select joint (1 to 5)
        if (key >= '1' && key <= '5') {
            selectedJoint = key - '0'; // Convert char to int (1-5)
            std::cout << "Selected joint: arm_joint_" << selectedJoint << std::endl;
        } 
        // Decrease angle of selected joint
        else if (key == 'q') {
            jointPositions[selectedJoint - 1] -= step;
            std::cout << "arm_joint_" << selectedJoint << " = " << jointPositions[selectedJoint - 1] << " rad" << std::endl;
        }
        // Increase angle of selected joint
        else if (key == 'd') {
            jointPositions[selectedJoint - 1] += step;
            std::cout << "arm_joint_" << selectedJoint << " = " << jointPositions[selectedJoint - 1] << " rad" << std::endl;
        }
        // Reset joint angle to zero
        else if (key == 's') {
            jointPositions[selectedJoint - 1] = 0;
            std::cout << "arm_joint_" << selectedJoint << " stopped." << std::endl;
        }
        // Display all joint positions
        else if (key == 'e') { 
            std::cout << "Current joint positions:" << std::endl;
            for (int i = 0; i < jointPositions.size(); ++i) {
                std::cout << "arm_joint_" << (i + 1) << " = " << jointPositions[i] << " rad" << std::endl;
            }
        }
        // Exit the program
        else if (key == 'x') {
            break;
        }

        // Create the command and publish the current position
        brics_actuator::JointPositions msg = createArmPositionCommand(selectedJoint, jointPositions[selectedJoint - 1]);
        armPublisher.publish(msg);
        
        ros::spinOnce();
    }


    std::cout << "Stopping arm control." << std::endl;
    ros::shutdown();
    return 0;
}
