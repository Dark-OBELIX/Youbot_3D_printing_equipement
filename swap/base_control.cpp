#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <termios.h>
#include <unistd.h>

ros::Publisher platformPublisher;

// Function to read a key without waiting for "Enter"
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "youbot_base_keyboard_control");
    ros::NodeHandle n;

    platformPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    geometry_msgs::Twist twist;

    std::cout << "YouBot base control using keyboard!" << std::endl;
    std::cout << "Use the following keys:" << std::endl;
    std::cout << "Z : Move forward" << std::endl;
    std::cout << "S : Move backward" << std::endl;
    std::cout << "Q : Move left" << std::endl;
    std::cout << "D : Move right" << std::endl;
    std::cout << "A : Turn left" << std::endl;
    std::cout << "E : Turn right" << std::endl;
    std::cout << "X : Exit" << std::endl;

    double speed = 0.2;    // Translation speed
    double turnSpeed = 0.5; // Rotation speed

    while (ros::ok()) {
        char key = getKey();

        // Reset movement commands
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.angular.z = 0.0;

        // Control the robot based on pressed keys
        if (key == 'z') {
            twist.linear.x = speed; // Move forward
        } else if (key == 's') {
            twist.linear.x = -speed; // Move backward
        } else if (key == 'q') {
            twist.linear.y = speed; // Move left
        } else if (key == 'd') {
            twist.linear.y = -speed; // Move right
        } else if (key == 'a') {
            twist.angular.z = turnSpeed; // Turn left
        } else if (key == 'e') {
            twist.angular.z = -turnSpeed; // Turn right
        } else if (key == 'x') {
            break; // Exit the program
        }

        // Publish the commands
        platformPublisher.publish(twist);
        ros::spinOnce();
    }

    std::cout << "Stopping base control." << std::endl;
    ros::shutdown();
    return 0;
}
