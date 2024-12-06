#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    ROS_INFO("Received Joint State:");
    
    // Affiche le nom de chaque joint et sa position correspondante
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        ROS_INFO("Joint: %s, Position: %f", msg->name[i].c_str(), msg->position[i]);
    }
    ROS_INFO("------"); // Séparation entre les messages
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_state_listener");
    ros::NodeHandle nh;

    // Souscrire au topic /joint_states
    ros::Subscriber sub = nh.subscribe("/joint_states", 1000, jointStateCallback);

    // Boucle ROS pour recevoir les messages en continu
    ros::spin();

    return 0;
}
// rosrun youbot_ros_hello_world debug_node 