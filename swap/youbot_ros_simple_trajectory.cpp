#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include <vector>

// Fonction pour créer et envoyer un objectif de trajectoire
void sendTrajectoryGoal(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &ac, 
                        const std::vector<double> &positions, 
                        double wait_time) {
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectoryPoint point;

    // Définir les noms des articulations
    for (int i = 0; i < 5; i++) {
        std::stringstream jointName;
        jointName << "arm_joint_" << (i + 1);
        goal.trajectory.joint_names.push_back(jointName.str());
    }

    // Définir les positions des articulations
    for (const double &pos : positions) {
        point.positions.push_back(pos);
        point.velocities.push_back(0.0);  // Peut rester à 0.0 car le temps contrôle la vitesse
        point.accelerations.push_back(0.0);
    }

    point.time_from_start = ros::Duration(wait_time);  // Plus grand = mouvement plus lent
    goal.trajectory.points.push_back(point);
    goal.trajectory.header.frame_id = "arm_link_0";
    goal.trajectory.header.stamp = ros::Time::now();

    ac.sendGoal(goal);
    ac.waitForResult(ros::Duration(wait_time + 5)); // Temps tampon

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Position atteinte avec succès.");
    else
        ROS_WARN("Échec de l'atteinte de la position dans le temps imparti.");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "youbot_go_to_position_1");
    ros::NodeHandle n;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("arm_1/arm_controller/follow_joint_trajectory", true);

    ROS_INFO("Attente du démarrage du serveur d'actions.");
    ac.waitForServer();
    ROS_INFO("Serveur d'actions démarré.");

    // Position 1 à atteindre
    std::vector<double> position1 = {1.2, 2.7, -3, 0.8, 0.5}; 
    double wait_time = 5.0; // Attente prolongée pour ralentir le mouvement

    // Envoyer l'objectif
    sendTrajectoryGoal(ac, position1, wait_time);

    return 0;
}
