#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include <vector>
#include <iostream>

// Fonction pour envoyer une trajectoire avec un contrôle de vitesse
void sendTrajectoryGoal(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &ac, 
                        const std::vector<double> &positions, 
                        double time_to_reach) {
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectoryPoint point;

    // Définition des noms des articulations
    for (int i = 0; i < positions.size(); i++) {
        std::stringstream jointName;
        jointName << "arm_joint_" << (i + 1);
        goal.trajectory.joint_names.push_back(jointName.str());
    }

    // Définition des positions des articulations
    point.positions = positions;
    point.velocities.assign(positions.size(), 0.0);  // Vélocités nulles
    point.accelerations.assign(positions.size(), 0.0); // Accélérations nulles
    point.time_from_start = ros::Duration(time_to_reach); // Temps total pour atteindre la position
    goal.trajectory.points.push_back(point);
    goal.trajectory.header.frame_id = "arm_link_0";
    goal.trajectory.header.stamp = ros::Time::now();

    // Envoi de l'objectif et attente du résultat
    ac.sendGoal(goal);
    ac.waitForResult(ros::Duration(time_to_reach + 5)); // Temps d'attente avec une marge

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Position atteinte avec succès.");
    else
        ROS_WARN("Impossible d'atteindre la position dans le temps imparti.");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controlled_arm_motion");
    ros::NodeHandle n;

    // Connexion au serveur d'actions
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("arm_1/arm_controller/follow_joint_trajectory", true);

    ROS_INFO("En attente du démarrage du serveur d'actions.");
    ac.waitForServer();
    ROS_INFO("Serveur d'actions démarré.");

    // Définition des positions
    std::vector<double> start_position = {0.0, 0.0, 0.0, 0.0, 0.0}; // Position initiale
    std::vector<double> target_position_1 = {4.52, 2.45, -2.17, 3.36, 0.0}; // Position 1

    double time_to_reach = 10.0; // Temps pour atteindre la position (en secondes, mouvement lent)

    // Déplacement vers la position initiale
    ROS_INFO("Déplacement vers la position initiale.");
    sendTrajectoryGoal(ac, start_position, time_to_reach);

    // Déplacement vers la position 1
    ROS_INFO("Déplacement vers la position 1 avec vitesse contrôlée.");
    sendTrajectoryGoal(ac, target_position_1, time_to_reach);

    ROS_INFO("Déplacement terminé.");

    return 0;
}
