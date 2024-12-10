#include "ros/ros.h"
#include "brics_actuator/JointPositions.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <vector>
#include <termios.h>
#include <unistd.h>

ros::Publisher armPublisher;
std::vector<double> jointPositions(5, 0.0); // Positions des joints initialisées à zéro
bool positionsInitialized = false; // Indique si les positions ont été initialisées

// Fonction pour lire les entrées clavier sans attendre un appui sur Entrée
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

// Créer un message de commande pour positionner un joint spécifique
brics_actuator::JointPositions createArmPositionCommand(int jointIndex, double position) {
    brics_actuator::JointPositions msg;
    brics_actuator::JointValue joint;

    joint.timeStamp = ros::Time::now();
    joint.value = position;
    joint.unit = "rad"; // Utiliser les radians pour les angles

    // Nommer le joint
    std::stringstream jointName;
    jointName << "arm_joint_" << jointIndex;
    joint.joint_uri = jointName.str();

    msg.positions.push_back(joint);
    return msg;
}

// Callback pour récupérer les positions actuelles des joints
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // Synchroniser les positions initiales uniquement
    if (!positionsInitialized && msg->position.size() >= 5) {
        for (int i = 0; i < 5; ++i) {
            jointPositions[i] = msg->position[i];
        }
        positionsInitialized = true;
        ROS_INFO("Positions des joints initialisées à partir de joint_states.");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "youbot_arm_keyboard_control");
    ros::NodeHandle n;

    armPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 1);
    ros::Subscriber jointStateSubscriber = n.subscribe("joint_states", 10, jointStatesCallback);

    int selectedJoint = 1; // Commencer avec le premier joint sélectionné (arm_joint_1)
    double step = 0.1; // Incrément de 0.1 radian par pression de touche

    std::cout << "Contrôlez le bras YouBot avec le clavier !" << std::endl;
    std::cout << "Appuyez sur 1-5 pour sélectionner un moteur" << std::endl;
    std::cout << "Appuyez sur 'q' pour diminuer l'angle, 'd' pour l'augmenter" << std::endl;
    std::cout << "Appuyez sur 'e' pour afficher les positions actuelles des joints" << std::endl;
    std::cout << "Appuyez sur 's' pour arrêter le mouvement du joint sélectionné" << std::endl;
    std::cout << "Appuyez sur 'x' pour quitter" << std::endl;

    // Attendre que les positions initiales soient récupérées
    ros::Rate rate(10); // 10 Hz
    while (ros::ok() && !positionsInitialized) {
        ros::spinOnce();
        rate.sleep();
    }

    if (!ros::ok()) {
        std::cout << "ROS arrêté avant l'initialisation des positions des joints." << std::endl;
        return 0;
    }

    while (ros::ok()) {
        char key = getKey();

        // Sélectionner un joint (1 à 5)
        if (key >= '1' && key <= '5') {
            selectedJoint = key - '0'; // Convertir le caractère en entier (1-5)
            std::cout << "Joint sélectionné : arm_joint_" << selectedJoint << std::endl;
        }
        // Diminuer l'angle du joint sélectionné
        else if (key == 'q') {
            jointPositions[selectedJoint - 1] -= step;
            std::cout << "arm_joint_" << selectedJoint << " = " << jointPositions[selectedJoint - 1] << " rad" << std::endl;
        }
        // Augmenter l'angle du joint sélectionné
        else if (key == 'd') {
            jointPositions[selectedJoint - 1] += step;
            std::cout << "arm_joint_" << selectedJoint << " = " << jointPositions[selectedJoint - 1] << " rad" << std::endl;
        }
        // Afficher les positions actuelles des joints
        else if (key == 'e') {
            std::cout << "Positions actuelles des joints :" << std::endl;
            for (int i = 0; i < jointPositions.size(); ++i) {
                std::cout << "arm_joint_" << (i + 1) << " = " << jointPositions[i] << " rad" << std::endl;
            }
        }
        // Réinitialiser l'angle du joint à zéro
        else if (key == 's') {
            jointPositions[selectedJoint - 1] = 0;
            std::cout << "arm_joint_" << selectedJoint << " arrêté." << std::endl;
        }
        // Quitter le programme
        else if (key == 'x') {
            break;
        }

        // Créer la commande et publier la position actuelle
        brics_actuator::JointPositions msg = createArmPositionCommand(selectedJoint, jointPositions[selectedJoint - 1]);
        armPublisher.publish(msg);

        ros::spinOnce();
    }

    std::cout << "Arrêt du contrôle du bras." << std::endl;
    ros::shutdown();
    return 0;
}
