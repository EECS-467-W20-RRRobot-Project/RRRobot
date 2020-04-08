#include <arm.h>
#include <iostream>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>

#include "ros/ros.h"
#include <simulation_env/arm_command.h>
#include <simulation_env/arm_angles.h>

using std::cout;
using std::endl;
using namespace KDL;

Arm arm("/home/rrrobot/rrrobot_src/src/gazebo_models/fanuc_robotic_arm/model.sdf");
KDL::Chain chain = arm.getArm();
KDL::Chain correct_chain;

void angle_callback(const simulation_env::arm_angles &msg)
{
    // Create solver based on kinematic chain
    static ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);

    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = KDL::JntArray(nj);

    jointpositions(0) = msg.shoulder_pivot_angle;
    jointpositions(1) = msg.shoulder_joint_angle;
    jointpositions(2) = msg.elbow_joint_angle;
    jointpositions(3) = msg.wrist_pivot_angle;
    jointpositions(4) = msg.wrist_joint_angle;
    jointpositions(5) = msg.end_effector_pivot_angle;

    // Create the frame that will contain the results
    KDL::Frame cartpos;

    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions, cartpos);
    if (kinematics_status >= 0)
    {
        std::cout << cartpos << std::endl;
        //printf("%s \n", "Succes, thanks KDL!");
    }
    else
    {
        //printf("%s \n", "Error: could not calculate forward kinematics :(");
    }
}

int main(int argc, char **argv)
{
    cout << "Starting arm controller..." << endl;
    ros::init(argc, argv, "arm_control_test");

    // correct.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0.0))));
    // correct.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0, 0, 1))));
    // correct.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0, 0, 1))));

    ros::NodeHandle nh;
    // publisher = nh.advertise<simulation_env::arm_command>("/arm_node/arm_commands", 1000);
    ros::Subscriber sub = nh.subscribe("/arm_node/arm_positions", 1000, angle_callback);

    // Assign some values to the joint positions
    // for (unsigned int i = 0; i < nj; i++)
    // {
    //     float myinput;
    //     // printf("Enter the position of joint %i: ", i);
    //     // scanf("%e", &myinput);
    //     jointpositions(i) = (double)0.0;
    // }

    ros::spin();
    cout << "Arm controller finished." << endl;
}