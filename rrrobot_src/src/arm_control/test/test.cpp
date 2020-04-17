// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <arm.h>
#include <fstream>
#include <string>
#include <vector>

using namespace KDL;
using std::cin;
using std::cout;
using std::endl;
using std::ofstream;
using std::string;
using std::vector;

int main(int argc, char **argv)
{
    //Definition of a kinematic chain & add segments to the chain
    Arm arm(/*"/home/rrrobot/rrrobot_src/src/gazebo_models/basic_arm/model.sdf"); /(*/ "/home/rrrobot/rrrobot_src/src/gazebo_models/fanuc_robotic_arm/model.sdf");
    //KDL::Chain chain = arm.getArm();

    // chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0.0))));
    // chain.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0, 0, 1))));
    // chain.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0, 0, 1))));
    // chain.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0.0, 0.0, 0.480))));
    // chain.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0.0, 0.0, 0.645))));
    // chain.addSegment(Segment(Joint(Joint::RotZ)));
    // chain.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0.0, 0.0, 0.120))));
    // chain.addSegment(Segment(Joint(Joint::RotZ)));

    // Create solver based on kinematic chain
    //ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

    // Create joint array
    unsigned int nj = arm.getArm().getNrOfJoints();
    KDL::JntArray jointpositions = JntArray(nj);
    for (int pos = 0; pos < nj; ++pos)
    {
        jointpositions(pos) = 0;
    }

    // // Assign some values to the joint positions for (unsigned int i = 0; i < nj; i++)
    // for (int i = 0; i < nj; ++i)
    // {
    //     float myinput;
    //     printf("Enter the position of joint %i: ", i);
    //     scanf("%e", &myinput);
    //     jointpositions(i) = (double)myinput;
    // }
    // // Create the frame that will contain the results
    // KDL::Frame cartpos;

    // // Calculate forward position kinematics
    // bool kinematics_status;
    // kinematics_status = fksolver.JntToCart(jointpositions, cartpos);
    // if (kinematics_status >= 0)
    // {
    //     // cout << segments << endl;
    //     std::cout << cartpos << std::endl;
    //     // printf("%s \n", "Succes, thanks KDL!");
    // }
    // else
    // {
    //     // printf("%s \n", "Error: could not calculate forward kinematics :(");
    // }
    // while (1)
    // {
    //     cout << "Enter the shoulder_pivot angle: ";
    //     cin >> jointpositions(0);
    //     cout << "Enter the shoulder_joint angle: ";
    //     cin >> jointpositions(1);
    //     cout << "Enter the elbow_joint angle: ";
    //     cin >> jointpositions(2);
    //     cout << "Enter the wrist_pivot angle: ";
    //     cin >> jointpositions(3);
    //     cout << "Enter the wrist_joint angle: ";
    //     cin >> jointpositions(4);
    //     cout << "Enter the end_effector_pivot angle: ";
    //     cin >> jointpositions(5);

    // Create the frame that will contain the results
    KDL::Frame cartpos;
    ofstream f_raw("configurations.txt");
    ofstream f_corrected("corrected_configurations.txt");
    int which_joint;
    cout << "Which joint should be moved (0: shoulder pivot, 5: end effector pivot): ";
    cin >> which_joint;

    for (double shoulder_pivot = 0.0; shoulder_pivot <= 6.28; shoulder_pivot += 0.1)
    {
        // Calculate forward position kinematics
        bool kinematics_status;
        // for (int segments = 0; segments <= chain.getNrOfSegments(); ++segments)
        // {
        //     kinematics_status = fksolver.JntToCart(jointpositions, cartpos, segments);
        //     if (kinematics_status >= 0)
        //     {
        //         cout << segments << endl;
        //         std::cout << cartpos << std::endl;
        //         // printf("%s \n", "Succes, thanks KDL!");
        //     }
        //     else
        //     {
        //         // printf("%s \n", "Error: could not calculate forward kinematics :(");
        //     }
        // }
        jointpositions(which_joint) = shoulder_pivot;
        kinematics_status = arm.calculateForwardKinematics(jointpositions, cartpos); //fksolver.JntToCart(jointpositions, cartpos);
        cout << "Final position: " << endl;
        cout << cartpos << endl;

        KDL::Vector pos = cartpos.p;
        f_raw << pos.x() << "," << pos.y() << "," << pos.z() << "\n";

        KDL::Vector corrected = cartpos.M * cartpos.p;
        f_corrected << corrected.x() << "," << corrected.y() << "," << corrected.z() << "\n";
    }
    // }

    KDL::Frame desired(KDL::Vector(0.000767, -1.87014, 2.0658));
    KDL::JntArray final_config(arm.getArm().getNrOfJoints());
    bool error = arm.calculateInverseKinematics(jointpositions, desired, final_config);
    if (error == false)
    {
        for (int joint = 0; joint < arm.getArm().getNrOfJoints(); ++joint)
        {
            cout << final_config(joint) << endl;
        }
    }
    else
    {
        cout << "Failed to find configuration" << endl;
    }

    KDL::JntArray vel(arm.getArm().getNrOfJoints());
    KDL::JntArray accel(arm.getArm().getNrOfJoints());
    KDL::Wrenches ext_force(arm.getArm().getNrOfSegments());
    KDL::JntArray required_force(arm.getArm().getNrOfJoints());
    int error_val = arm.calculateInverseDynamics(jointpositions, vel, accel, ext_force, required_force);
    if (error_val == 0)
    {
        for (int joint = 0; joint < arm.getArm().getNrOfJoints(); ++joint)
        {
            cout << required_force(joint) << endl;
        }
    }
    else
    {
        cout << "Failed to find required torques, error #" << error_val << endl;
    }

    // record required torques for a variety of joint positions
    KDL::JntArray pos(arm.getArm().getNrOfJoints());
    for (int joint = 0; joint < arm.getArm().getNrOfJoints(); ++joint)
    {
        pos(joint) = 0;
    }

    ofstream f("required_" + std::to_string(which_joint) + "_torque.txt");
    for (double angle = -3.14159; angle < 3.14159; angle += 0.1)
    {
        pos(which_joint) = angle;
        arm.calculateInverseDynamics(pos, vel, accel, ext_force, required_force);

        f << angle << "," << required_force(0) << "," << required_force(1) << "," << required_force(2) << "," << required_force(3) << "," << required_force(4) << "," << required_force(5) << "\n";
    }

    // KDL::JntArray pos(arm.getArm().getNrOfJoints());
    KDL::JntArray required_angles(arm.getArm().getNrOfJoints());

    KDL::SetToZero(pos);
    KDL::SetToZero(required_angles);

    vector<KDL::Frame> desired_positions;
    desired_positions.push_back(KDL::Frame(KDL::Vector(1, -0.5, 1)));
    desired_positions.push_back(KDL::Frame(KDL::Vector(1, -0.4, 1)));
    desired_positions.push_back(KDL::Frame(KDL::Vector(1, -0.3, 1)));
    desired_positions.push_back(KDL::Frame(KDL::Vector(1, -0.2, 1)));
    desired_positions.push_back(KDL::Frame(KDL::Vector(1, -0.1, 1)));
    desired_positions.push_back(KDL::Frame(KDL::Vector(1, 0.0, 1)));
    desired_positions.push_back(KDL::Frame(KDL::Vector(1, 0.1, 1)));
    desired_positions.push_back(KDL::Frame(KDL::Vector(1, 0.2, 1)));
    desired_positions.push_back(KDL::Frame(KDL::Vector(1, 0.3, 1)));
    desired_positions.push_back(KDL::Frame(KDL::Vector(1, 0.4, 1)));
    desired_positions.push_back(KDL::Frame(KDL::Vector(1, 0.5, 1)));

    ofstream ik_f("inverse_kinematics.txt");
    for (int state = 0; state < desired_positions.size(); ++state)
    {
        arm.calculateInverseKinematics(pos, desired_positions[state], required_angles);

        ik_f << desired_positions[state].p.x() << "," << desired_positions[state].p.y() << "," << desired_positions[state].p.z() << ",";
        ik_f << required_angles(0) << "," << required_angles(1) << "," << required_angles(2) << "," << required_angles(3) << "," << required_angles(4) << "," << required_angles(5) << "\n";
    }
}