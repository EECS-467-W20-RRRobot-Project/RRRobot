// arm_controller_node.cpp

#include <algorithm>
#include <vector>
#include <string>
#include <memory>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <osrf_gear/VacuumGripperControl.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "topic_names.h"
#include "rrrobot/arm_command.h"

#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/rotationalinertia.hpp>
#include <kdl/joint.hpp>
#include <kdl/frames.hpp>
#include <LinearMath/btTransform.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/frames_io.hpp>

#include <fstream>
#include <iostream>
#include <string>

#include "arm_representation.h"

using namespace std;


int main(int argc, char **argv)
{
    ArmRepresentation arm;

    KDL::JntArray pos(arm.getChain()->getNrOfJoints());
    KDL::SetToZero(pos);

    int joint;

    // std::cout << "Enter joint to exercise: ";
    // std::cin >> joint;

    KDL::Frame end_effector_pose;
    std::ofstream f("arm_test.debug");
    int error_code;
    for (joint = 0; joint < arm.getChain()->getNrOfJoints(); ++joint)
    {
        for (double pos_val = 0.0; pos_val <= 2 * M_PI; pos_val += 0.1)
        {
            pos(joint) = pos_val;
            error_code = arm.calculateForwardKinematics(pos, end_effector_pose);

            if (error_code != 0)
            {
                ROS_ERROR("Forward Kinematics Failure: %i", error_code);
            }

            f << end_effector_pose.p.x() << "," << end_effector_pose.p.y() << "," << end_effector_pose.p.z() << '\n';

            error_code = arm.calculateInverseKinematics(vector<double>(arm.getChain()->getNrOfJoints(), 0.0), end_effector_pose, pos);

            if (error_code != 0)
            {
                ROS_ERROR("Inverse Kinematics Failure: %i", error_code);
            }
        }
    }

    double linear_arm_actuator_joint, shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint;

    while (true)
    {
        cout << "linear_arm_actuator_joint: ";
        cin >> pos(0); //linear_arm_actuator_joint;
        cout << "shoulder_pan_joint: ";
        cin >> pos(1); //shoulder_pan_joint;
        cout << "shoulder_lift_joint: ";
        cin >> pos(2); //shoulder_lift_joint;
        cout << "elbow_joint: ";
        cin >> pos(3); //elbow_joint;
        cout << "wrist_1_joint: ";
        cin >> pos(4); //wrist_1_joint;
        cout << "wrist_2_joint: ";
        cin >> pos(5); //wrist_2_joint;
        cout << "wrist_3_joint: ";
        cin >> pos(6); //wrist_3_joint;

        error_code = arm.calculateForwardKinematics(pos, end_effector_pose);

        if (error_code != 0)
        {
            ROS_ERROR("Forward kinematics failure: %i", error_code);
        }

        cout << "x,y,z: " << end_effector_pose.p.x() << ", " << end_effector_pose.p.y() << ", " << end_effector_pose.p.z() << endl;
        cout << end_effector_pose.M << endl;
    }
}