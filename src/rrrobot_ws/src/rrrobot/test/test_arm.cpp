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

// class ArmRepresentation
// {
// public:
//     ArmRepresentation(const KDL::Frame &base_pose = KDL::Frame(KDL::Vector(0.3, 0.92, 1)))
//     {
//         const double base_len = 0.2;
//         const double shoulder_height = 0.1273;
//         const double upper_arm_length = 0.612;
//         const double forearm_length = 0.5723;
//         const double shoulder_offset = 0.220941;
//         const double elbow_offset = -0.1719;
//         const double wrist_1_length = 0.163941 - elbow_offset - shoulder_offset;
//         const double wrist_2_length = 0.1157;
//         const double wrist_3_length = 0.0922;

//         // KDL::Vector pos; //(0, 0, base_len/2);
//         // KDL::Rotation rot;

//         // The joints might be off by one
//         KDL::Segment linear_arm_actuator("linear_arm_actuator_joint",
//                                          KDL::Joint(KDL::Joint::JointType::None), base_pose);
//         arm.addSegment(linear_arm_actuator);

//         const KDL::Vector pos_base(0, 0, base_len / 2);
//         const KDL::Rotation rot_base(KDL::Rotation::RPY(0, 0, 0));
//         KDL::Segment base_link("base_link", KDL::Joint(KDL::Joint::JointType::TransY),
//                                KDL::Frame(rot_base, pos_base));
//         arm.addSegment(base_link);

//         const KDL::Vector pos_shoulder(0, 0, shoulder_height);
//         const KDL::Rotation rot_shoulder(KDL::Rotation::RPY(0, 0, 0));
//         KDL::Segment shoulder_link("shoulder_link", KDL::Joint(KDL::Joint::JointType::RotZ),
//                                    KDL::Frame(rot_shoulder, pos_shoulder));
//         arm.addSegment(shoulder_link);

//         const KDL::Vector pos_upper_arm(0, shoulder_offset, 0);
//         const KDL::Rotation rot_upper_arm(KDL::Rotation::RPY(0.0, M_PI / 2.0, 0.0));
//         KDL::Segment upper_arm_link("upper_arm_link", KDL::Joint(KDL::Joint::JointType::RotY),
//                                     KDL::Frame(rot_upper_arm, pos_upper_arm));
//         arm.addSegment(upper_arm_link);

//         const KDL::Vector pos_forearm(0, elbow_offset, upper_arm_length);
//         const KDL::Rotation rot_forearm(KDL::Rotation::RPY(0.0, 0.0, 0.0));
//         KDL::Segment forearm_link("forearm_link", KDL::Joint(KDL::Joint::JointType::RotY),
//                                   KDL::Frame(rot_forearm, pos_forearm));
//         arm.addSegment(forearm_link);

//         const KDL::Vector pos_wrist_1(0, 0, forearm_length);
//         const KDL::Rotation rot_wrist_1(KDL::Rotation::RPY(0.0, M_PI / 2.0, 0.0));
//         KDL::Segment wrist_1_link("wrist_1_link", KDL::Joint(KDL::Joint::JointType::RotY),
//                                   KDL::Frame(rot_wrist_1, pos_wrist_1));
//         arm.addSegment(wrist_1_link);

//         const KDL::Vector pos_wrist_2(0, wrist_1_length, 0);
//         const KDL::Rotation rot_wrist_2(KDL::Rotation::RPY(0.0, 0.0, 0.0));
//         KDL::Segment wrist_2_link("wrist_2_link", KDL::Joint(KDL::Joint::JointType::RotZ),
//                                   KDL::Frame(rot_wrist_2, pos_wrist_2));
//         arm.addSegment(wrist_2_link);

//         const KDL::Vector pos_wrist_3(0, 0, wrist_2_length);
//         const KDL::Rotation rot_wrist_3(KDL::Rotation::RPY(0.0, 0.0, 0.0));
//         KDL::Segment wrist_3_link("wrist_3_link", KDL::Joint(KDL::Joint::JointType::RotY),
//                                   KDL::Frame(rot_wrist_3, pos_wrist_3));
//         arm.addSegment(wrist_3_link);

//         const KDL::Vector pos_ee(0, wrist_3_length, 0.0);
//         const KDL::Rotation rot_ee(KDL::Rotation::RPY(0.0, 0.0, M_PI / 2.0));
//         KDL::Segment ee_link("ee_link", KDL::Joint(KDL::Joint::JointType::None),
//                              KDL::Frame(rot_ee, pos_ee));
//         arm.addSegment(ee_link);

//         // arm.addSegment(base_link);
//         // arm.addSegment(shoulder_link);
//         // arm.addSegment(upper_arm_link);
//         // arm.addSegment(forearm_link);
//         // arm.addSegment(wrist_1_link);
//         // arm.addSegment(wrist_2_link);
//         // arm.addSegment(wrist_3_link);
//         // arm.addSegment(ee_link);

//         fk_solver.reset(new KDL::ChainFkSolverPos_recursive(arm));
//         ik_solver.reset(new KDL::ChainIkSolverPos_LMA(arm));
//     }

//     int calculateForwardKinematics(const KDL::JntArray &joint_positions, KDL::Frame &end_effector_pose)
//     {
//         return fk_solver->JntToCart(joint_positions, end_effector_pose);
//     }

//     int calculateInverseKinematics(const KDL::JntArray &cur_configuration,
//                                    const KDL::Frame &desired_end_effector_pose,
//                                    KDL::JntArray &final_joint_configuration)
//     {
//         return ik_solver->CartToJnt(cur_configuration, desired_end_effector_pose, final_joint_configuration);
//     }

//     string *get_joint_names()
//     {
//         // TODO: return ["linear_arm_actuator_joint",
//         //         "shoulder_pan_joint",
//         //         "shoulder_lift_joint",
//         //         "elbow_joint",
//         //         "wrist_1_joint",
//         //         "wrist_2_joint",
//         //         "wrist_3_joint"];
//     }

//     const KDL::Chain &getArm() const
//     {
//         return arm;
//     }

// private:
//     KDL::Chain arm;

//     std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
//     std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver;
// };

int main(int argc, char **argv)
{
    ArmRepresentation arm;

    KDL::JntArray pos(arm.getChain()->getNrOfJoints());
    KDL::SetToZero(pos);

    int joint;

    // std::cout << "Enter joint to exercise: ";
    // std::cin >> joint;

    KDL::Frame end_effector_pose;
    std::ofstream f("data.txt");
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