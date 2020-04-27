// arm_representation.cpp

#include "arm_representation.h"
#include <math.h>

using namespace std;

// Constructor
ArmRepresentation::ArmRepresentation(const KDL::Frame &base_pose)
{
    // Parameters for coordinate transformations of robot links and joints
    const double base_len = 0.2;
    const double shoulder_height = 0.1273;
    const double upper_arm_length = 0.612;
    const double forearm_length = 0.5723;
    const double shoulder_offset = 0.220941;
    const double elbow_offset = -0.1719;
    const double wrist_1_length = 0.163941 - elbow_offset - shoulder_offset;
    const double wrist_2_length = 0.1157;
    const double wrist_3_length = 0.0922;

    // Define chain of links and joints using KDL
    KDL::Segment linear_arm_actuator("linear_arm_actuator_joint",
                                     KDL::Joint(KDL::Joint::JointType::None), base_pose);
    chain.addSegment(linear_arm_actuator);

    const KDL::Vector pos_base(0, 0, base_len / 2);
    const KDL::Rotation rot_base(KDL::Rotation::Quaternion(0, 0, 0, 1));
    KDL::Segment base_link("base_link", KDL::Joint(KDL::Joint::JointType::None), //KDL::Joint("linear_arm_actuator_joint", KDL::Joint::JointType::TransY),
                           KDL::Frame(rot_base, pos_base));
    chain.addSegment(base_link);

    const KDL::Vector pos_shoulder(0, 0, shoulder_height);
    const KDL::Rotation rot_shoulder(KDL::Rotation::Quaternion(0, 0, 0, 1));
    KDL::Segment shoulder_link("shoulder_link", KDL::Joint("linear_arm_actuator_joint", KDL::Joint::JointType::TransY), //KDL::Joint("shoulder_pan_joint", KDL::Joint::JointType::RotZ),
                               KDL::Frame(rot_shoulder, pos_shoulder));
    chain.addSegment(shoulder_link);

    const KDL::Vector pos_upper_arm(0, shoulder_offset, 0);
    const KDL::Rotation rot_upper_arm(KDL::Rotation::Quaternion(0, sqrt(2) / 2, 0, sqrt(2) / 2));
    KDL::Segment upper_arm_link("upper_arm_link", KDL::Joint("shoulder_pan_joint", KDL::Joint::JointType::RotZ), //KDL::Joint("shoulder_lift_joint", KDL::Joint::JointType::RotY),
                                KDL::Frame(rot_upper_arm, pos_upper_arm));
    chain.addSegment(upper_arm_link);

    const KDL::Vector pos_forearm(0, elbow_offset, upper_arm_length);
    const KDL::Rotation rot_forearm(KDL::Rotation::Quaternion(0, 0, 0, 1));
    KDL::Segment forearm_link("forearm_link", KDL::Joint("shoulder_lift_joint", KDL::Joint::JointType::RotY), //KDL::Joint("elbow_joint", KDL::Joint::JointType::RotY),
                              KDL::Frame(rot_forearm, pos_forearm));
    chain.addSegment(forearm_link);

    const KDL::Vector pos_wrist_1(0, 0, forearm_length);
    const KDL::Rotation rot_wrist_1(KDL::Rotation::Quaternion(0, sqrt(2) / 2, 0, sqrt(2) / 2));
    KDL::Segment wrist_1_link("wrist_1_link", KDL::Joint("elbow_joint", KDL::Joint::JointType::RotY), //KDL::Joint("wrist_1_joint", KDL::Joint::JointType::RotY),
                              KDL::Frame(rot_wrist_1, pos_wrist_1));
    chain.addSegment(wrist_1_link);

    const KDL::Vector pos_wrist_2(0, wrist_1_length, 0);
    const KDL::Rotation rot_wrist_2(KDL::Rotation::Quaternion(0, 0, 0, 1));
    KDL::Segment wrist_2_link("wrist_2_link", KDL::Joint("wrist_1_joint", KDL::Joint::JointType::RotY), //KDL::Joint("wrist_2_joint", KDL::Joint::JointType::RotZ),
                              KDL::Frame(rot_wrist_2, pos_wrist_2));
    chain.addSegment(wrist_2_link);

    const KDL::Vector pos_wrist_3(0, 0, wrist_2_length);
    const KDL::Rotation rot_wrist_3(KDL::Rotation::Quaternion(0, 0, 0, 1));
    KDL::Segment wrist_3_link("wrist_3_link", KDL::Joint("wrist_2_joint", KDL::Joint::JointType::RotZ), //KDL::Joint("wrist_3_joint", KDL::Joint::JointType::RotY),
                              KDL::Frame(rot_wrist_3, pos_wrist_3));
    chain.addSegment(wrist_3_link);

    const KDL::Vector pos_ee(0, wrist_3_length, 0.0);
    const KDL::Rotation rot_ee(KDL::Rotation::Quaternion(0, 0, sqrt(2) / 2, sqrt(2) / 2));
    KDL::Segment ee_link("ee_link", KDL::Joint("wrist_3_joint", KDL::Joint::JointType::RotY), //KDL::Joint(KDL::Joint::JointType::None),
                         KDL::Frame(rot_ee, pos_ee));
    chain.addSegment(ee_link);
}

// Use KDL to calculate position of end effector from joint states
int ArmRepresentation::calculateForwardKinematics(const KDL::JntArray &joint_positions, KDL::Frame &end_effector_pose, int joint_nbr)
{
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    int status = fk_solver.JntToCart(joint_positions, end_effector_pose, joint_nbr);

    return status;
}

// Use KDL to calculate joint states from position of end effector
int ArmRepresentation::calculateInverseKinematics(const vector<double> &cur_configuration,
                                                  const KDL::Frame &desired_end_effector_pose,
                                                  KDL::JntArray &final_joint_configuration)
{
    KDL::ChainIkSolverPos_LMA ik_solver(chain, 1e-3, 2000, 1e-8);
    
    // Convert cur_configuration vector to JntArray
    int num_joints = chain.getNrOfJoints();
    KDL::JntArray pos(num_joints);
    KDL::SetToZero(pos);

    for (int i = 0; i < num_joints; ++i) {
        pos(i) = cur_configuration[i];
    }

    int status = ik_solver.CartToJnt(pos, desired_end_effector_pose, final_joint_configuration);

    return status;
}

// Get vector of joint names
vector<string> ArmRepresentation::get_joint_names()
{
    vector<string> joint_names;

    for (auto it : chain.segments)
    {
        if (it.getJoint().getType() != KDL::Joint::JointType::None)
            joint_names.push_back(it.getJoint().getName());
    }

    return joint_names;
}

// Return reference to object
KDL::Chain *ArmRepresentation::getChain()
{
    return &chain;
}
