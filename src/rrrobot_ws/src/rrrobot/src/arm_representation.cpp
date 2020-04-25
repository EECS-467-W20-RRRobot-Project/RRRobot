// arm_controller_node.cpp

#include "arm_representation.h"

// using namespace std;
using std::string;
using std::vector;

ArmRepresentation::ArmRepresentation(const KDL::Frame &base_pose)
{
    const double base_len = 0.2;
    const double shoulder_height = 0.1273;
    const double upper_arm_length = 0.612;
    const double forearm_length = 0.5723;
    const double shoulder_offset = 0.220941;
    const double elbow_offset = -0.1719;
    const double wrist_1_length = 0.163941 - elbow_offset - shoulder_offset;
    const double wrist_2_length = 0.1157;
    const double wrist_3_length = 0.0922;

    // KDL::Vector pos; //(0, 0, base_len/2);
    // KDL::Rotation rot;

    // The joints might be off by one
    KDL::Segment linear_arm_actuator("linear_arm_actuator_joint",
                                     KDL::Joint(KDL::Joint::JointType::None), base_pose);
    chain.addSegment(linear_arm_actuator);

    const KDL::Vector pos_base(0, 0, base_len / 2);
    const KDL::Rotation rot_base(KDL::Rotation::RPY(0, 0, 0));
    KDL::Segment base_link("base_link", KDL::Joint("linear_arm_actuator_joint", KDL::Joint::JointType::TransY),
                           KDL::Frame(rot_base, pos_base));
    chain.addSegment(base_link);

    const KDL::Vector pos_shoulder(0, 0, shoulder_height);
    const KDL::Rotation rot_shoulder(KDL::Rotation::RPY(0, 0, 0));
    KDL::Segment shoulder_link("shoulder_link", KDL::Joint("shoulder_pan_joint", KDL::Joint::JointType::RotZ),
                               KDL::Frame(rot_shoulder, pos_shoulder));
    chain.addSegment(shoulder_link);

    const KDL::Vector pos_upper_arm(0, shoulder_offset, 0);
    const KDL::Rotation rot_upper_arm(KDL::Rotation::RPY(0.0, M_PI / 2.0, 0.0));
    KDL::Segment upper_arm_link("upper_arm_link", KDL::Joint("shoulder_lift_joint", KDL::Joint::JointType::RotY),
                                KDL::Frame(rot_upper_arm, pos_upper_arm));
    chain.addSegment(upper_arm_link);

    const KDL::Vector pos_forearm(0, elbow_offset, upper_arm_length);
    const KDL::Rotation rot_forearm(KDL::Rotation::RPY(0.0, 0.0, 0.0));
    KDL::Segment forearm_link("forearm_link", KDL::Joint("elbow_joint", KDL::Joint::JointType::RotY),
                              KDL::Frame(rot_forearm, pos_forearm));
    chain.addSegment(forearm_link);

    const KDL::Vector pos_wrist_1(0, 0, forearm_length);
    const KDL::Rotation rot_wrist_1(KDL::Rotation::RPY(0.0, M_PI / 2.0, 0.0));
    KDL::Segment wrist_1_link("wrist_1_link", KDL::Joint("wrist_1_joint", KDL::Joint::JointType::RotY),
                              KDL::Frame(rot_wrist_1, pos_wrist_1));
    chain.addSegment(wrist_1_link);

    const KDL::Vector pos_wrist_2(0, wrist_1_length, 0);
    const KDL::Rotation rot_wrist_2(KDL::Rotation::RPY(0.0, 0.0, 0.0));
    KDL::Segment wrist_2_link("wrist_2_link", KDL::Joint("wrist_2_joint", KDL::Joint::JointType::RotZ),
                              KDL::Frame(rot_wrist_2, pos_wrist_2));
    chain.addSegment(wrist_2_link);

    const KDL::Vector pos_wrist_3(0, 0, wrist_2_length);
    const KDL::Rotation rot_wrist_3(KDL::Rotation::RPY(0.0, 0.0, 0.0));
    KDL::Segment wrist_3_link("wrist_3_link", KDL::Joint("wrist_3_joint", KDL::Joint::JointType::RotY),
                              KDL::Frame(rot_wrist_3, pos_wrist_3));
    chain.addSegment(wrist_3_link);

    const KDL::Vector pos_ee(0, wrist_3_length, 0.0);
    const KDL::Rotation rot_ee(KDL::Rotation::RPY(0.0, 0.0, M_PI / 2.0));
    KDL::Segment ee_link("ee_link", KDL::Joint(KDL::Joint::JointType::None),
                         KDL::Frame(rot_ee, pos_ee));
    chain.addSegment(ee_link);

    // arm.addSegment(base_link);
    // arm.addSegment(shoulder_link);
    // arm.addSegment(upper_arm_link);
    // arm.addSegment(forearm_link);
    // arm.addSegment(wrist_1_link);
    // arm.addSegment(wrist_2_link);
    // arm.addSegment(wrist_3_link);
    // arm.addSegment(ee_link);

    // fk_solver = KDL::ChainFkSolverPos_recursive(chain);
    // ik_solver = KDL::ChainIkSolverPos_LMA(chain);

    // fk_solver.reset(new KDL::ChainFkSolverPos_recursive(arm));
    // ik_solver.reset(new KDL::ChainIkSolverPos_LMA(arm));
}

int ArmRepresentation::calculateForwardKinematics(const KDL::JntArray &joint_positions, KDL::Frame &end_effector_pose)
{
    KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(chain);
    int status = fk_solver.JntToCart(joint_positions, end_effector_pose);

    return status;
}

int ArmRepresentation::calculateInverseKinematics(const KDL::JntArray &cur_configuration,
                                                  const KDL::Frame &desired_end_effector_pose,
                                                  KDL::JntArray &final_joint_configuration)
{
    KDL::ChainIkSolverPos_LMA ik_solver = KDL::ChainIkSolverPos_LMA(chain);
    int status = ik_solver.CartToJnt(cur_configuration, desired_end_effector_pose, final_joint_configuration);

    return status;
}

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

KDL::Chain *ArmRepresentation::getChain()
{
    return &chain;
}