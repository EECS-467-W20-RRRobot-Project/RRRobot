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

#include <fstream>
#include <iostream>
#include <string>

// using namespace std;
using std::string;

class ArmRepresentation
{
public:
    ArmRepresentation(const KDL::Frame &base_pose = KDL::Frame(KDL::Rotation::Quaternion(0, 0, 0, 1), KDL::Vector(0.3, 0.92, 0.9))); //KDL::Frame(KDL::Vector(0.3, 0.92, 1)));

    int calculateForwardKinematics(const KDL::JntArray &joint_positions, KDL::Frame &end_effector_pose);

    int calculateInverseKinematics(const KDL::JntArray &cur_configuration,
                                   const KDL::Frame &desired_end_effector_pose,
                                   KDL::JntArray &final_joint_configuration);

    std::vector<string> get_joint_names();

    KDL::Chain *getChain();

private:
    KDL::Chain chain;

    // KDL::ChainFkSolverPos_recursive fk_solver;
    // KDL::ChainIkSolverPos_LMA ik_solver;
};