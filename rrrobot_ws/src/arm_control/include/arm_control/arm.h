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
#include <memory>

#include <string>

class Arm
{
public:
    /*
     * sdf_file: this is the sdf file that defines the model. This
     *      class will use that description of the robot to model 
     *      its dynamics.
     */
    Arm(const std::string &sdf_file);

    /*
     * Using the provided joint positions, this calculates the position of the end of the
     * arm. Returns whether it was successful
     */
    bool calculateForwardKinematics(KDL::JntArray joint_positions, KDL::Frame &end_effector_frame);

    bool calculateInverseKinematics(const KDL::JntArray &cur_configuration, const KDL::Frame &desired_end_effector_frame, KDL::JntArray &final_configuration);

    int calculateInverseDynamics(const KDL::JntArray &pos, const KDL::JntArray &vel, const KDL::JntArray &accel, const KDL::Wrenches &f_external, KDL::JntArray &required_torque);

    const KDL::Chain &getArm() const;

    void print() const;

private:
    KDL::Chain arm;

    // Create solver based on kinematic chain
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fksolver;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> iksolver;
    std::unique_ptr<KDL::ChainIdSolver_RNE> idsolver;
};