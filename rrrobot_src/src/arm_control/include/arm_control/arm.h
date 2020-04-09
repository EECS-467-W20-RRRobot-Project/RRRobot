#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/rotationalinertia.hpp>
#include <kdl/joint.hpp>
#include <kdl/frames.hpp>
#include <LinearMath/btTransform.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
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
     * Gets the center of mass of the remaining arm, starting from the
     * link in index (ie the center of mass of links l[i]-l[n])
     */
    KDL::Vector getCOM(size_t index);
    /*
     * Gets the center of mass of the remaining arm, starting from the
     * link named 'link_name'
     */
    KDL::Vector getCOM(const std::string &link_name);

    /*
     * Gets the inertia of the remaining arm about the joint in index. This
     * treats the preceding segments as if they are stationary and calculates
     * the inertia of the remaining arm.
     */
    float getInertia(size_t index);
    /*
     * Gets the inertia of the remaining arm about the joint named 'joint_name'. This
     * treats the preceding segments as if they are stationary and calculates
     * the inertia of the remaining arm.
     */
    float getInertia(const std::string &joint_name);

    /*
     * This returns the mass of the remaining links, starting from (and including)
     * the link at index
     */
    float getSupportedMass(size_t index);
    /*
     * This returns the mass of the remaining links, starting from (and including)
     * the link named 'link_name'
     */
    float getSupportedMass(const std::string &link_name);

    /*
     * This returns the torques required to get the arm moving with the desired 
     * angular acceleration.
     */
    std::vector<float> getRequiredTorques(/*std::vector<float> theta_des, std::vector<float> theta_dot_des, */ std::vector<float> theta_double_dot_des);

    /*
     * Using the provided joint positions, this calculates the position of the end of the
     * arm. Returns whether it was successful
     */
    bool calculateForwardKinematics(KDL::JntArray joint_positions, KDL::Frame &end_effector_frame);

    const KDL::Chain &getArm() const;

    void print() const;

private:
    KDL::Chain arm;

    // Create solver based on kinematic chain
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fksolver;

    /*
     * i_com: the inertia about the center of mass of this link
     * mass: the mass of this link
     * distance: the distance from the center of mass to the new axis
     */
    float parallelAxisTheorem(float i_com, float mass, float distance);
};