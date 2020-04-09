#include <arm.h>

#include "gtest/gtest.h"
#include <kdl/frames_io.hpp>
#include <kdl/chain.hpp>
#include <cmath>

const double ALLOWED_ERROR = 0.05;

double getDistance(const KDL::Vector &a, const KDL::Vector &b)
{
    return sqrt(pow(a.x() - b.x(), 2) + pow(a.y() - b.y(), 2) + pow(a.z() - b.z(), 2));
}

TEST(ArmRepresentationTests, shoulder_pivot)
{
    Arm arm("/home/rrrobot/rrrobot_src/src/gazebo_models/fanuc_robotic_arm/model.sdf");

    // Create joint array
    unsigned int nj = arm.getArm().getNrOfJoints();
    KDL::JntArray jointpositions = KDL::JntArray(nj);
    for (int pos = 0; pos < nj; ++pos)
    {
        jointpositions(pos) = 0;
    }

    // Create the frame that will contain the results
    KDL::Frame cartpos;
    KDL::Vector correct(0.000767, -1.87014, 2.0658);

    jointpositions(0) = 0.0;
    arm.calculateForwardKinematics(jointpositions, cartpos);

    EXPECT_NEAR(getDistance(cartpos.p, correct), 0.0, ALLOWED_ERROR);

    jointpositions(0) = 2 * M_PI;
    arm.calculateForwardKinematics(jointpositions, cartpos);
    EXPECT_NEAR(getDistance(cartpos.p, correct), 0.0, ALLOWED_ERROR);

    jointpositions(0) = M_PI;
    arm.calculateForwardKinematics(jointpositions, cartpos);
    correct = KDL::Vector(0.000767, 1.87014, 2.0658);
    EXPECT_NEAR(getDistance(cartpos.p, correct), 0.0, ALLOWED_ERROR);
}

TEST(ArmRepresentationTests, elbow_pivot)
{
    Arm arm("/home/rrrobot/rrrobot_src/src/gazebo_models/fanuc_robotic_arm/model.sdf");

    // Create joint array
    unsigned int nj = arm.getArm().getNrOfJoints();
    KDL::JntArray jointpositions = KDL::JntArray(nj);
    for (int pos = 0; pos < nj; ++pos)
    {
        jointpositions(pos) = 0;
    }

    // Create the frame that will contain the results
    KDL::Frame cartpos;
    KDL::Vector correct(0.000767, -1.87014, 2.0658);

    for (double pos = 0.0; pos <= M_2_PI; pos += 0.1)
    {
        jointpositions(3) = pos;
        arm.calculateForwardKinematics(jointpositions, cartpos);
        EXPECT_NEAR(getDistance(cartpos.p, correct), 0.0, ALLOWED_ERROR);
    }
}

TEST(ArmRepresentationTests, end_effector_pivot)
{
    Arm arm("/home/rrrobot/rrrobot_src/src/gazebo_models/fanuc_robotic_arm/model.sdf");

    // Create joint array
    unsigned int nj = arm.getArm().getNrOfJoints();
    KDL::JntArray jointpositions = KDL::JntArray(nj);
    for (int pos = 0; pos < nj; ++pos)
    {
        jointpositions(pos) = 0;
    }

    // Create the frame that will contain the results
    KDL::Frame cartpos;
    KDL::Vector correct(0.000767, -1.87014, 2.0658);

    for (double pos = 0.0; pos <= M_2_PI; pos += 0.1)
    {
        jointpositions(5) = pos;
        arm.calculateForwardKinematics(jointpositions, cartpos);
        EXPECT_NEAR(getDistance(cartpos.p, correct), 0.0, ALLOWED_ERROR);
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}