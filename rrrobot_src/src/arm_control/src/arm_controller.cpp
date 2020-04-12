#include <arm.h>
#include <iostream>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <vector>

#include "ros/ros.h"
#include <simulation_env/arm_command.h>
#include <simulation_env/arm_angles.h>
#include <simulation_env/arm_command.h>

using std::cout;
using std::endl;
using std::vector;
using namespace KDL;

Arm arm("/home/rrrobot/rrrobot_src/src/gazebo_models/fanuc_robotic_arm/model.sdf");
KDL::Chain chain = arm.getArm();
KDL::Chain simple_chain;
ros::Publisher publisher;

void enforce_torque_limits(simulation_env::arm_command &cmd, double limit = 2000.0)
{
    if (cmd.shoulder_pivot_force > limit)
        cmd.shoulder_pivot_force = limit;
    else if (cmd.shoulder_pivot_force < -limit)
        cmd.shoulder_pivot_force = -limit;

    if (cmd.shoulder_joint_force > limit)
        cmd.shoulder_joint_force = limit;
    else if (cmd.shoulder_joint_force < -limit)
        cmd.shoulder_joint_force = -limit;

    if (cmd.elbow_joint_force > limit)
        cmd.elbow_joint_force = limit;
    else if (cmd.elbow_joint_force < -limit)
        cmd.elbow_joint_force = -limit;

    if (cmd.wrist_pivot_force > limit)
        cmd.wrist_pivot_force = limit;
    else if (cmd.wrist_pivot_force < -limit)
        cmd.wrist_pivot_force = -limit;

    if (cmd.wrist_joint_force > limit)
        cmd.wrist_joint_force = limit;
    else if (cmd.wrist_joint_force < -limit)
        cmd.wrist_joint_force = -limit;

    if (cmd.end_effector_pivot_force > limit)
        cmd.end_effector_pivot_force = limit;
    else if (cmd.end_effector_pivot_force < -limit)
        cmd.end_effector_pivot_force = -limit;
}

void angle_callback(const simulation_env::arm_angles &msg)
{
    // Create solver based on kinematic chain
    static ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
    static KDL::JntArray prev_pos = KDL::JntArray(chain.getNrOfJoints());
    static KDL::JntArray prev_vel = KDL::JntArray(chain.getNrOfJoints());
    static double prev_time; // = ros::Time::now();
    static bool first_call = true;
    static simulation_env::arm_command last_command; // = {.shoulder_pivot_force = 0, .shoulder_joint_force = 0, .elbow_joint_force = 0, .wrist_pivot_force = 0, .wrist_joint_force = 0, .end_effector_pivot_force = 0};

    double cur_time = ros::Time::now().toSec();

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

    // // Calculate forward position kinematics
    // bool kinematics_status;
    // kinematics_status = fksolver.JntToCart(jointpositions, cartpos);
    // if (kinematics_status >= 0)
    // {
    //     // cout << nj << endl;
    //     // cout << "shoulder_pivot: " << jointpositions(0) << endl;
    //     // cout << "shoulder_joint: " << jointpositions(1) << endl;
    //     // cout << "elbow_joint: " << jointpositions(2) << endl;
    //     // cout << "wrist_pivot: " << jointpositions(3) << endl;
    //     // cout << "wrist_joint: " << jointpositions(4) << endl;
    //     // cout << "end_effector_pivot: " << jointpositions(5) << endl;

    //     std::cout << cartpos.p << std::endl;
    //     double roll, pitch, yaw;
    //     cartpos.M.GetRPY(roll, pitch, yaw);
    //     cout << "roll: " << roll << endl;
    //     cout << "pitch: " << pitch << endl;
    //     cout << "yaw: " << yaw << endl;
    //     cout << endl;

    //     // std::cout << cartpos << std::endl;
    //     //printf("%s \n", "Succes, thanks KDL!");
    // }
    // else
    // {
    //     //printf("%s \n", "Error: could not calculate forward kinematics :(");
    // }

    // try to move arm to (1, 1, 1)
    double vel_default = 0.0;
    KDL::JntArray vel(arm.getArm().getNrOfJoints());
    KDL::JntArray accel(arm.getArm().getNrOfJoints());

    // for (int joint = 0; joint < arm.getArm().getNrOfJoints(); ++joint)
    // {
    //     // vel(joint) = 0;
    //     accel(joint) = 0;
    // }

    vector<double> last_command_values;
    last_command_values.push_back(last_command.shoulder_pivot_force);
    last_command_values.push_back(last_command.shoulder_joint_force);
    last_command_values.push_back(last_command.elbow_joint_force);
    last_command_values.push_back(last_command.wrist_pivot_force);
    last_command_values.push_back(last_command.wrist_joint_force);
    last_command_values.push_back(last_command.end_effector_pivot_force);

    for (int joint = 0; joint < arm.getArm().getNrOfJoints(); ++joint)
    {
        KDL::Vector axis = arm.getArm().getSegment(joint).getJoint().JointAxis();
        double inertia = KDL::dot(arm.getArm().getSegment(joint).getInertia().getRotationalInertia() * axis, axis); /*axis of rotation */
        accel(joint) = (true || first_call || (inertia == 0)) ? 0.0 : last_command_values[joint] / inertia;
    }

    vel(0) = msg.shoulder_pivot_velocity;
    vel(1) = msg.shoulder_joint_velocity;
    vel(2) = msg.elbow_joint_velocity;
    vel(3) = 0; //msg.wrist_pivot_velocity;
    vel(4) = msg.wrist_joint_velocity;
    vel(5) = 0; //msg.end_effector_pivot_velocity;
    // vel(0) = 0; //(jointpositions(0) - prev_pos(0); //vel_default; //jointpositions(0) - msg.shoulder_pivot_angle;
    // vel(1) = 0; //vel_default;                     //jointpositions(1) - msg.shoulder_joint_angle;
    // vel(2) = 0; //vel_default;                     //jointpositions(2) - msg.elbow_joint_angle;
    // vel(3) = 0; //vel_default;                     //jointpositions(3) - msg.wrist_pivot_angle;
    // vel(4) = 0; //vel_default;                     //jointpositions(4) - msg.wrist_joint_angle;
    // vel(5) = 0; //vel_default;                     //jointpositions(5) - msg.end_effector_pivot_angle;
    // }
    // else
    // {
    //     for (int joint = 0; joint < arm.getArm().getNrOfJoints(); ++joint)
    //     {
    //         vel(joint) = (jointpositions(0) - prev_pos(0)) / (cur_time - prev_time);
    //         accel(joint) = (vel(joint) - prev_vel(joint)) / (cur_time - prev_time);
    //     }
    //     // vel(0) = (jointpositions(0) - prev_pos(0)) / (cur_time - prev_time); //vel_default; //jointpositions(0) - msg.shoulder_pivot_angle;
    //     // vel(1) = vel_default;                                                //jointpositions(1) - msg.shoulder_joint_angle;
    //     // vel(2) = vel_default;                                                //jointpositions(2) - msg.elbow_joint_angle;
    //     // vel(3) = vel_default;                                                //jointpositions(3) - msg.wrist_pivot_angle;
    //     // vel(4) = vel_default;                                                //jointpositions(4) - msg.wrist_joint_angle;
    //     // vel(5) = vel_default;                                                //jointpositions(5) - msg.end_effector_pivot_angle;
    // }
    KDL::Wrenches ext_force(arm.getArm().getNrOfSegments());
    KDL::JntArray required_force(arm.getArm().getNrOfJoints());
    KDL::Frame desired(KDL::Vector(1, 1, 1));
    KDL::JntArray required_positions(arm.getArm().getNrOfJoints());
    // bool failed = arm.calculateInverseKinematics(jointpositions, desired, required_positions);
    // int error_val = arm.calculateInverseDynamics(required_positions, vel, accel, ext_force, required_force);
    int error_val = arm.calculateInverseDynamics(jointpositions, vel, accel, ext_force, required_force);

    simulation_env::arm_command cmd;
    cmd.shoulder_pivot_force = 0; ///* -*/ required_force(0); // last added
    cmd.shoulder_joint_force = required_force(1);
    cmd.elbow_joint_force = /* -*/ required_force(2);
    cmd.wrist_pivot_force = 0; ///*-*/ -required_force(3);
    cmd.wrist_joint_force = /* -*/ -required_force(4);
    cmd.end_effector_pivot_force = 0; ///*-*/ required_force(5);

    enforce_torque_limits(cmd, 10000);

    // cout << "Command force:" << endl;
    // cout << "\tShoulder pivot: " << cmd.shoulder_pivot_force << endl;
    // cout << "\tShoulder joint: " << cmd.shoulder_joint_force << endl;
    // cout << "\tElbow joint: " << cmd.elbow_joint_force << endl;
    // cout << "\tWrist pivot: " << cmd.wrist_pivot_force << endl;
    // cout << "\tWrist joint: " << cmd.wrist_joint_force << endl;
    // cout << "\tEnd effector pivot: " << cmd.end_effector_pivot_force << endl;

    publisher.publish(cmd);
    ros::spinOnce();

    first_call = false;
    prev_pos = jointpositions;
    prev_vel = vel;
    prev_time = cur_time;
    last_command = cmd;
}

int main(int argc, char **argv)
{
    cout << "Starting arm controller..." << endl;
    ros::init(argc, argv, "arm_control_test");

    arm.print();

    // correct.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0.0))));
    // correct.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0, 0, 1))));
    // correct.addSegment(Segment(Joint(Joint::RotX), Frame(Vector(0, 0, 1))));

    ros::NodeHandle nh;
    publisher = nh.advertise<simulation_env::arm_command>("/arm_node/arm_commands", 1000);
    ros::Subscriber sub = nh.subscribe("/arm_node/arm_positions", 1000, angle_callback);

    cout << "0 positions for each segment" << endl;
    cout << "Base:" << endl;
    cout << chain.getSegment(0).pose(0.0) << endl;
    cout << "Link_1:" << endl;
    cout << chain.getSegment(0).pose(0.0) * chain.getSegment(1).pose(0.0) << endl;
    cout << "Link_2:" << endl;
    cout << chain.getSegment(0).pose(0) * chain.getSegment(1).pose(0.0) * chain.getSegment(2).pose(0.0) << endl;
    cout << "Link_3:" << endl;
    cout << chain.getSegment(0).pose(0) * chain.getSegment(1).pose(0.0) * chain.getSegment(2).pose(0.0) * chain.getSegment(3).pose(0.0) << endl;
    cout << "Link_4:" << endl;
    cout << chain.getSegment(0).pose(0) * chain.getSegment(1).pose(0.0) * chain.getSegment(2).pose(0.0) * chain.getSegment(3).pose(0.0) * chain.getSegment(4).pose(0.0) << endl;
    cout << "Link_5:" << endl;
    cout << chain.getSegment(0).pose(0) * chain.getSegment(1).pose(0.0) * chain.getSegment(2).pose(0.0) * chain.getSegment(3).pose(0.0) * chain.getSegment(4).pose(0.0) * chain.getSegment(5).pose(0.0) << endl;
    cout << "Link_6:" << endl;
    cout << chain.getSegment(0).pose(0) * chain.getSegment(1).pose(0.0) * chain.getSegment(2).pose(0.0) * chain.getSegment(3).pose(0.0) * chain.getSegment(4).pose(0.0) * chain.getSegment(5).pose(0.0) * chain.getSegment(6).pose(0.0) << endl;

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