#include <arm.h>
#include <iostream>

#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>

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
KDL::JntArray pos(arm.getArm().getNrOfJoints());

ros::Publisher shoulder_pivot_publisher;
ros::Publisher shoulder_joint_publisher;
ros::Publisher elbow_joint_publisher;
ros::Publisher wrist_pivot_publisher;
ros::Publisher wrist_joint_publisher;
ros::Publisher end_effector_pivot_publisher;

void shoulder_pivot_callback(const std_msgs::Float64 &cmd)
{
    pos(0) = cmd.data;
}

void shoulder_joint_callback(const std_msgs::Float64 &cmd)
{
    pos(1) = cmd.data;
}

void elbow_joint_callback(const std_msgs::Float64 &cmd)
{
    pos(2) = cmd.data;
}

void wrist_pivot_callback(const std_msgs::Float64 &cmd)
{
    pos(3) = cmd.data;
}

void wrist_joint_callback(const std_msgs::Float64 &cmd)
{
    pos(4) = cmd.data;
}

void end_effector_pivot_callback(const std_msgs::Float64 &cmd)
{
    pos(5) = cmd.data;
}

void fixAngles(const KDL::JntArray &current_angles, KDL::JntArray &desired_angles)
{
    // TODO: go clockwise or counter-clockwise depending on which is shorter
}

void endEffectorPoseTargetCallback(const geometry_msgs::Pose &msg)
{
    KDL::JntArray required_angles(arm.getArm().getNrOfJoints());
    KDL::SetToZero(required_angles);

    KDL::Frame desired(KDL::Rotation::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                       KDL::Vector(msg.position.x, msg.position.y, msg.position.z));

    arm.calculateInverseKinematics(pos, desired, required_angles);

    fixAngles(pos, required_angles);

    std_msgs::Float64 shoulder_pivot_setpoint;
    shoulder_pivot_setpoint.data = required_angles(0);
    std_msgs::Float64 shoulder_joint_setpoint;
    shoulder_joint_setpoint.data = required_angles(1);
    std_msgs::Float64 elbow_joint_setpoint;
    elbow_joint_setpoint.data = required_angles(2);
    std_msgs::Float64 wrist_pivot_setpoint;
    wrist_pivot_setpoint.data = required_angles(3);
    std_msgs::Float64 wrist_joint_setpoint;
    wrist_joint_setpoint.data = required_angles(4);
    std_msgs::Float64 end_effector_setpoint;
    end_effector_setpoint.data = required_angles(5);

    shoulder_pivot_publisher.publish(shoulder_pivot_setpoint);
    shoulder_joint_publisher.publish(shoulder_joint_setpoint);
    elbow_joint_publisher.publish(elbow_joint_setpoint);
    wrist_pivot_publisher.publish(wrist_pivot_setpoint);
    wrist_joint_publisher.publish(wrist_joint_setpoint);
    end_effector_pivot_publisher.publish(end_effector_setpoint);
}

int main(int argc, char **argv)
{
    cout << "Starting arm controller..." << endl;
    ros::init(argc, argv, "arm_control_node");

    ros::NodeHandle nh;
    ros::Subscriber shoulder_pivot_sub = nh.subscribe("/arm_node/arm_positions/shoulder_pivot_angle", 1000, &shoulder_pivot_callback);
    ros::Subscriber shoulder_joint_sub = nh.subscribe("/arm_node/arm_positions/shoulder_joint_angle", 1000, &shoulder_joint_callback);
    ros::Subscriber elbow_joint_sub = nh.subscribe("/arm_node/arm_positions/elbow_joint_angle", 1000, &elbow_joint_callback);
    ros::Subscriber wrist_pivot_sub = nh.subscribe("/arm_node/arm_positions/wrist_pivot_angle", 1000, &wrist_pivot_callback);
    ros::Subscriber wrist_joint_sub = nh.subscribe("/arm_node/arm_positions/wrist_joint_angle", 1000, &wrist_joint_callback);
    ros::Subscriber end_effector_sub = nh.subscribe("/arm_node/arm_positions/end_effector_pivot_angle", 1000, &end_effector_pivot_callback);

    ros::Subscriber end_effector_pose_setpoint_sub = nh.subscribe("/arm_node/arm_pose_setpoint", 1000, &endEffectorPoseTargetCallback);

    KDL::SetToZero(pos);

    shoulder_pivot_publisher = nh.advertise<std_msgs::Float64>("/arm_node/shoulder_pivot_setpoint", 1000);
    shoulder_joint_publisher = nh.advertise<std_msgs::Float64>("/arm_node/shoulder_joint_setpoint", 1000);
    elbow_joint_publisher = nh.advertise<std_msgs::Float64>("/arm_node/elbow_joint_setpoint", 1000);
    wrist_pivot_publisher = nh.advertise<std_msgs::Float64>("/arm_node/wrist_pivot_setpoint", 1000);
    wrist_joint_publisher = nh.advertise<std_msgs::Float64>("/arm_node/wrist_joint_setpoint", 1000);
    end_effector_pivot_publisher = nh.advertise<std_msgs::Float64>("/arm_node/end_effector_pivot_setpoint", 1000);

    ros::spin();
}