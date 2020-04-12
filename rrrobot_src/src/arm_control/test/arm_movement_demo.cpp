#include <std_msgs/Float64.h>
#include "ros/ros.h"
#include <ros/rate.h>

#include <arm.h>

#include <iostream>
#include <vector>
#include <cmath>

using std::cout;
using std::endl;
using std::vector;

std_msgs::Float64 shoulder_pivot_angle;
std_msgs::Float64 shoulder_joint_angle;
std_msgs::Float64 elbow_joint_angle;
std_msgs::Float64 wrist_pivot_angle;
std_msgs::Float64 wrist_joint_angle;
std_msgs::Float64 end_effector_pivot_angle;
Arm arm("/home/rrrobot/rrrobot_src/src/gazebo_models/fanuc_robotic_arm/model.sdf");
KDL::JntArray pos(arm.getArm().getNrOfJoints());
char received_angles = 0;

void shoulder_pivot_callback(const std_msgs::Float64 &cmd)
{
    // update the torque applied to each joint when a message is received
    shoulder_pivot_angle = cmd;
    pos(0) = cmd.data;
    // cout << "shoulder pivot: " << shoulder_pivot_angle.data << endl;
    received_angles |= 0x01;
}

void shoulder_joint_callback(const std_msgs::Float64 &cmd)
{
    // update the torque applied to each joint when a message is received
    shoulder_joint_angle = cmd;
    pos(1) = cmd.data;
    // cout << "shoulder joint: " << shoulder_joint_angle.data << endl;
    received_angles |= 0x02;
}

void elbow_joint_callback(const std_msgs::Float64 &cmd)
{
    // update the torque applied to each joint when a message is received
    elbow_joint_angle = cmd;
    pos(2) = cmd.data;
    // cout << "elbow joint: " << elbow_joint_angle.data << endl;
    received_angles |= 0x04;
}

void wrist_pivot_callback(const std_msgs::Float64 &cmd)
{
    // update the torque applied to each joint when a message is received
    wrist_pivot_angle = cmd;
    pos(3) = cmd.data;
    // cout << "wrist pivot: " << wrist_pivot_angle.data << endl;
    received_angles |= 0x08;
}

void wrist_joint_callback(const std_msgs::Float64 &cmd)
{
    // update the torque applied to each joint when a message is received
    wrist_joint_angle = cmd;
    pos(4) = cmd.data;
    // cout << "wrist joint: " << wrist_joint_angle.data << endl;
    received_angles |= 0x10;
}

void end_effector_pivot_callback(const std_msgs::Float64 &cmd)
{
    // update the torque applied to each joint when a message is received
    end_effector_pivot_angle = cmd;
    pos(5) = cmd.data;
    // cout << "end effector pivot: " << end_effector_pivot_angle.data << endl;
    received_angles |= 0x20;
}

void fix_angle(double cur_pos, double &desired)
{
    while (desired > M_PI)
    {
        desired -= 2 * M_PI;
    }
    while (desired < -M_PI)
    {
        desired += 2 * M_PI;
    }

    double cur_pos_truncated = cur_pos; // % (M_PI);
    while (cur_pos_truncated > M_PI)
    {
        cur_pos_truncated -= 2 * M_PI;
    }
    while (cur_pos_truncated < -M_PI)
    {
        cur_pos_truncated += 2 * M_PI;
    }
    // say desired = 1.5708
    // 4 cases:
    //  1. cur_pos = 0.2 --> set target as cur_pos + 1.3708
    //  2. cur_pos = -0.2 --> set target as cur_pos + 1.7708
    //  3. cur_pos = -2.5 --> set target as cur_pos + 2.21239
    //  4. cur_pos = 2.5 --> set target as cur_pos + 0.9292

    if ((cur_pos_truncated - desired) < M_PI && (cur_pos_truncated - desired) > 0)
    {
        desired = cur_pos - (cur_pos_truncated - desired);
    }
    else if ((desired - cur_pos_truncated) < M_PI && (desired - cur_pos_truncated) > 0)
    {
        desired = cur_pos + (desired - cur_pos_truncated);
    }
    else if ((cur_pos_truncated - desired) > -M_PI && (cur_pos_truncated - desired) < 0)
    {
        desired = cur_pos - (cur_pos_truncated - desired);
    }
    else if ((desired - cur_pos_truncated) > -M_PI && (desired - cur_pos_truncated) < 0)
    {
        desired = cur_pos + (desired - cur_pos_truncated);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_control_demo");

    KDL::SetToZero(pos);

    ros::Publisher shoulder_pivot_publisher;
    ros::Publisher shoulder_joint_publisher;
    ros::Publisher elbow_joint_publisher;
    ros::Publisher wrist_pivot_publisher;
    ros::Publisher wrist_joint_publisher;
    ros::Publisher end_effector_pivot_publisher;

    ros::Subscriber shoulder_pivot_sub;
    ros::Subscriber shoulder_joint_sub;
    ros::Subscriber elbow_joint_sub;
    ros::Subscriber wrist_pivot_sub;
    ros::Subscriber wrist_joint_sub;
    ros::Subscriber end_effector_sub;

    std_msgs::Float64 shoulder_pivot_target;
    shoulder_pivot_target.data = 0;
    std_msgs::Float64 shoulder_joint_target;
    shoulder_joint_target.data = 0;
    std_msgs::Float64 elbow_joint_target;
    elbow_joint_target.data = 0;
    std_msgs::Float64 wrist_pivot_target;
    wrist_pivot_target.data = 0;
    std_msgs::Float64 wrist_joint_target;
    wrist_joint_target.data = 0;
    std_msgs::Float64 end_effector_pivot_target;
    end_effector_pivot_target.data = 0;

    ros::NodeHandle nh;
    ros::Rate pub_rate(0.1); // publish every 10 seconds
    ros::Rate quick(1);

    shoulder_pivot_publisher = nh.advertise<std_msgs::Float64>("/arm_node/shoulder_pivot_setpoint", 1000);
    shoulder_joint_publisher = nh.advertise<std_msgs::Float64>("/arm_node/shoulder_joint_setpoint", 1000);
    elbow_joint_publisher = nh.advertise<std_msgs::Float64>("/arm_node/elbow_joint_setpoint", 1000);
    wrist_pivot_publisher = nh.advertise<std_msgs::Float64>("/arm_node/wrist_pivot_setpoint", 1000);
    wrist_joint_publisher = nh.advertise<std_msgs::Float64>("/arm_node/wrist_joint_setpoint", 1000);
    end_effector_pivot_publisher = nh.advertise<std_msgs::Float64>("/arm_node/end_effector_pivot_setpoint", 1000);

    shoulder_pivot_sub = nh.subscribe("/arm_node/arm_positions/shoulder_pivot_angle", 1000, &shoulder_pivot_callback);
    shoulder_joint_sub = nh.subscribe("/arm_node/arm_positions/shoulder_joint_angle", 1000, &shoulder_joint_callback);
    elbow_joint_sub = nh.subscribe("/arm_node/arm_positions/elbow_joint_angle", 1000, &elbow_joint_callback);
    wrist_pivot_sub = nh.subscribe("/arm_node/arm_positions/wrist_pivot_angle", 1000, &wrist_pivot_callback);
    wrist_joint_sub = nh.subscribe("/arm_node/arm_positions/wrist_joint_angle", 1000, &wrist_joint_callback);
    end_effector_sub = nh.subscribe("/arm_node/arm_positions/end_effector_pivot_angle", 1000, &end_effector_pivot_callback);

    int state = 0;
    KDL::JntArray required_angles(arm.getArm().getNrOfJoints());
    KDL::SetToZero(required_angles);

    vector<KDL::Frame> desired_positions;
    // desired_positions.push_back(KDL::Frame(KDL::Vector(1, -0.5, 1)));
    // desired_positions.push_back(KDL::Frame(KDL::Vector(1, -0.4, 1)));
    // desired_positions.push_back(KDL::Frame(KDL::Vector(1, -0.3, 1)));
    // desired_positions.push_back(KDL::Frame(KDL::Vector(1, -0.2, 1)));
    // desired_positions.push_back(KDL::Frame(KDL::Vector(1, -0.1, 1)));
    // desired_positions.push_back(KDL::Frame(KDL::Vector(1, 0.0, 1)));
    // desired_positions.push_back(KDL::Frame(KDL::Vector(1, 0.1, 1)));
    // desired_positions.push_back(KDL::Frame(KDL::Vector(1, 0.2, 1)));
    // desired_positions.push_back(KDL::Frame(KDL::Vector(1, 0.3, 1)));
    // desired_positions.push_back(KDL::Frame(KDL::Vector(1, 0.4, 1)));
    // desired_positions.push_back(KDL::Frame(KDL::Vector(1, 0.5, 1)));
    desired_positions.push_back(KDL::Frame(KDL::Vector(1, 1, 1)));
    desired_positions.push_back(KDL::Frame(KDL::Vector(1, -1, 1)));
    desired_positions.push_back(KDL::Frame(KDL::Vector(1, -1, 2)));
    desired_positions.push_back(KDL::Frame(KDL::Vector(1, 1, 3)));

    while (ros::ok())
    {
        // ros::spinOnce();
        // ros::spinOnce();
        // quick.sleep();

        // pos(0) = shoulder_pivot_angle.data;
        // pos(1) = shoulder_joint_angle.data;
        // pos(2) = elbow_joint_angle.data;
        // pos(3) = wrist_pivot_angle.data;
        // pos(4) = wrist_joint_angle.data;
        // pos(5) = end_effector_pivot_angle.data;

        // if (state) //fabs(shoulder_joint_target.data) < 0.001)
        // {
        arm.calculateInverseKinematics(pos, desired_positions[state] /*KDL::Frame(KDL::Vector(1, 0, 0))*/, required_angles);
        //shoulder_joint_target.data = 0.7;
        // }
        // else
        // {
        //     arm.calculateInverseKinematics(pos, KDL::Frame(KDL::Vector(-1, -1, 1)), required_angles);
        //     //shoulder_joint_target.data = 0.0;
        // }

        cout << desired_positions[state].p.x() << " " << desired_positions[state].p.y() << " " << desired_positions[state].p.z() << endl;
        cout << "\t" << required_angles(0) << "\t" << required_angles(1) << "\t" << required_angles(2) << "\t" << required_angles(3) << "\t" << required_angles(4) << "\t" << required_angles(5) << endl;
        cout << endl;

        for (int idx = 0; idx < arm.getArm().getNrOfJoints(); ++idx)
        {
            // cout << "Cur: " << pos(idx) << "\tRequired: " << required_angles(idx) << endl;
            fix_angle(pos(idx), required_angles(idx));
            // cout << "\tPost fix: Required: " << required_angles(idx) << endl;
        }

        shoulder_pivot_target.data = required_angles(0);
        shoulder_joint_target.data = required_angles(1);
        elbow_joint_target.data = required_angles(2);
        wrist_pivot_target.data = required_angles(3);
        wrist_joint_target.data = required_angles(4);
        end_effector_pivot_target.data = required_angles(5);

        // cout << "Received angles: " << (int)received_angles << endl;
        // if (received_angles == 0b00111111)
        // {
        shoulder_pivot_publisher.publish(shoulder_pivot_target);
        shoulder_joint_publisher.publish(shoulder_joint_target);
        elbow_joint_publisher.publish(elbow_joint_target);
        wrist_pivot_publisher.publish(wrist_pivot_target);
        wrist_joint_publisher.publish(wrist_joint_target);
        end_effector_pivot_publisher.publish(end_effector_pivot_target);

        state = (state + 1) % desired_positions.size();
        // }
        // else
        // {
        //     ros::spinOnce();
        // }

        pub_rate.sleep();

        ros::spinOnce();
    }
}