#include <geometry_msgs/Pose.h>
#include "ros/ros.h"
#include <ros/rate.h>

#include <arm.h>

#include <iostream>
#include <vector>
#include <cmath>

using std::cout;
using std::endl;
using std::vector;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_control_demo");

    ros::NodeHandle nh;
    ros::Rate pub_rate(0.1); // publish every 10 seconds
    ros::Publisher end_effector_setpoint_publisher = nh.advertise<geometry_msgs::Pose>("/arm_node/arm_pose_setpoint", 1000);

    int state = 0;

    vector<KDL::Frame> desired_positions;
    desired_positions.push_back(KDL::Frame(KDL::Rotation::RPY(-2.35619, 0.7853, 0.0), KDL::Vector(1, 1, 1)));
    desired_positions.push_back(KDL::Frame(KDL::Vector(1, -1, 1)));
    desired_positions.push_back(KDL::Frame(KDL::Vector(1, -1, 2)));
    desired_positions.push_back(KDL::Frame(KDL::Vector(1, 1, 3)));

    while (ros::ok())
    {
        const KDL::Frame &cur_setpoint = desired_positions[state];
        geometry_msgs::Pose msg;
        msg.position.x = cur_setpoint.p.x();
        msg.position.y = cur_setpoint.p.y();
        msg.position.z = cur_setpoint.p.z();
        cur_setpoint.M.GetQuaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);

        end_effector_setpoint_publisher.publish(msg);
        ros::spinOnce();

        state = (state + 1) % desired_positions.size();

        pub_rate.sleep();
    }
}