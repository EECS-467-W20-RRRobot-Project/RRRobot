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
#include <sensor_msgs/JointState.h>
#include <osrf_gear/VacuumGripperState.h>

#include "topic_names.h"
#include "rrrobot/arm_command.h"
#include "arm_representation.h"

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

#include <ros/console.h>

#include <cstdlib>

using namespace std;


class ArmController
{
public:
    ArmController(ros::NodeHandle &node, ArmRepresentation *arm_) : gripper_enabled_(false), item_attached_(false), arm(arm_)
    {
        arm_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(ARM_COMMAND_CHANNEL, 10);

        gripper_ = node.serviceClient<osrf_gear::VacuumGripperControl>(GRIPPER_CONTROL_CHANNEL);

        // arm = std::move(arm_);
        // ROS_INFO("Arm size (in constructor): %d", arm->getChain()->getNrOfJoints());
        arm_current_joint_states_ = KDL::JntArray(arm->getChain()->getNrOfJoints());
        KDL::SetToZero(arm_current_joint_states_);
    }

    void joint_state_callback(const sensor_msgs::JointState &joint_state_msg)
    {
        // ROS_INFO("Received joint state");

        // Convert std_msgs::double[] to KDL::JntArray
        int nbr_joints = arm->getChain()->getNrOfJoints();
        vector<string> msg_joint_names = joint_state_msg.name;
        vector<string> cur_joint_names = arm->get_joint_names();
        vector<double> position = joint_state_msg.position;

        // Print joint name vectors for debugging
        // ROS_INFO_STREAM("msg_joint_names: {");
        // for (int i = 0; i < nbr_joints; ++i)
        // {
        //     ROS_INFO_STREAM(msg_joint_names[i] << ", ");
        // }
        // ROS_INFO_STREAM("}\ncur_joint_names: {");
        // for (int j = 0; j < cur_joint_names.size(); ++j)
        // {
        //     ROS_INFO_STREAM(cur_joint_names[j] << ", ");
        // }
        // ROS_INFO_STREAM("}");

        for (int i = 0; i < nbr_joints; ++i)
        {
            arm_current_joint_states_(i) = 0.0;
        }

        for (size_t i = 0; i < position.size(); ++i)
        {
            for (size_t j = 0; j < cur_joint_names.size(); ++j)
            {
                if (msg_joint_names[i].compare(cur_joint_names[j]) == 0)
                {
                    arm_current_joint_states_(j) = position[i];
                }
            }
        }
    }

    void gripper_state_callback(const osrf_gear::VacuumGripperState::ConstPtr &gripper_state_msg)
    {
        gripper_enabled_ = gripper_state_msg->enabled;
        item_attached_ = gripper_state_msg->attached;
    }

    void arm_destination_callback(const rrrobot::arm_command &target_pose)
    {
        ROS_INFO("Received target pose");

        int nbr_joints = arm->getChain()->getNrOfJoints();
        vector<double> positions;
        double time_from_start = 5; // Seconds
        ros::Rate timeout(1.0/time_from_start);//

        // Move arm to pickup item from conveyor belt
        positions = calc_joint_positions(target_pose.grab_location);
        send_joint_trajectory(positions, time_from_start);

        // Wait until target position is reached
        while (!have_reached_target(frame_to_pose(calc_end_effector_pose()), target_pose.grab_location))
        {
            // TODO: Do something if not able to reach target
            time_from_start += 5;
            send_joint_trajectory(positions, time_from_start);
            timeout.sleep();

            // update sleep time_from_start
            timeout = ros::Rate(1.0/time_from_start);
            continue;
        }

        // Turn on suction
        while (!gripper_enabled_)
        {
            gripper_control(true);
        }

        // Wait until object is attached
        while (!item_attached_)
        {
            // TODO: Do something if item is not able to attach
            continue;
        }

        // Move item to desired position
        positions = calc_joint_positions(target_pose.drop_location);
        send_joint_trajectory(positions, time_from_start);

        // Wait until target position is reached
        while (!have_reached_target(frame_to_pose(calc_end_effector_pose()), target_pose.drop_location))
        {
            // TODO: Do something if not able to reach target
            continue;
        }

        // Turn off suction
        while (gripper_enabled_)
        {
            gripper_control(false);
        }

        // Wait until object is detached
        while (item_attached_)
        {
            // TODO: Do something if item doesn't detach
            continue;
        }
    }

private:
    bool gripper_enabled_;
    bool item_attached_;
    ros::Publisher arm_joint_trajectory_publisher_;
    ros::ServiceClient gripper_;
    KDL::JntArray arm_current_joint_states_;
    ArmRepresentation *arm;

    vector<double> calc_joint_positions(const geometry_msgs::Pose &pose)
    {
        // KDL::JntArray cur_configuration(arm->getChain()->getNrOfJoints()); // = arm_current_joint_states_;
        // KDL::SetToZero(cur_configuration);
        KDL::Frame desired_end_effector_pose = pose_to_frame(pose);
        KDL::JntArray final_joint_configuration(arm->getChain()->getNrOfJoints());

        // ROS_INFO("cur_configuration (%i x %i)", cur_configuration.rows(), cur_configuration.columns());

        int error_code = arm->calculateInverseKinematics(arm_current_joint_states_, desired_end_effector_pose, final_joint_configuration);

        // Check status of IK and print error message if there is a failure
        if (error_code != 0)
        {
            ROS_ERROR("Inverse Kinematics Failure: %i", error_code);
        }

        // Convert data attribute (Eigen::VectorXd) of KDL::JntArray to double[] via data() function
        int nbr_joints = arm->getChain()->getNrOfJoints();
        Eigen::VectorXd mat = final_joint_configuration.data;
        // cout << mat << endl;
        vector<double> positions;
        for (size_t idx = 0; idx < arm->getChain()->getNrOfJoints(); ++idx)
        {
            positions.push_back(mat[idx]);
        }

        return positions;
    }

    KDL::Frame calc_end_effector_pose()
    {
        KDL::Frame end_effector_pose;
        // KDL::JntArray joint_positions = arm_current_joint_states_;
        KDL::JntArray joint_positions(arm->getChain()->getNrOfJoints());
        KDL::SetToZero(joint_positions);

        // ROS_INFO("joint_positions (%i x %i)", joint_positions.rows(), joint_positions.columns());
        // ROS_INFO("arm chain # of joints: %d", arm->getChain()->getNrOfJoints());
        int error_code = arm->calculateForwardKinematics(joint_positions, end_effector_pose);

        // Check status of FK and do something if there is a failure
        if (error_code != 0)
        {
            ROS_ERROR("Forward Kinematics Failure: %i", error_code);
        }

        return end_effector_pose;
    }

    KDL::Frame pose_to_frame(const geometry_msgs::Pose &pose)
    {
        double p_x = pose.position.x;
        double p_y = pose.position.y;
        double p_z = pose.position.z;

        double q_x = pose.orientation.x;
        double q_y = pose.orientation.y;
        double q_z = pose.orientation.z;
        double q_w = pose.orientation.w;

        KDL::Rotation rot(KDL::Rotation::Quaternion(q_x, q_y, q_z, q_w));
        KDL::Vector pos(p_x, p_y, p_z);
        return KDL::Frame(rot, pos);
    }

    geometry_msgs::Pose frame_to_pose(const KDL::Frame &frame)
    {
        double p_x = frame.p.x();
        double p_y = frame.p.y();
        double p_z = frame.p.z();

        double q_x, q_y, q_z, q_w;
        frame.M.GetQuaternion(q_x, q_y, q_z, q_w);

        geometry_msgs::Pose pose;

        pose.position.x = p_x;
        pose.position.y = p_y;
        pose.position.z = p_z;

        pose.orientation.x = q_x;
        pose.orientation.y = q_y;
        pose.orientation.z = q_z;
        pose.orientation.w = q_w;

        return pose;
    }

    void send_joint_trajectory(const vector<double> &positions, double time_from_start)
    {
        // Declare JointTrajectory message
        trajectory_msgs::JointTrajectory msg;

        // Fill the names of the joints to be controlled
        msg.joint_names = arm->get_joint_names();

        // Create one point in the trajectory
        msg.points.resize(1);
        msg.points[0].positions.resize(msg.joint_names.size(), 0.0);

        // Set joint positions
        for (size_t idx = 0; idx < positions.size(); ++idx)
        {
            msg.points[0].positions[idx] = positions[idx];
        }

        // How long to take getting to the point (floating point seconds)
        msg.points[0].time_from_start = ros::Duration(time_from_start);

        //ROS_INFO("Sending command: \n%s", msg);

        arm_joint_trajectory_publisher_.publish(msg);
    }

    void gripper_control(bool state)
    {
        osrf_gear::VacuumGripperControl srv;
        srv.request.enable = state;

        gripper_.call(srv);
    }

    bool have_reached_target(geometry_msgs::Pose cur, geometry_msgs::Pose target)
    {
        // TODO: Tune threshold values
        float pos_thresh = 0.01; // Meters
        float rot_thresh = 0.02; // Radians

        float pos_err = fabs(cur.position.x - target.position.x) +
                        fabs(cur.position.y - target.position.y) +
                        fabs(cur.position.z - target.position.z);

        if (pos_err > pos_thresh)
        {
            return false;
        }

        float qx_err = fabs(cur.orientation.x - target.orientation.x);
        float qy_err = fabs(cur.orientation.y - target.orientation.y);
        float qz_err = fabs(cur.orientation.z - target.orientation.z);
        float qw_err = fabs(cur.orientation.w - target.orientation.w);

        if (qx_err > rot_thresh)
        {
            return false;
        }

        if (qy_err > rot_thresh)
        {
            return false;
        }

        if (qz_err > rot_thresh)
        {
            return false;
        }

        if (qw_err > rot_thresh)
        {
            return false;
        }

        return true;
    }
};

int main(int argc, char **argv)
{
    cout << "Starting arm_controller_node" << endl;

    // Last argument is the default name of the node.
    ros::init(argc, argv, "arm_controller_node");

    ros::NodeHandle node;

    ArmRepresentation arm;

    ArmController ac(node, &arm);

    // Subscribe to arm destination and joint states channels
    ros::Subscriber arm_destination_sub = node.subscribe(ARM_DESTINATION_CHANNEL, 1, &ArmController::arm_destination_callback, &ac);

    ros::Subscriber gripper_state_sub = node.subscribe(GRIPPER_STATE_CHANNEL, 10, &ArmController::gripper_state_callback, &ac);

    ros::Subscriber joint_state_sub = node.subscribe(ARM_JOINT_STATES_CHANNEL, 10, &ArmController::joint_state_callback, &ac);

    ROS_INFO("Setup complete");

    // Execute callbacks on new data until ctrl-c
    ros::spin();

    return 0;
}
