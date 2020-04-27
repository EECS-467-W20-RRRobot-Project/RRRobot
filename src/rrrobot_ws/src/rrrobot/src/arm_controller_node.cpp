// arm_controller_node.cpp

#include <algorithm>
#include <vector>
#include <string>
#include <memory>

#include <ros/ros.h>
#include <ros/console.h>

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

#include <stdlib.h> /* srand, rand */
#include <time.h>   /* time */
#include <sstream>

#include <cstdlib>
#include <math.h>

using namespace std;

class ArmController
{
public:
    // Constructor
    ArmController(ros::NodeHandle &node, ArmRepresentation *arm_, double time_per_step, int retry_attempts, double item_attach_z_adjustment) : gripper_enabled_(false), item_attached_(false), arm(arm_), time_per_step(time_per_step), retry_attempts(retry_attempts), item_attach_z_adjustment(item_attach_z_adjustment)
    {
        arm_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(ARM_COMMAND_CHANNEL, 10);

        gripper_ = node.serviceClient<osrf_gear::VacuumGripperControl>(GRIPPER_CONTROL_CHANNEL);

        arm_current_joint_states_.resize(arm->getChain()->getNrOfJoints(), 0.0);

        /* initialize random seed: */
        srand(time(NULL));
    }

    // Receive joint state messages
    void joint_state_callback(const sensor_msgs::JointState &joint_state_msg)
    {
        // Copy joint states to private variable arm_current_joint_states_
        int nbr_joints = arm->getChain()->getNrOfJoints();
        vector<string> msg_joint_names = joint_state_msg.name;
        vector<string> cur_joint_names = arm->get_joint_names();
        vector<double> position = joint_state_msg.position;

        // Set all values to 0
        for (int i = 0; i < nbr_joints; ++i)
        {
            arm_current_joint_states_[i] = 0.0;
        }

        // Match up joint names from message to internal joint names order
        for (size_t i = 0; i < position.size(); ++i)
        {
            for (size_t j = 0; j < cur_joint_names.size(); ++j)
            {
                if (msg_joint_names[i].compare(cur_joint_names[j]) == 0)
                {
                    arm_current_joint_states_[j] = position[i];
                }
            }
        }
    }

    // Receive gripper state messages
    void gripper_state_callback(const osrf_gear::VacuumGripperState::ConstPtr &gripper_state_msg)
    {
        // Store message states in private variables
        gripper_enabled_ = gripper_state_msg->enabled;
        item_attached_ = gripper_state_msg->attached;
    }

    // Receive message from RRRobot node with desired pose to pick up object from conveyor belt
    void arm_destination_callback(const rrrobot::arm_command &target_pose)
    {
        ROS_INFO("Received target pose");
        int attempts = 0;
        int nbr_joints = arm->getChain()->getNrOfJoints();
        vector<double> positions;

        // Turn on suction
        while (!gripper_control(true))
        {
            if (attempts == retry_attempts)
            {
                ROS_ERROR("STEP 1: Suction failed, not going to target");
                return;
            }

            ros::Duration(1).sleep();
            attempts++;
        }

        ROS_INFO("STEP 1: Suction turned on");

        // Move arm to pickup item from conveyor belt
        attempts = 1;
        if (calc_joint_positions(target_pose.grab_location, above_conveyor, positions))
            send_joint_trajectory(positions, arm_action_phase::bin_to_belt);

        // Check if target has been reached
        while (!have_reached_target(frame_to_pose(calc_end_effector_pose()), target_pose.grab_location))
        {
            if (attempts == retry_attempts)
            {
                ROS_ERROR("STEP 2: Did not reach target in allotted time");
                return;
            }

            if (calc_joint_positions(target_pose.grab_location, get_randomized_start_state(above_conveyor) /*target_pose.grab_location.position.y)*/, positions))
                send_joint_trajectory(positions, arm_action_phase::bin_to_belt);
            attempts++;
        }

        ROS_INFO("STEP 2: Reached target conveyor belt position");

        attempts = 0;
        geometry_msgs::Pose target_location = target_pose.grab_location;

        // Wait until object is attached
        while (!item_attached_)
        {
            if (attempts == retry_attempts)
            {
                ROS_ERROR("STEP 3: Could not pick up item");
                return;
            }

            // If item is not able to attach, adjust z and try again
            target_location.position.z -= item_attach_z_adjustment; // meters
            if (calc_joint_positions(target_location, arm_current_joint_states_, positions))
                send_joint_trajectory(positions, arm_action_phase::item_pickup_adjustment);
            attempts++;
        }

        ROS_INFO("STEP 3: Item picked up");

        // Move item to desired position
        attempts = 1;
        KDL::Frame end_effector_pose;
        
        if (target_pose.drop_location.position.y > 1.0)
        {
            positions = bin1;
            KDL::JntArray pos(positions.size());
            for (size_t idx = 0; idx < positions.size(); ++idx)
            {
                pos(idx) = positions[idx];
            }
            arm->calculateForwardKinematics(pos, end_effector_pose);
        }
        else
        {
            positions = bin2;
            KDL::JntArray pos(positions.size());
            for (size_t idx = 0; idx < positions.size(); ++idx)
            {
                pos(idx) = positions[idx];
            }
            arm->calculateForwardKinematics(pos, end_effector_pose);
        }

        // Check if target has been reached
        while (!have_reached_target(frame_to_pose(calc_end_effector_pose()), frame_to_pose(end_effector_pose))) //target_pose.drop_location))
        {
            if (attempts == retry_attempts)
            {
                ROS_ERROR("STEP 4: Did not reach target in allotted time");
                return;
            }

            send_joint_trajectory(positions, arm_action_phase::belt_to_bin);
            attempts++;
        }

        ROS_INFO("STEP 4: Reached drop location");

        // Turn off suction
        attempts = 0;
        while (!gripper_control(false))
        {
            if (attempts == retry_attempts)
            {
                ROS_ERROR("STEP 5: Suction not turning off... ¯\\_(ツ)_/¯");
                return;
            }

            ros::Duration(1).sleep();
            attempts++;
        }

        ROS_INFO("STEP 5: Suction turned off");

        // Wait until object is detached
        attempts = 0;
        while (item_attached_)
        {
            if (attempts == retry_attempts)
            {
                ROS_ERROR("STEP 6: Item not detaching.. ¯\\_(ツ)_/¯");
            }

            ros::Duration(1).sleep();
            attempts++;
        }

        ROS_INFO("STEP 6: Dropped item in bin");
    }

private:
    // Private variables
    bool gripper_enabled_;
    bool item_attached_;
    ros::Publisher arm_joint_trajectory_publisher_;
    ros::ServiceClient gripper_;
    vector<double> arm_current_joint_states_;
    ArmRepresentation *arm;
    enum arm_action_phase
    {
        belt_to_bin,
        bin_to_belt,
        item_pickup_adjustment
    };
    double time_per_step;
    int retry_attempts;
    double item_attach_z_adjustment;
    // Define intermediate joint positions
    const vector<double> above_conveyor = {0, 0, -M_PI / 2, M_PI / 2, -M_PI / 2, -M_PI / 2, 0};
    const vector<double> above_bins = {0, M_PI, -M_PI / 2, M_PI / 2, -M_PI / 2, -M_PI / 2, 0};
    const vector<double> bin1 = {0.05, 0.0, -2.2, -2.2, -0.25, 1.5708, 0.0};
    const vector<double> bin2 = {-0.7, 0.0, -2.2, -2.2, -0.25, 1.5708, 0.0};

    // Randomize start state to avoid inverse kinematics getting stuck in local minimum
    vector<double> get_randomized_start_state(const vector<double> &preferred_state)
    {
        vector<double> cur;
        std::stringstream location;
        location << "Random location: ";
        // cur.push_back(y);
        for (size_t pos = 0; pos < arm->getChain()->getNrOfJoints(); ++pos) {
            int val = rand() % 2;
            double diff = 0;
            
            if (val == 1) {
                diff = -0.1;
            }
            else {
                diff = 0.1;
            }
            
            cur.push_back(preferred_state[pos] + diff);
            location << cur[pos] << " ";
        }

        location << '\n';
        ROS_INFO(location.str().c_str());

        return cur;
    }

    // Use inverse kinematics to calculate joint positions to reach desired pose 
    bool calc_joint_positions(const geometry_msgs::Pose &pose, const vector<double> &start_state, vector<double> &positions)
    {
        KDL::Frame desired_end_effector_pose = pose_to_frame(pose);
        KDL::JntArray final_joint_configuration(arm->getChain()->getNrOfJoints());

        // ROS_INFO("cur_configuration (%i x %i)", cur_configuration.rows(), cur_configuration.columns());

        int error_code = arm->calculateInverseKinematics(start_state, desired_end_effector_pose, final_joint_configuration);

        // Check status of IK and print error message if there is a failure
        if (error_code != 0)
        {
            ROS_ERROR("Inverse Kinematics Failure: %i", error_code);
        }

        // Convert data attribute (Eigen::VectorXd) of KDL::JntArray to double[] via data() function
        int nbr_joints = arm->getChain()->getNrOfJoints();
        Eigen::VectorXd mat = final_joint_configuration.data;
        positions.clear();
        std::stringstream pos_str;
        pos_str << "Calculated joint positions: ";
        
        for (size_t idx = 0; idx < arm->getChain()->getNrOfJoints(); ++idx)
        {
            positions.push_back(mat[idx]);
            pos_str << mat[idx] << " ";
        }
        
        pos_str << '\n';

        ROS_INFO(pos_str.str().c_str());

        return (error_code == 0);
    }

    // Use forward kinematics to calculate end effector pose from current joint states
    KDL::Frame calc_end_effector_pose()
    {
        int num_joints = arm->getChain()->getNrOfJoints();

        KDL::Frame end_effector_pose;
        KDL::JntArray joint_positions(num_joints);

        // Copy current joint states values into JntArray
        for (int i = 0; i < num_joints; ++i)
        {
            joint_positions(i) = arm_current_joint_states_[i];
        }

        int error_code = arm->calculateForwardKinematics(joint_positions, end_effector_pose);

        // Check status of FK and do something if there is a failure
        if (error_code != 0)
        {
            ROS_ERROR("Forward Kinematics Failure: %i", error_code);
        }

        return end_effector_pose;
    }

    // Convert from Pose to KDL::Frame
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

    // Convert from KDL::Frame to Pose
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

    // Send desired joint states to joint controller 
    void send_joint_trajectory(const vector<double> &target, arm_action_phase phase)
    {
        // Declare JointTrajectory message
        trajectory_msgs::JointTrajectory msg;

        // Fill the names of the joints to be controlled
        msg.joint_names = arm->get_joint_names();

        int num_points = 3;

        // Create points in the trajectory
        msg.points.resize(num_points);

        for (int i = 0; i < num_points; ++i)
        {
            if (i == 0)
            {
                if (phase == arm_action_phase::belt_to_bin)
                {
                    msg.points[i].positions = above_conveyor;
                }
                else if (phase == arm_action_phase::bin_to_belt)
                {
                    msg.points[i].positions = above_bins;
                }
                else if (phase == arm_action_phase::item_pickup_adjustment)
                {
                    num_points = 1;
                    msg.points.resize(num_points);
                    msg.points[i].positions = target;
                    msg.points[i].time_from_start = ros::Duration(time_per_step);
                    break;
                }

                msg.points[i].time_from_start = ros::Duration((i + 1) * time_per_step);
            }
            else if (i == 1)
            {
                if (phase == arm_action_phase::belt_to_bin)
                {
                    msg.points[i].positions = above_bins;
                }
                else if (phase == arm_action_phase::bin_to_belt)
                {
                    msg.points[i].positions = above_conveyor;
                }

                msg.points[i].time_from_start = ros::Duration((i + 1) * time_per_step);
            }
            else if (i == (num_points - 1))
            {
                msg.points[i].positions = target;
                msg.points[i].time_from_start = ros::Duration((i + 1) * time_per_step);
            }
        }

        arm_joint_trajectory_publisher_.publish(msg);

        // Wait to reach target
        ros::Duration((num_points + 1) * time_per_step).sleep();
    }

    // Turn gripper on or off
    bool gripper_control(bool state)
    {
        osrf_gear::VacuumGripperControl srv;
        srv.request.enable = state;

        bool success = gripper_.call(srv);

        if (!success)
        {
            ROS_ERROR("Could not enable gripper");
        }

        return success;
    }

    // Check if the end effector has reached the target position (within threshold)
    bool have_reached_target(geometry_msgs::Pose cur, geometry_msgs::Pose target)
    {
        // Threshold values
        float pos_thresh = 0.15; // Meters
        float rot_thresh = 0.05;

        float pos_err = fabs(cur.position.x - target.position.x) +
                        fabs(cur.position.y - target.position.y) +
                        fabs(cur.position.z - target.position.z);

        // ROS_INFO_STREAM("pos_err: " << pos_err << " (" << pos_thresh << ")");

        if (pos_err > pos_thresh)
        {
            return false;
        }

        float qx_err = fabs(fabs(cur.orientation.x) - fabs(target.orientation.x));
        float qy_err = fabs(fabs(cur.orientation.y) - fabs(target.orientation.y));
        float qz_err = fabs(fabs(cur.orientation.z) - fabs(target.orientation.z));
        float qw_err = fabs(fabs(cur.orientation.w) - fabs(target.orientation.w));

        // ROS_INFO_STREAM("qx_err: " << qx_err << " (" << rot_thresh << ")");
        // ROS_INFO_STREAM("qy_err: " << qy_err << " (" << rot_thresh << ")");
        // ROS_INFO_STREAM("qz_err: " << qz_err << " (" << rot_thresh << ")");
        // ROS_INFO_STREAM("qw_err: " << qw_err << " (" << rot_thresh << ")");

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
    ROS_INFO("Starting arm_controller_node");

    // Last argument is the default name of the node.
    ros::init(argc, argv, "arm_controller_node");

    ros::NodeHandle node;

    ArmRepresentation arm;

    // Set parameters
    double time_per_step = 3.0; // seconds
    int retry_attempts = 3;
    double item_attach_z_adjustment = 0.05; // meters
    ArmController ac(node, &arm, time_per_step, retry_attempts, item_attach_z_adjustment);

    // Start asynchronous spinner to receive messages on subscribed topics
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // Subscribe to ROS topics
    ros::Subscriber arm_destination_sub = node.subscribe(ARM_DESTINATION_CHANNEL, 1, &ArmController::arm_destination_callback, &ac);
    ros::Subscriber gripper_state_sub = node.subscribe(GRIPPER_STATE_CHANNEL, 1, &ArmController::gripper_state_callback, &ac);
    ros::Subscriber joint_state_sub = node.subscribe(ARM_JOINT_STATES_CHANNEL, 1, &ArmController::joint_state_callback, &ac);

    ROS_INFO("Setup complete");

    // Execute callbacks on new data until ctrl-c
    ros::waitForShutdown();

    return 0;
}
