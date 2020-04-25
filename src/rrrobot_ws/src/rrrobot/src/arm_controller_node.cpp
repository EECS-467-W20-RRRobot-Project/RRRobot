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

#include <cstdlib>

using namespace std;

class ArmRepresentation
{
public:
    ArmRepresentation(const KDL::Frame &base_pose = KDL::Frame(KDL::Vector(0.3, 0.92, 1)))
    {
        const double base_len = 0.2;
        const double shoulder_height = 0.1273;
        const double upper_arm_length = 0.612;
        const double forearm_length = 0.5723;
        const double shoulder_offset = 0.220941;
        const double elbow_offset = -0.1719;
        const double wrist_1_length = 0.163941 - elbow_offset - shoulder_offset;
        const double wrist_2_length = 0.1157;
        const double wrist_3_length = 0.0922;

        // KDL::Vector pos; //(0, 0, base_len/2);
        // KDL::Rotation rot;

        // The joints might be off by one
        KDL::Segment linear_arm_actuator("linear_arm_actuator_joint",
                                         KDL::Joint(KDL::Joint::JointType::None), base_pose);
        arm.addSegment(linear_arm_actuator);

        const KDL::Vector pos_base(0, 0, base_len / 2);
        const KDL::Rotation rot_base(KDL::Rotation::RPY(0, 0, 0));
        KDL::Segment base_link("base_link", KDL::Joint(KDL::Joint::JointType::TransY),
                               KDL::Frame(rot_base, pos_base));
        arm.addSegment(base_link);

        const KDL::Vector pos_shoulder(0, 0, shoulder_height);
        const KDL::Rotation rot_shoulder(KDL::Rotation::RPY(0, 0, 0));
        KDL::Segment shoulder_link("shoulder_link", KDL::Joint(KDL::Joint::JointType::RotZ),
                                   KDL::Frame(rot_shoulder, pos_shoulder));
        arm.addSegment(shoulder_link);

        const KDL::Vector pos_upper_arm(0, shoulder_offset, 0);
        const KDL::Rotation rot_upper_arm(KDL::Rotation::RPY(0.0, M_PI / 2.0, 0.0));
        KDL::Segment upper_arm_link("upper_arm_link", KDL::Joint(KDL::Joint::JointType::RotY),
                                    KDL::Frame(rot_upper_arm, pos_upper_arm));
        arm.addSegment(upper_arm_link);

        const KDL::Vector pos_forearm(0, elbow_offset, upper_arm_length);
        const KDL::Rotation rot_forearm(KDL::Rotation::RPY(0.0, 0.0, 0.0));
        KDL::Segment forearm_link("forearm_link", KDL::Joint(KDL::Joint::JointType::RotY),
                                  KDL::Frame(rot_forearm, pos_forearm));
        arm.addSegment(forearm_link);

        const KDL::Vector pos_wrist_1(0, 0, forearm_length);
        const KDL::Rotation rot_wrist_1(KDL::Rotation::RPY(0.0, M_PI / 2.0, 0.0));
        KDL::Segment wrist_1_link("wrist_1_link", KDL::Joint(KDL::Joint::JointType::RotY),
                                  KDL::Frame(rot_wrist_1, pos_wrist_1));
        arm.addSegment(wrist_1_link);

        const KDL::Vector pos_wrist_2(0, wrist_1_length, 0);
        const KDL::Rotation rot_wrist_2(KDL::Rotation::RPY(0.0, 0.0, 0.0));
        KDL::Segment wrist_2_link("wrist_2_link", KDL::Joint(KDL::Joint::JointType::RotZ),
                                  KDL::Frame(rot_wrist_2, pos_wrist_2));
        arm.addSegment(wrist_2_link);

        const KDL::Vector pos_wrist_3(0, 0, wrist_2_length);
        const KDL::Rotation rot_wrist_3(KDL::Rotation::RPY(0.0, 0.0, 0.0));
        KDL::Segment wrist_3_link("wrist_3_link", KDL::Joint(KDL::Joint::JointType::RotY),
                                  KDL::Frame(rot_wrist_3, pos_wrist_3));
        arm.addSegment(wrist_3_link);

        const KDL::Vector pos_ee(0, wrist_3_length, 0.0);
        const KDL::Rotation rot_ee(KDL::Rotation::RPY(0.0, 0.0, M_PI / 2.0));
        KDL::Segment ee_link("ee_link", KDL::Joint(KDL::Joint::JointType::None),
                             KDL::Frame(rot_ee, pos_ee));
        arm.addSegment(ee_link);

        // arm.addSegment(base_link);
        // arm.addSegment(shoulder_link);
        // arm.addSegment(upper_arm_link);
        // arm.addSegment(forearm_link);
        // arm.addSegment(wrist_1_link);
        // arm.addSegment(wrist_2_link);
        // arm.addSegment(wrist_3_link);
        // arm.addSegment(ee_link);

        fk_solver.reset(new KDL::ChainFkSolverPos_recursive(arm));
        ik_solver.reset(new KDL::ChainIkSolverPos_LMA(arm));
    }

    bool calculateForwardKinematics(const KDL::JntArray &joint_positions, KDL::Frame &end_effector_pose)
    {
        return fk_solver->JntToCart(joint_positions, end_effector_pose);
    }

    bool calculateInverseKinematics(const KDL::JntArray &cur_configuration, 
                const KDL::Frame &desired_end_effector_pose, 
                KDL::JntArray &final_joint_configuration)
    {
        return ik_solver->CartToJnt(cur_configuration, desired_end_effector_pose, final_joint_configuration);
    }

    vector<string> get_joint_names() {
        vector<string> joints;
        
        for (int segment = 0; segment < arm.getNrOfSegments(); ++segment) {
            KDL::Joint cur = arm.getSegment(segment).getJoint();
            
            if (cur.getType() != KDL::Joint::JointType::None)
            {
                joints.push_back(cur.getName());
            }
        }

        return joints;
    }

    const KDL::Chain &getArm() const
    {
        return arm;
    }
    
private:
    KDL::Chain arm;

    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver;
};


class ArmController {
public:
    ArmController(ros::NodeHandle & node) : gripper_enabled_(false), item_attached_(false) {
        arm_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(ARM_COMMAND_CHANNEL, 10);

        gripper_ = node.serviceClient<osrf_gear::VacuumGripperControl>(GRIPPER_CONTROL_CHANNEL);

        arm = ArmRepresentation();
    }

    void joint_state_callback(const sensor_msgs::JointState & joint_state_msg) {
        ROS_INFO_STREAM_THROTTLE(10, "Arm Joint States (throttled to 0.1 Hz):\n" << joint_state_msg);
        
        // ROS_INFO_STREAM("Joint States:\n" << joint_state_msg);
        
        // Convert std_msgs::double[] to KDL::JntArray
        int nbr_joints = arm.getArm().getNrOfJoints();
        vector<string> msg_joint_names = joint_state_msg.name;
        vector<string> cur_joint_names = arm.get_joint_names();
        vector<double> position = joint_state_msg.position;

        for (int i = 0; i < nbr_joints; ++i) {
            for (int j = 0; i < nbr_joints; ++j) {
                if (msg_joint_names[i].compare(cur_joint_names[j]) == 0) {
                    arm_current_joint_states_(j) = position[i];
                }
            }
        }
    }

    void gripper_state_callback(const osrf_gear::VacuumGripperState::ConstPtr & gripper_state_msg) {
        gripper_enabled_ = gripper_state_msg->enabled;
        item_attached_ = gripper_state_msg->attached;
    }

    void arm_destination_callback(const rrrobot::arm_command & target_pose) {
        int nbr_joints = arm.getArm().getNrOfJoints();
        vector<double> positions;
        double time_from_start = 5; // Seconds
        
        // Move arm to pickup item from conveyor belt
        positions = calc_joint_positions(target_pose.grab_location);
        send_joint_trajectory(positions, time_from_start);

        // Wait until target position is reached
        while (!have_reached_target(frame_to_pose(calc_end_effector_pose()), target_pose.grab_location)) {
            // TODO: Do something if not able to reach target
            continue;
        }
        
        // Turn on suction
        while (!gripper_enabled_) {
            gripper_control(true);
        }

        // Wait until object is attached
        while (!item_attached_) {
            // TODO: Do something if item is not able to attach
            continue;
        }

        // Move item to desired position
        positions = calc_joint_positions(target_pose.drop_location);
        send_joint_trajectory(positions, time_from_start);
        
        // Wait until target position is reached
        while (!have_reached_target(frame_to_pose(calc_end_effector_pose()), target_pose.drop_location)) {
            // TODO: Do something if not able to reach target
            continue;
        }

        // Turn off suction
        while (gripper_enabled_) {
            gripper_control(false);
        }

        // Wait until object is detached
        while (item_attached_) {
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
    ArmRepresentation arm;

    vector<double> calc_joint_positions(const geometry_msgs::Pose & pose) {
        KDL::JntArray cur_configuration = arm_current_joint_states_;
        KDL::Frame desired_end_effector_pose = pose_to_frame(pose);
        KDL::JntArray final_joint_configuration = KDL::JntArray();
        
        // Check status of IK and print error message if there is a failure
        if (!arm.calculateInverseKinematics(cur_configuration, desired_end_effector_pose, final_joint_configuration)) {
            ROS_ERROR("Inverse Kinematics Failure");
        }

        // Convert data attribute (Eigen::VectorXd) of KDL::JntArray to double[] via data() function
        int nbr_joints = arm.getArm().getNrOfJoints();
        Eigen::VectorXd mat = final_joint_configuration.data;
        vector<double> positions(mat.data(), mat.data() + mat.rows() * mat.cols());
        
        return positions;
    }

    const KDL::Frame& calc_end_effector_pose() {
        KDL::Frame end_effector_pose;
        KDL::JntArray joint_positions = arm_current_joint_states_;
        
        // Check status of FK and do something if there is a failure
        if (!arm.calculateForwardKinematics(joint_positions, end_effector_pose)) {
            ROS_ERROR("Forward Kinematics Failure");
        }

        return end_effector_pose;
    }

    const KDL::Frame& pose_to_frame(const geometry_msgs::Pose & pose) {
        double p_x = pose.position.x;
        double p_y = pose.position.y;
        double p_z = pose.position.z;

        double q_x = pose.orientation.x;
        double q_y = pose.orientation.y;
        double q_z = pose.orientation.z;
        double q_w = pose.orientation.w;

        return KDL::Frame(KDL::Rotation::Quaternion(q_x, q_y, q_z, q_w), KDL::Vector(p_x, p_y, p_z));
    }

    const geometry_msgs::Pose& frame_to_pose(const KDL::Frame & frame) {
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

    void send_joint_trajectory(vector<double> & positions, double time_from_start) {
        // Declare JointTrajectory message
        trajectory_msgs::JointTrajectory msg;
        
        // Fill the names of the joints to be controlled
        msg.joint_names = arm.get_joint_names();
        
        // Create one point in the trajectory
        msg.points.resize(1);

        // Set joint positions
        msg.points[0].positions = positions;
        
        // How long to take getting to the point (floating point seconds)
        msg.points[0].time_from_start = ros::Duration(time_from_start);

        ROS_INFO_STREAM("Sending command:\n" << msg);

        arm_joint_trajectory_publisher_.publish(msg);
    }

    void gripper_control(bool state) {
        osrf_gear::VacuumGripperControl srv;
        srv.request.enable = state;

        gripper_.call(srv);
    }

    bool have_reached_target(geometry_msgs::Pose cur, geometry_msgs::Pose target) {
        // TODO: Tune threshold values
        float pos_thresh = 0.01; // Meters
        float rot_thresh = 0.02; // Radians

        float pos_err = fabs(cur.position.x - target.position.x) +
                        fabs(cur.position.y - target.position.y) +
                        fabs(cur.position.z - target.position.z);
        
        if (pos_err > pos_thresh) {
            return false;
        }

        float qx_err = fabs(cur.orientation.x - target.orientation.x);
        float qy_err = fabs(cur.orientation.y - target.orientation.y);
        float qz_err = fabs(cur.orientation.z - target.orientation.z);
        float qw_err = fabs(cur.orientation.w - target.orientation.w);
        
        if (qx_err > rot_thresh) {
            return false;
        }

        if (qy_err > rot_thresh) {
            return false;
        }

        if (qz_err > rot_thresh) {
            return false;
        }

        if (qw_err > rot_thresh) {
            return false;
        }

        return true;
    }
};


int main(int argc, char ** argv) {
    cout << "Starting arm_controller_node" << endl;
    
    // Last argument is the default name of the node.
    ros::init(argc, argv, "arm_controller_node");

    ros::NodeHandle node;

    ArmController ac(node);
    
    // Subscribe to arm destination and joint states channels
    ros::Subscriber arm_destination_sub = node.subscribe(ARM_DESTINATION_CHANNEL, 1, &ArmController::arm_destination_callback, &ac);

    ros::Subscriber gripper_state_sub = node.subscribe(GRIPPER_STATE_CHANNEL, 10, &ArmController::gripper_state_callback, &ac);
    
    ros::Subscriber joint_state_sub = node.subscribe(ARM_JOINT_STATES_CHANNEL, 10, &ArmController::joint_state_callback, &ac);

    ROS_INFO("Setup complete");

    // Execute callbacks on new data until ctrl-c
    ros::spin();

    return 0;
}
