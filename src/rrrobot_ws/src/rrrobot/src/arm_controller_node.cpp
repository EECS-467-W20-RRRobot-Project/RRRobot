// arm_controller_node.cpp

#include <algorithm>
#include <vector>
#include <string>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>


Class ArmController {
public:
    ArmController(ros::NodeHandle & node) : arm_1_has_been_zeroed {
        arm_1_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 10);
    }

    void arm_1_joint_state_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg) {
        ROS_INFO_STREAM_THROTTLE(10, "Joint States arm 1 (throttled to 0.1 Hz):\n" << *joint_state_msg);
        // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
        arm_1_current_joint_states_ = *joint_state_msg;
        if (!arm_1_has_been_zeroed_) {
            arm_1_has_been_zeroed_ = true;
            ROS_INFO("Sending arm to zero joint positions...");
            send_arm_to_zero_state(arm_1_joint_trajectory_publisher_);
        }
    }

    /// Create a JointTrajectory with all positions set to zero, and command the arm.
    void send_arm_to_zero_state(ros::Publisher & joint_trajectory_publisher) {
        // Create a message to send.
        trajectory_msgs::JointTrajectory msg;

        // Fill the names of the joints to be controlled.
        // Note that the vacuum_gripper_joint is not controllable.
        msg.joint_names.clear();
        msg.joint_names.push_back("shoulder_pan_joint");
        msg.joint_names.push_back("shoulder_lift_joint");
        msg.joint_names.push_back("elbow_joint");
        msg.joint_names.push_back("wrist_1_joint");
        msg.joint_names.push_back("wrist_2_joint");
        msg.joint_names.push_back("wrist_3_joint");
        msg.joint_names.push_back("linear_arm_actuator_joint");
        // Create one point in the trajectory.
        msg.points.resize(1);
        // Resize the vector to the same length as the joint names.
        // Values are initialized to 0.
        msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
        // How long to take getting to the point (floating point seconds).
        msg.points[0].time_from_start = ros::Duration(0.001);
        ROS_INFO_STREAM("Sending command:\n" << msg);
        joint_trajectory_publisher.publish(msg);
    }

private:
    ros::Publisher arm_1_joint_trajectory_publisher_;
    sensor_msgs::JointState arm_1_current_joint_states_;
    bool arm_1_has_been_zeroed_;
}


// TODO: Determine message type
void location_callback(const sensor_msgs::/*MessageType*/::ConstPtr & msg) {
    
}


int main(int argc, char ** argv) {
    // Last argument is the default name of the node.
    ros::init(argc, argv, "arm_controller_node");

    ros::NodeHandle node;
    
    // TODO: Subscribe to topic to receive current and destination locations
    ros::Subscriber sub = node.subscribe(/*topic*/, 1, location_callback);

    // TODO: Move arm to pickup item from conveyor belt and turn on suction

    // TODO: Move item to desired position and turn off suction

}