// arm_controller_node.cpp

#include <algorithm>
#include <vector>
#include <string>
#include <memory>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

using namespace std;

class ArmController {
public:
    ArmController(ros::NodeHandle & node) : arm_has_been_zeroed_(false) {
        arm_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 10);

        joint_names = ["linear_arm_actuator_joint",
                       "shoulder_pan_joint",
                       "shoulder_lift_joint",
                       "elbow_joint",
                       "wrist_1_joint",
                       "wrist_2_joint",
                       "wrist_3_joint"];
    }

    void joint_state_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg) {
        ROS_INFO_STREAM_THROTTLE(10, "Arm Joint States (throttled to 0.1 Hz):\n" << *joint_state_msg);
        
        // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
        
        arm_current_joint_states_ = *joint_state_msg;
        
        if (!arm_has_been_zeroed_) {
            arm_has_been_zeroed_ = true;
            
            ROS_INFO("Sending arm to zero joint positions...");
            
            float64[] positions = [0, 0, 0, 0, 0, 0, 0];
            float64 time_from_start = 0.001;
            send_joint_trajectory(positions, time_from_start);
        }
    }

    void target_callback(const geometry_msgs::Pose::ConstPtr & target_pose) {
        // TODO: Use IK to determine joint positions from target_pose


        // Send target joint trajectory
        float64[] positions = [0, 0, 0, 0, 0, 0, 0];
        float64 time_from_start = 2;
        send_joint_trajectory(positions, time_from_start);
        
        
    }

private:
    bool arm_has_been_zeroed_;
    ros::Publisher arm_joint_trajectory_publisher_;
    sensor_msgs::JointState arm_current_joint_states_;
    const string[] joint_names_;

    void send_joint_trajectory(const float64[] & positions, const float64 time_from_start) {
        trajectory_msgs::JointTrajectory msg;
        
        // Fill the names of the joints to be controlled
        msg.joint_names = joint_names_;
        
        // Create on point in the trajectory
        msg.points.resize(1);

        // Set joint positions
        msg.points[0].positions = positions;
        
        // How long to take getting to the point (floating point seconds)
        msg.points[0].time_from_start = ros::Duration(time_from_start);

        ROS_INFO_STREAM("Sending command:\n" << msg);

        arm_joint_trajectory_publisher_.publish(msg);
    }
};


int main(int argc, char ** argv) {
    cout << "Starting arm_controller_node" << endl;
    
    // Last argument is the default name of the node.
    ros::init(argc, argv, "arm_controller_node");

    ros::NodeHandle node;

    ArmController ac(node);
    
    // TODO: Subscribe to topic to receive current and destination locations
    ros::Subscriber sub = node.subscribe("/target_pose", 1, &ArmController::target_callback, &ac);
    ros::spin();

    // TODO: Move arm to pickup item from conveyor belt and turn on suction

    // TODO: Move item to desired position and turn off suction

}
