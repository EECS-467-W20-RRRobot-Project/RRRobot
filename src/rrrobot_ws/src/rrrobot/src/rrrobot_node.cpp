// rrrobot_node.cpp

#include <algorithm>
#include <vector>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <fstream>
#include <iostream>

#include <ros/ros.h>

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <osrf_gear/VacuumGripperState.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "topic_names.h"
#include "rrrobot/arm_command.h"

using namespace std;

class Position
{
public:
	float x, y, z;

	Position(float x, float y, float z) : x(x), y(y), z(z){};
	Position() : x(0), y(0), z(0) {}
};

class RRRobot
{
public:
	RRRobot(ros::NodeHandle &node)
		: current_robot_state(RobotState::WAITING_FOR_CLASSIFICATION | RobotState::WAITING_FOR_GRAB_LOCATION),
		  nh(node)
	{
		cv_classification_sub = nh.subscribe(CV_CLASSIFICATION_CHANNEL, 1000, &RRRobot::cv_classification_callback, this);
		gripper_state_sub = nh.subscribe(GRIPPER_STATE_CHANNEL, 1000, &RRRobot::gripper_state_callback, this);
		depth_camera_sub = nh.subscribe(DESIRED_GRASP_POSE_CHANNEL, 1000, &RRRobot::grasp_pose_callback, this);

		conveyor_pub = nh.serviceClient<osrf_gear::ConveyorBeltControl>(CONVEYOR_POWER_CHANNEL);
		arm_destination_pub = nh.advertise<rrrobot::arm_command>(ARM_DESTINATION_CHANNEL, 1000);

		// start competition
		ros::ServiceClient comp_start = nh.serviceClient<std_srvs::Trigger>(START_COMPETITION_CHANNEL);
		std_srvs::Trigger trg;
		comp_start.call(trg);
	}

	void cv_classification_callback(const std_msgs::String &classification)
	{
		std::string type = classification.data.substr(classification.data.find(":") + 1);

		Position drop_point = destination(type);
		desired_drop_pose.position.x = drop_point.x;
		desired_drop_pose.position.y = drop_point.y;
		desired_drop_pose.position.z = drop_point.z;
		
		// update state
		if (current_robot_state & RobotState::WAITING_FOR_GRAB_LOCATION)
		{
			current_robot_state &= ~RobotState::WAITING_FOR_CLASSIFICATION;
		}
		else if ((current_robot_state & RobotState::MOVING_ARM) == 0x0)
		{
			current_robot_state = RobotState::MOVING_ARM;

			// tell the arm to move to grab the object
			publish_arm_command();
		}

		// print_state();
	}

	void gripper_state_callback(const osrf_gear::VacuumGripperState &state)
	{
		if (state.attached)
		{
			current_robot_state = RobotState::MOVING_ARM;

			if (!gripper_state.attached)
			{
				// start the conveyor belt again
				set_conveyor(100);
			}
		}
		// just dropped the object
		else if (gripper_state.attached /* && !state.attached */)
		{
			current_robot_state = RobotState::WAITING_FOR_CLASSIFICATION | RobotState::WAITING_FOR_GRAB_LOCATION;
		}

		// store current state
		gripper_state = state;

		// print_state();
	}

	void grasp_pose_callback(const geometry_msgs::Pose &grasp_pose)
	{
		// stop conveyor belt
		set_conveyor(0);

		desired_grasp_pose = grasp_pose;
		// Add z offset so end effector doesn't hit object
		desired_grasp_pose.position.z += 0.01;

		if (current_robot_state & RobotState::WAITING_FOR_CLASSIFICATION)
		{
			current_robot_state &= ~RobotState::WAITING_FOR_GRAB_LOCATION;
		}
		else if ((current_robot_state & RobotState::MOVING_ARM) == 0x0)
		{
			current_robot_state = RobotState::MOVING_ARM;

			publish_arm_command();
		}

		// print_state();
	}

private:
	enum RobotState
	{
		WAITING_FOR_CLASSIFICATION = 0x1,
		WAITING_FOR_GRAB_LOCATION = 0x1 << 1,
		MOVING_ARM = 0x1 << 2
	};

	int current_robot_state;
	osrf_gear::VacuumGripperState gripper_state;
	geometry_msgs::Pose desired_grasp_pose;
	geometry_msgs::Pose desired_drop_pose;

	ros::NodeHandle nh;

	ros::Subscriber cv_classification_sub;
	ros::Subscriber gripper_state_sub; // know when item has been grabbed, so conveyor can be started
	ros::Subscriber depth_camera_sub;  // get desired grab location

	ros::ServiceClient conveyor_pub;
	ros::Publisher arm_destination_pub;

	Position trash_bin = Position(-0.3, 0.383, 1);
	Position recycle_bin = Position(-0.3, 1.15, 1);

	// Determine item destination bin based on classification
	Position destination(const std::string &type) const
	{
		Position pos;

		if (type == "trash")
		{
			pos = trash_bin;
		}
		else
		{
			pos = recycle_bin;
		}

		return pos;
	}

	// Publish message including grab and drop off location for arm controller
	void publish_arm_command()
	{
		rrrobot::arm_command cmd;
		cmd.grab_location = desired_grasp_pose;
		cmd.drop_location = desired_drop_pose;

		arm_destination_pub.publish(cmd);
		ros::spinOnce();
	}

	// Set conveyor power (0 or 50-100)
	void set_conveyor(int power)
	{
		if (power != 0 && (power < 50 || power > 100))
		{
			return;
		}

		osrf_gear::ConveyorBeltControl cmd;
		cmd.request.power = power;
		conveyor_pub.call(cmd);
	}

	// Print current state for debugging
	void print_state()
	{
		if (current_robot_state & RobotState::WAITING_FOR_CLASSIFICATION)
		{
			cout << "Waiting for classification\t";
		}
		if (current_robot_state & RobotState::WAITING_FOR_GRAB_LOCATION)
		{
			cout << "Waiting for grab location\t";
		}
		if (current_robot_state & RobotState::MOVING_ARM)
		{
			cout << "Moving Arm\t";
		}
		cout << endl;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rrrobot");

	ros::NodeHandle node;

	RRRobot robot(node);
	ros::spin(); // This executes callbacks on new data until ctrl-c.

	return 0;
}
