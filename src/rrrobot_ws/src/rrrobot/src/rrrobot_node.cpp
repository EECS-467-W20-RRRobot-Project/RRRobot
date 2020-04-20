// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <vector>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <fstream>

#include <ros/ros.h>

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

/// Example class that can hold state and provide methods that handle incoming data.
class MyCompetitionClass
{
public:
  explicit MyCompetitionClass(ros::NodeHandle & node)
  : current_score_(0), arm_1_has_been_zeroed_(false), arm_2_has_been_zeroed_(false)
  {
    arm_1_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/arm1/arm/command", 10);
  }

  /// Called when a new JointState message is received.
  void arm_1_joint_state_callback(
    const sensor_msgs::JointState::ConstPtr & joint_state_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "Joint States arm 1 (throttled to 0.1 Hz):\n" << *joint_state_msg);
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
};

class Position {
  public:
    float x, y, z;

    Position(float x, float y, float z) : x(x), y(y), z(z) {};
};

Position destination(String type) {
	Position pos;
	float z = 1;
  
	if (type == "cardboard") {
    	pos = Position(-0.3, -1.916, z);
  	}
  	else if (type == "glass") {
    	pos = Position(-0.3, -1.15, z);
  	}
  	else if (type == "metal") {
    	pos = Position(-0.3, -0.383, z);
  	}
  	else if (type == "paper") {
    	pos = Position(-0.3, 0.383, z);
  	}
  	else if (type == "plastic")
  	{
    	pos = Position(-0.3, 1.15, z);
  	}
  	else if (type == "trash")
  	{
    	pos = Position(-0.3, 1.916, z);
  	}

  	return pos;
}

void conveyor_controller(ros::NodeHandle node, int power) {
	if (power != 0 && (power < 50 || power > 100)) {
		return;
	}

	ros::ServiceClient client = node.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
	osrf_gear::ConveyorBeltControl cmd;
	cmd.power = power;
	client.call(cmd);

	return;
}


// void spawn_items(ros::NodeHandle & node) {
// 	ros::Publisher pub = node.advertise<rrrobot::model_insertion>("/object_to_load", 1000);

// 	if (ros::ok())
// 	{
// 		rrrobot::model_insertion msg;
		
// 		// TODO: Randomly select model to spawn
// 		// msg.model_name = "SOMETHING";

// 		// TODO: Determine position to spawn items
// 		// can place objects on conveyor belt just upstream from the arms 
// 		// at roughly (0.238416, 5.474367, 0.935669)
// 		msg.pose.position.x = 0;
// 		msg.pose.position.y = 0;
// 		msg.pose.position.z = 0;

// 		pub.publish(msg);
// 		ros::spinOnce();
// 	}
// }


int main(int argc, char ** argv) {
	// Last argument is the default name of the node.
	ros::init(argc, argv, "rrrobot_node");

	ros::NodeHandle node;

	// Instance of custom class from above.
	MyCompetitionClass comp_class(node);

	// Subscribe to the '/ariac/joint_states' topic.
	ros::Subscriber arm_1_joint_state_subscriber = node.subscribe(
		"/ariac/arm1/joint_states", 10,
		&MyCompetitionClass::arm_1_joint_state_callback, &comp_class);
	
	// TODO: Spawn items (random order)
	//    - Make a call to gazebo model insert script
	spawn_items(node);

	// TODO: Listen to cv_model topic for classification
	// ros::Subscriber laser_profiler_subscriber = node.subscribe(
	//   "/ariac/laser_profiler_1", 10, laser_profiler_callback);

	// TODO: Listen to depth camera handler
	//    - Pass store current object location
	//    - Make call to arm controller (see below)

	// TODO: Start arm controller
	//    - Stop conveyor belt
	//    - Send destination and current location to arm controller
	//    - Wait to receive successful drop-off signal
	//    - Start conveyor belt
	//    - Spawn new items

	ROS_INFO("Setup complete.");
	start_competition(node);
	ros::spin();  // This executes callbacks on new data until ctrl-c.

	return 0;
}
