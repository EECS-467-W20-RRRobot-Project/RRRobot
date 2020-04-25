// COMPUTER VISION
#define CV_CLASSIFICATION_CHANNEL   "/cv_model"                     // CV Model classifications

// DEPTH CAMERA
#define DESIRED_GRASP_POSE_CHANNEL  "/desired_grasp_pose"           // Pose determined from depth camera

// ARM CONTROLLER
#define ARM_DESTINATION_CHANNEL     "/arm_controller/destination"   // Sent from rrrobot_node to arm_controller when depth camera sees item
#define ARM_COMMAND_CHANNEL         "/ariac/arm1/arm/command"       // Send joint trajectories to arm
#define ARM_JOINT_STATES_CHANNEL    "/ariac/arm1/joint_states"      // See arm's current joint states
#define GRIPPER_STATE_CHANNEL       "/ariac/arm1/gripper/state"     // Get the state of the gripper
#define GRIPPER_CONTROL_CHANNEL     "/ariac/arm1/gripper/control"   // Turn gripper on or off

// COMPETITION
#define START_COMPETITION_CHANNEL   "/ariac/start_competition"      // Start ARIAC competition
#define CONVEYOR_POWER_CHANNEL      "/ariac/conveyor/control"       // Turn conveyor belt on or off