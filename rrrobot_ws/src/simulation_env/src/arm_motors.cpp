#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <memory>
#include <iostream>

#include "ros/ros.h"
#include "simulation_env/arm_command.h"

using std::cout;
using std::endl;

namespace gazebo
{
class ArmControl : public ModelPlugin
{
public:
    void arm_command_callback(const simulation_env::arm_command &cmd)
    {
        // update the torque applied to each joint when a message is received
        model->GetJoint("shoulder_pivot")->SetForce(0, cmd.shoulder_pivot_force);
        model->GetJoint("shoulder_joint")->SetForce(0, cmd.shoulder_joint_force);
        model->GetJoint("elbow_joint")->SetForce(0, cmd.elbow_joint_force);
        model->GetJoint("wrist_pivot")->SetForce(0, cmd.wrist_pivot_force);
        model->GetJoint("wrist_joint")->SetForce(0, cmd.wrist_joint_force);
        model->GetJoint("end_effector_pivot")->SetForce(0, cmd.end_effector_pivot_force);

        ros::spinOnce();
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        cout << "Loading motor control plugin for arm" << endl;

        model = _parent;

        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "arm");
        }

        nh.reset(new ros::NodeHandle("arm_node"));
        subscriber = nh->subscribe("arm_commands", 1000, &ArmControl::arm_command_callback, this);
    }

private:
    physics::ModelPtr model;
    std::unique_ptr<ros::NodeHandle> nh;
    ros::Subscriber subscriber;
};
GZ_REGISTER_MODEL_PLUGIN(ArmControl)
} // namespace gazebo
