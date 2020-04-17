#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <memory>
#include <iostream>

#include "ros/ros.h"
#include "simulation_env/arm_command.h"
#include <std_msgs/Float64.h>

using std::cout;
using std::endl;

namespace gazebo
{
class ArmControl : public ModelPlugin
{
public:
    void shoulder_pivot_callback(const std_msgs::Float64 &cmd)
    {
        // update the torque applied to each joint when a message is received
        model->GetJoint("shoulder_pivot")->SetForce(0, cmd.data);

        ros::spinOnce();
    }

    void shoulder_joint_callback(const std_msgs::Float64 &cmd)
    {
        // update the torque applied to each joint when a message is received
        model->GetJoint("shoulder_joint")->SetForce(0, cmd.data);

        ros::spinOnce();
    }

    void elbow_joint_callback(const std_msgs::Float64 &cmd)
    {
        // update the torque applied to each joint when a message is received
        model->GetJoint("elbow_joint")->SetForce(0, cmd.data);

        ros::spinOnce();
    }

    void wrist_pivot_callback(const std_msgs::Float64 &cmd)
    {
        // update the torque applied to each joint when a message is received
        model->GetJoint("wrist_pivot")->SetForce(0, cmd.data);

        ros::spinOnce();
    }

    void wrist_joint_callback(const std_msgs::Float64 &cmd)
    {
        // update the torque applied to each joint when a message is received
        model->GetJoint("wrist_joint")->SetForce(0, cmd.data);

        ros::spinOnce();
    }

    void end_effector_pivot_callback(const std_msgs::Float64 &cmd)
    {
        // update the torque applied to each joint when a message is received
        model->GetJoint("end_effector_pivot")->SetForce(0, cmd.data);

        ros::spinOnce();
    }

    void arm_command_callback(const simulation_env::arm_command &cmd)
    {
        // update the torque applied to each joint when a message is received
        // model->GetJoint("shoulder_pivot")->SetForce(0, cmd.shoulder_pivot_force);
        // model->GetJoint("shoulder_joint")->SetForce(0, -cmd.shoulder_joint_force);
        // model->GetJoint("elbow_joint")->SetForce(0, cmd.elbow_joint_force);
        // model->GetJoint("wrist_pivot")->SetForce(0, cmd.wrist_pivot_force);
        // model->GetJoint("wrist_joint")->SetForce(0, cmd.wrist_joint_force);
        // model->GetJoint("end_effector_pivot")->SetForce(0, cmd.end_effector_pivot_force);

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
        shoulder_pivot_sub = nh->subscribe("/arm_node/arm_commands/shoulder_pivot_torque", 1000, &ArmControl::shoulder_pivot_callback, this);
        shoulder_joint_sub = nh->subscribe("/arm_node/arm_commands/shoulder_joint_torque", 1000, &ArmControl::shoulder_joint_callback, this);
        elbow_joint_sub = nh->subscribe("/arm_node/arm_commands/elbow_joint_torque", 1000, &ArmControl::elbow_joint_callback, this);
        wrist_pivot_sub = nh->subscribe("/arm_node/arm_commands/wrist_pivot_torque", 1000, &ArmControl::wrist_pivot_callback, this);
        wrist_joint_sub = nh->subscribe("/arm_node/arm_commands/wrist_joint_torque", 1000, &ArmControl::wrist_joint_callback, this);
        end_effector_sub = nh->subscribe("/arm_node/arm_commands/end_effector_pivot_torque", 1000, &ArmControl::end_effector_pivot_callback, this);

        cout << "Subscribed to all torque channels" << endl;
    }

private:
    physics::ModelPtr model;
    std::unique_ptr<ros::NodeHandle> nh;
    ros::Subscriber subscriber;

    ros::Subscriber shoulder_pivot_sub;
    ros::Subscriber shoulder_joint_sub;
    ros::Subscriber elbow_joint_sub;
    ros::Subscriber wrist_pivot_sub;
    ros::Subscriber wrist_joint_sub;
    ros::Subscriber end_effector_sub;
};
GZ_REGISTER_MODEL_PLUGIN(ArmControl)
} // namespace gazebo
