#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <memory>
#include <iostream>

#include "ros/ros.h"
#include "simulation_env/arm_angles.h"

using std::cout;
using std::endl;

namespace gazebo
{
class JointAngle : public ModelPlugin
{
public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        cout << "Loading JointAngle sensor plugin for arm" << endl;

        model = _parent;

        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&JointAngle::onUpdate, this));
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "arm");
        }

        nh.reset(new ros::NodeHandle("arm_node"));
        publisher = nh->advertise<simulation_env::arm_angles>("arm_positions", 1);
    }

private:
    void onUpdate()
    {
        simulation_env::arm_angles msg;

        // read in the joint angle for each joint in the arm
        msg.shoulder_pivot_angle = model->GetJoint("shoulder_pivot")->Position();
        msg.shoulder_joint_angle = model->GetJoint("shoulder_joint")->Position();
        msg.elbow_joint_angle = model->GetJoint("elbow_joint")->Position();
        msg.wrist_pivot_angle = model->GetJoint("wrist_pivot")->Position();
        msg.wrist_joint_angle = model->GetJoint("wrist_joint")->Position();
        msg.end_effector_pivot_angle = model->GetJoint("end_effector_pivot")->Position();

        // publish the updated sensor measurements
        publisher.publish(msg);

        // make sure the message gets published
        ros::spinOnce();
    }

    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> nh;
    ros::Publisher publisher;
};
GZ_REGISTER_MODEL_PLUGIN(JointAngle)
} // namespace gazebo
