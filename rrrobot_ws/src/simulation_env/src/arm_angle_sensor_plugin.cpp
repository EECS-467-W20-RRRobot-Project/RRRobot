#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <memory>
#include <iostream>
#include <cmath>

#include "ros/ros.h"
#include "simulation_env/arm_angles.h"
#include <std_msgs/Float64.h>

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
        // publisher = nh->advertise<simulation_env::arm_angles>("arm_positions", 1000);

        shoulder_pivot_pub = nh->advertise<std_msgs::Float64>("/arm_node/arm_positions/shoulder_pivot_angle", 1);
        shoulder_joint_pub = nh->advertise<std_msgs::Float64>("/arm_node/arm_positions/shoulder_joint_angle", 1);
        elbow_joint_pub = nh->advertise<std_msgs::Float64>("/arm_node/arm_positions/elbow_joint_angle", 1);
        wrist_pivot_pub = nh->advertise<std_msgs::Float64>("/arm_node/arm_positions/wrist_pivot_angle", 1);
        wrist_joint_pub = nh->advertise<std_msgs::Float64>("/arm_node/arm_positions/wrist_joint_angle", 1);
        end_effector_pivot_pub = nh->advertise<std_msgs::Float64>("/arm_node/arm_positions/end_effector_pivot_angle", 1);

        std_msgs::Float64 shoulder_pivot_angle;
        shoulder_pivot_angle.data = model->GetJoint("shoulder_pivot")->Position(); //msg.shoulder_pivot_angle;
        std_msgs::Float64 shoulder_joint_angle;
        shoulder_joint_angle.data = model->GetJoint("shoulder_joint")->Position(); //msg.shoulder_joint_angle;
        std_msgs::Float64 elbow_joint_angle;
        elbow_joint_angle.data = model->GetJoint("elbow_joint")->Position(); //msg.elbow_joint_angle;
        std_msgs::Float64 wrist_pivot_angle;
        wrist_pivot_angle.data = model->GetJoint("wrist_pivot")->Position(); //msg.wrist_pivot_angle;
        std_msgs::Float64 wrist_joint_angle;
        wrist_joint_angle.data = model->GetJoint("wrist_joint")->Position(); //msg.wrist_joint_angle;
        std_msgs::Float64 end_effector_pivot_angle;
        end_effector_pivot_angle.data = model->GetJoint("end_effector_pivot")->Position(); //msg.end_effector_pivot_angle;
        // cout << "shoulder_pivot: " << shoulder_pivot_angle.data << endl;
        // cout << "shoulder_joint: " << shoulder_joint_angle.data << endl;
        // cout << "elbow_joint: " << elbow_joint_angle.data << endl;
        // cout << "wrist_pivot: " << wrist_pivot_angle.data << endl;
        // cout << "wrist_joint: " << wrist_joint_angle.data << endl;
        // cout << "end_effector_pivot: " << end_effector_pivot_angle.data << endl;
    }

private:
    void onUpdate()
    {
        simulation_env::arm_angles msg;

        // // read in the joint angle for each joint in the arm
        // msg.shoulder_pivot_angle = model->GetJoint("shoulder_pivot")->Position();
        // msg.shoulder_joint_angle = -model->GetJoint("shoulder_joint")->Position();
        // msg.elbow_joint_angle = -model->GetJoint("elbow_joint")->Position();
        // msg.wrist_pivot_angle = model->GetJoint("wrist_pivot")->Position();
        // msg.wrist_joint_angle = model->GetJoint("wrist_joint")->Position() + M_PI_2;
        // msg.end_effector_pivot_angle = model->GetJoint("end_effector_pivot")->Position();

        // // read in the angular velocity for each joint
        // msg.shoulder_pivot_velocity = model->GetJoint("shoulder_pivot")->GetVelocity(0);
        // msg.shoulder_joint_velocity = -model->GetJoint("shoulder_joint")->GetVelocity(0);
        // msg.elbow_joint_velocity = -model->GetJoint("elbow_joint")->GetVelocity(0);
        // msg.wrist_pivot_velocity = model->GetJoint("wrist_pivot")->GetVelocity(0);
        // msg.wrist_joint_velocity = model->GetJoint("wrist_joint")->GetVelocity(0);
        // msg.end_effector_pivot_velocity = model->GetJoint("end_effector_pivot")->GetVelocity(0);

        // // publish the updated sensor measurements
        // publisher.publish(msg);
        // ros::spinOnce();

        std_msgs::Float64 shoulder_pivot_angle;
        shoulder_pivot_angle.data = model->GetJoint("shoulder_pivot")->Position(); //msg.shoulder_pivot_angle;
        std_msgs::Float64 shoulder_joint_angle;
        shoulder_joint_angle.data = model->GetJoint("shoulder_joint")->Position(); //msg.shoulder_joint_angle;
        std_msgs::Float64 elbow_joint_angle;
        elbow_joint_angle.data = model->GetJoint("elbow_joint")->Position(); //msg.elbow_joint_angle;
        std_msgs::Float64 wrist_pivot_angle;
        wrist_pivot_angle.data = model->GetJoint("wrist_pivot")->Position(); //msg.wrist_pivot_angle;
        std_msgs::Float64 wrist_joint_angle;
        wrist_joint_angle.data = model->GetJoint("wrist_joint")->Position(); //msg.wrist_joint_angle;
        std_msgs::Float64 end_effector_pivot_angle;
        end_effector_pivot_angle.data = model->GetJoint("end_effector_pivot")->Position(); //msg.end_effector_pivot_angle;

        // cout << "Publishing: " << shoulder_pivot_angle.data << '\t' << shoulder_joint_angle.data << '\t' << elbow_joint_angle.data << '\t' << wrist_pivot_angle.data << '\t' << wrist_joint_angle.data << '\t' << end_effector_pivot_angle.data << endl;

        shoulder_pivot_pub.publish(shoulder_pivot_angle);
        // ros::spinOnce();
        shoulder_joint_pub.publish(shoulder_joint_angle);
        // ros::spinOnce();
        elbow_joint_pub.publish(elbow_joint_angle);
        // ros::spinOnce();
        wrist_pivot_pub.publish(wrist_pivot_angle);
        // ros::spinOnce();
        wrist_joint_pub.publish(wrist_joint_angle);
        // ros::spinOnce();
        end_effector_pivot_pub.publish(end_effector_pivot_angle);
        // ros::spinOnce();

        // make sure the message gets published
        ros::spinOnce();
    }

    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> nh;
    ros::Publisher publisher;

    ros::Publisher shoulder_pivot_pub;
    ros::Publisher shoulder_joint_pub;
    ros::Publisher elbow_joint_pub;
    ros::Publisher wrist_pivot_pub;
    ros::Publisher wrist_joint_pub;
    ros::Publisher end_effector_pivot_pub;
};
GZ_REGISTER_MODEL_PLUGIN(JointAngle)
} // namespace gazebo
