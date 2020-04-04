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
      if(!ros::isInitialized())
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
      msg.shoulder_pivot_angle = dynamic_cast<gazebo::physics::Joint*>(model->GetChild("shoulder_pivot").get())->Position();
      msg.shoulder_joint_angle = dynamic_cast<gazebo::physics::Joint*>(model->GetChild("shoulder_joint").get())->Position();
      msg.elbow_joint_angle = dynamic_cast<gazebo::physics::Joint*>(model->GetChild("elbow_joint").get())->Position();
      msg.wrist_pivot_angle = dynamic_cast<gazebo::physics::Joint*>(model->GetChild("wrist_pivot").get())->Position();
      msg.wrist_joint_angle = dynamic_cast<gazebo::physics::Joint*>(model->GetChild("wrist_joint").get())->Position();
      msg.end_effector_pivot_angle = dynamic_cast<gazebo::physics::Joint*>(model->GetChild("end_effector_pivot").get())->Position();
      
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
}
