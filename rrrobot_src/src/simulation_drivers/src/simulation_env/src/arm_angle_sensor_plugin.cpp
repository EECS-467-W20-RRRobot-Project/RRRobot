#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <std_msgs/Float32.h>
#include <memory>
#include <iostream>

#include "ros/ros.h"

using std::cout;
using std::endl;

namespace gazebo
{
  class JointAngle : public ModelPlugin
  {
    public: 
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      cout << "Loading model" << endl;

      model = _parent;

      updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&JointAngle::onUpdate, this));
      if(!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "arm");
      }
      
      nh.reset(new ros::NodeHandle("arm_node"));
      publisher = nh->advertise<std_msgs::Float32>("arm_positions", 1);      
    }

    private:

    void onUpdate()
    {
      //cout << __func__ << endl;
      std_msgs::Float32 msg;
      msg.data = 0.0f;
      gazebo::physics::BasePtr joint = model->GetChild("shoulder_joint");
      //cout << "Joint type: " << joint->GetType() << "joint type & EntityType::JOINT = " << (joint->GetType() & gazebo::physics::Base::EntityType::JOINT) << endl;
      if((joint->GetType() & gazebo::physics::Base::EntityType::JOINT) != 0)
      {
        //cout << "Got an element of type JOINT" << endl;
        //cout << "Joint Position: " << dynamic_cast<gazebo::physics::Joint*>(joint.get())->Position() << endl;
        msg.data = dynamic_cast<gazebo::physics::Joint*>(joint.get())->Position();
	//cout << "Position: " << msg.data << endl;
      }
      if(msg.data != 0.0f) publisher.publish(msg);
      ros::spinOnce();
    }
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> nh;
    ros::Publisher publisher;
  };
  GZ_REGISTER_MODEL_PLUGIN(JointAngle)
}
