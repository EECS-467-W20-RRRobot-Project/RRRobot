#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "ros/ros.h"
#include <std_msgs/String.h>

#include <string>
#include <memory>
#include <iostream>

namespace gazebo
{
class ModelInsertion : public WorldPlugin
{
public:
    void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        std::cout << "Loading model insertion plugin" << std::endl;

        // Option 1: Insert model from file via function call.
        // The filename must be in the GAZEBO_MODEL_PATH environment variable.
        parent = _parent; //->InsertModelFile("model://box");

        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "arm");
        }

        nh.reset(new ros::NodeHandle());
        subscriber = nh->subscribe("object_to_load", 1000, &ModelInsertion::insertModel, this);
    }

    void insertModel(const std_msgs::String &msg)
    {
        std::cout << "Received message to load model: " << msg.data << std::endl;

        parent->InsertModelFile(std::string("model://") + msg.data);
    }

private:
    physics::WorldPtr parent;
    std::unique_ptr<ros::NodeHandle> nh;
    ros::Subscriber subscriber;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ModelInsertion)
} // namespace gazebo