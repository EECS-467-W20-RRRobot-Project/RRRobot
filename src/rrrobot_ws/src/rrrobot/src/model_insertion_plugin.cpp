#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "ros/ros.h"
#include <std_msgs/String.h>
#include "rrrobot/model_insertion.h"

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

        parent = _parent;

        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "arm");
        }

        nh.reset(new ros::NodeHandle());
        subscriber = nh->subscribe("object_to_load", 1000, &ModelInsertion::insertModel, this);
    }

    void insertModel(const rrrobot::model_insertion &msg)
    {
        std::cout << "Received message to load model: " << msg.model_name << std::endl;

        // Create a new transport node
        transport::NodePtr node(new transport::Node());

        // Initialize the node with the world name
        node->Init(parent->Name());

        // Create a publisher on the ~/factory topic
        transport::PublisherPtr factoryPub =
            node->Advertise<msgs::Factory>("~/factory");

        // Create the message
        msgs::Factory to_pub;

        // Model file to load
        to_pub.set_sdf_filename(std::string("model://") + msg.model_name);

        // Pose to initialize the model to
        msgs::Set(to_pub.mutable_pose(),
                  ignition::math::Pose3d(
                      ignition::math::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
                      ignition::math::Quaterniond(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)));

        // Send the message
        factoryPub->Publish(to_pub);
    }

private:
    physics::WorldPtr parent;
    std::unique_ptr<ros::NodeHandle> nh;
    ros::Subscriber subscriber;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ModelInsertion)
} // namespace gazebo