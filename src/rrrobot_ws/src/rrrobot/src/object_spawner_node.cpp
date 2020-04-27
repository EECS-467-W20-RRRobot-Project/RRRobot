#include <vector>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <fstream>

#include <ros/ros.h>

#include <osrf_gear/VacuumGripperState.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include "rrrobot/model_insertion.h"
#include <kdl/frames.hpp>

#include <iostream>
using std::cout;
using std::endl;

class ObjectSpawner
{
public:
    ObjectSpawner(const std::string &model_file = "/app/rrrobot_ws/src/gazebo_models/model_mappings.txt")
        : DEFAULT_SPAWN_POINT(1.21825, 4.0, 0.937978),
          CONVEYOR_WIDTH(0.391404)
    {
        // DEFAULT_SPAWN_POINT.x = 0.2516105;
        // DEFAULT_SPAWN_POINT.y = 5.474367;
        // DEFAULT_SPAWN_POINT.z = 0.935669;
        model_pub = nh.advertise<rrrobot::model_insertion>("/object_to_load", 1000);
        image_pub = nh.advertise<std_msgs::String>("/current_image", 1000);
        sub = nh.subscribe("/ariac/arm1/gripper/state", 1000, &ObjectSpawner::objectPlacedCallback, this);
        srand(time(NULL));
        read_in_items(model_file);
    }

    void objectPlacedCallback(const osrf_gear::VacuumGripperState &state)
    {
        static bool prev_state = false;
        static bool placed = false;

        if (!placed)
        {
            spawn_item();
            placed = true;
        }

        //cout << "State received: " << (state.enabled ? "true" : "false") << endl;
        // gripper is released
        if (!state.enabled && prev_state)
            spawn_item();

        prev_state = state.enabled;
    }

private:
    struct ObjectMapping
    {
        std::string model_name;
        std::string image_file;
        std::string correct_classification;
    };

    const KDL::Vector DEFAULT_SPAWN_POINT;
    const float CONVEYOR_WIDTH;
    ros::NodeHandle nh;
    ros::Publisher model_pub;
    ros::Publisher image_pub;
    ros::Subscriber sub;
    std::vector<ObjectMapping> models; // TODO: just string? or do we need
                                       //to store the correct classification too?

    void read_in_items(const std::string &model_file)
    {
        std::ifstream file(model_file);
        std::string cur;
        while (std::getline(file, cur))
        {
            // cout << "cur: " << cur << " cur.find(,): " << cur.find(",") << endl;
            // line format: model_name, image_file, model_classification
            ObjectMapping cur_model;
            cur_model.model_name = cur.substr(0, cur.find(","));
            cur.erase(0, cur.find(",") + 1);
            cur_model.image_file = cur.substr(0, cur.find(","));
            cur.erase(0, cur.find(",") + 1);
            cur_model.correct_classification = cur;
            models.push_back(cur_model);
        }
    }

    void spawn_item()
    {
        if (ros::ok())
        {
            rrrobot::model_insertion msg;
            std_msgs::String image_file_msg;

            int rand_idx = rand() % models.size();
            msg.model_name = models[rand_idx].model_name;
            image_file_msg.data = models[rand_idx].image_file;
            cout << "Spawning " << msg.model_name << endl;

            // Determine position to spawn items
            // can place objects on conveyor belt just upstream from the arms
            // at roughly (0.2516105, 5.474367, 0.935669)
            // x range: 1.022548-1.413952
            // y range: anything (essentially)
            // z
            msg.pose.position.x = DEFAULT_SPAWN_POINT.x() /* + some random error */;
            msg.pose.position.y = DEFAULT_SPAWN_POINT.y();
            msg.pose.position.z = DEFAULT_SPAWN_POINT.z() + 1.0;

            model_pub.publish(msg);
            image_pub.publish(image_file_msg);
            ros::spinOnce();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_spawner");

    ObjectSpawner spawner;

    ros::spin();
}