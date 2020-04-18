#include "simulation_env/model_insertion.h"
#include "ros/ros.h"

#include <iostream>

using std::cin;
using std::cout;
using std::endl;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inert_models");

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<simulation_env::model_insertion>("/object_to_load", 1000);

    while (ros::ok())
    {
        simulation_env::model_insertion msg;
        cout << "Enter the model name: ";
        cin >> msg.model_name;
        cout << "Enter the pose: " << endl;
        cout << "x: ";
        cin >> msg.pose.position.x;
        cout << "y: ";
        cin >> msg.pose.position.y;
        cout << "z: ";
        cin >> msg.pose.position.z;

        pub.publish(msg);
        ros::spinOnce();
    }
}