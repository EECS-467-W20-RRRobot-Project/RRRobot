#include "ros/ros.h"
#include "simulation_env/arm_command.h"
#include "simulation_env/arm_angles.h"

class PID_Control
{
public:
    PID_Control(float Kp_in, float Ki_in, float Kd_in)
        : Kp(Kp_in), Ki(Ki_in), Kd(Kd_in)
    {}

    float update(float error)
    {
        if(error > 0 and prev_error < 0 
            || error < 0 and prev_error > 0)
        {
            reset();
        }
        sum_error += error;
        float to_ret = Kp * error + Ki * sum_error + Kd * (error - prev_error);

        prev_error = error;
        return to_ret;
    }

    void reset()
    {
        sum_error = 0;
        prev_error = 0;
    }

private:
    const float Kp;
    const float Ki;
    const float Kd;

    double sum_error;
    double prev_error;
};


PID_Control pid_controller(1000, 1, 0);
ros::Publisher publisher;



void angle_callback(const simulation_env::arm_angles &msg)
{
    // just control the shoulder joint to go to 0 (should be roughly straight up)
    float output = pid_controller.update(0 - msg.shoulder_joint_angle);

    simulation_env::arm_command cmd;
    cmd.shoulder_joint_force = output;

    publisher.publish(cmd);
    ros::spinOnce();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_control_test");

    ros::NodeHandle nh;
    publisher = nh.advertise<simulation_env::arm_command>("/arm_node/arm_commands", 1000);
    ros::Subscriber sub = nh.subscribe("/arm_node/arm_positions", 1000, angle_callback);

    ros::spin();

    return 0;
}