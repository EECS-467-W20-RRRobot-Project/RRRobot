#/app/rrrobot_ws/src/rrrobot/scripts/rrrobot_run_no_build.sh &
cd /app/rrrobot_ws/src/rrrobot/src
python3 cv_model.py &
cd /app/rrrobot_ws/devel/lib/rrrobot
./rrrobot_node &
./depth_camera_node >> /dev/null &
./object_spawner_node &
./arm_controller_node &
rostopic echo /arm_controller/destination &
rostopic echo /desired_grasp_pose &
rostopic echo /cv_model &
