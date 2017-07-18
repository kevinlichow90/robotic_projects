###!/bin/bash

# minimize:
#    overshoot
#    time to goal (rise time?) 
#    settling time

# parameters to vary:
#    gain_angular_velocity
#    gain_velocity
#    stop_orientation_control_distance (possibly)    


gain_angular_velocity_range=100
gain_velocity_range=100

num_of_trials=100

for i in $(seq 1 $num_of_trials)
do
    kill -9 `ps aux | grep ros`
    sleep 1

    kill -9 `ps aux | grep gz`
    sleep 1

    kill -9 `ps aux | grep gazebo`
    sleep 1


    random_angular_velocity_gain=$(($RANDOM % $gain_angular_velocity_range))
    random_velocity_gain=$(($RANDOM % $gain_velocity_range))
    roslaunch two_wheeled_robot_gazebo two_wheeled_robot_world.launch & sleep 15
    echo $random_angular_velocity_gain
    echo $random_velocity_gain
    ../main $random_angular_velocity_gain $random_velocity_gain
done



