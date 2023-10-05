# ROS2 Self-Driving Car Package Commands

## Launch the simulation world for the self-driving car

ros2 launch self_driving_car_pkg world.launch.py

## Launch the driver node for the Prius self-driving car

ros2 run self_driving_car_pkg driver_node

## Spawn virtual traffic lights in the simulation environment

ros2 run self_driving_car_pkg lights_spawner.bash

## Start the data recorder node to collect sensor and vehicle data

ros2 run self_driving_car_pkg recorder_node

## Launch the spawner node for dynamic object placement

ros2 run self_driving_car_pkg spawner_node

## Teleoperate the car using the keyboard (for testing and debugging)

ros2 run teleop_twist_keyboard teleop_twist_keyboard

## Start the Gazebo simulator with verbose output and load the world

gazebo --verbose '.WORLD FILE PATH' -s libgazebo_ros_factory.so
