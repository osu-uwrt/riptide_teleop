cmake_minimum_required(VERSION 3.0.2)
project(riptide_teleop)

find_package(catkin REQUIRED
    COMPONENTS
    joy
    tf
    sensor_msgs
    tf2_geometry_msgs
)

catkin_package()