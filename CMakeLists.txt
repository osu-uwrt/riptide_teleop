cmake_minimum_required(VERSION 2.8.3)
project(riptide_teleop)

find_package(catkin REQUIRED
    COMPONENTS
    joy
    tf
    sensor_msgs
    tf2_geometry_msgs
)

catkin_package()