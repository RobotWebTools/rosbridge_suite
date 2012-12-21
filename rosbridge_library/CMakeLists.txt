cmake_minimum_required(VERSION 2.8.3)
project(rosbridge_library)

find_package(catkin REQUIRED COMPONENTS rospy rosservice rostopic)

catkin_package()
catkin_python_setup()

