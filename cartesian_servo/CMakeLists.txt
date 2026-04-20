cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)

project(cartesian_servo)

find_package(catkin REQUIRED COMPONENTS rtt_ros geometry_msgs std_msgs rtt_rosclock eigen_conversions cmake_modules)
find_package(OROCOS-RTT REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

orocos_component(cartesian_servo_module src/CartesianServo.cpp)
target_link_libraries(cartesian_servo_module ${catkin_LIBRARIES})

orocos_generate_package()
