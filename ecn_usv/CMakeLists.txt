cmake_minimum_required(VERSION 3.5)
project(ecn_usv)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/Target.srv")

install(PROGRAMS src/usv_sim.py src/trajectory.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY urdf launch DESTINATION share/${PROJECT_NAME})
  
ament_package()
