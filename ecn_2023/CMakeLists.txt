cmake_minimum_required(VERSION 3.5)
project(ecn_2023)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/Target.srv")
if("$ENV{ROS_DISTRO}" STRLESS_EQUAL "galactic")
    set(LEGACY_IDL TRUE)
else()
    set(LEGACY_IDL FALSE)
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
endif()

ament_auto_add_executable(uav uav.cpp)
ament_auto_add_executable(uav_target src/uav_target.cpp)

if(${LEGACY_IDL})
    rosidl_target_interfaces(uav ${PROJECT_NAME} "rosidl_typesupport_cpp")
    rosidl_target_interfaces(uav_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
    target_link_libraries(uav "${cpp_typesupport_target}")
    target_link_libraries(uav_target "${cpp_typesupport_target}")
endif()

install(PROGRAMS src/uav_sim.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY urdf launch DESTINATION share/${PROJECT_NAME})
  
ament_auto_package()
