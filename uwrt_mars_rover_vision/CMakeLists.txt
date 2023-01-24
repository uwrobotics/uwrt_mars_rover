cmake_minimum_required(VERSION 3.5)
project(uwrt_mars_rover_vision)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find_package(ZED 3 REQUIRED)
# find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)


# # ZED 3 reqs ----------------------------
# # IF(NOT WIN32)
# #     add_definitions(-Wno-format-extra-args)
# #     SET(SPECIAL_OS_LIBS "pthread" "X11")
# # ENDIF()

# include_directories(${CUDA_INCLUDE_DIRS})
# include_directories(${ZED_INCLUDE_DIRS})
# # include_directories(${OpenCV_INCLUDE_DIRS})
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# link_directories(${ZED_LIBRARY_DIR})
# # link_directories(${OpenCV_LIBRARY_DIRS})
# link_directories(${CUDA_LIBRARY_DIRS})

# FILE(GLOB_RECURSE SRC_FILES src/*.cpp)
# FILE(GLOB_RECURSE HDR_FILES include/*.hpp)

# ADD_EXECUTABLE(${PROJECT_NAME} ${HDR_FILES} ${SRC_FILES})
# add_definitions(-std=c++14 -O3)

# # if (LINK_SHARED_ZED)
# #     SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
# # else()
# #     SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
# # endif()

# TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${ZED_LIBS} )
# # ${OpenCV_LIBRARIES})
# # end zed 3 reqs


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# add custom messages and services
find_package(rosidl_default_generators REQUIRED)

# set(msg_files
# 	"msg/XboxController.msg"	
# )

# rosidl_generate_interfaces(${PROJECT_NAME}
# 	${msg_files}
# 	DEPENDENCIES std_msgs
# )

ament_export_dependencies(rosidl_default_runtime)

#include the 'include' directory
include_directories(include)

#create resource which references the libraries in the binary bin
set(node_plugins "")

# add the aruco target tracker component
add_library(aruco_tracker SHARED 
	src/aruco_target_tracker.cpp)
target_compile_definitions(aruco_tracker
	PRIVATE "UWRT_MARS_ROVER_VISION_DLL")
ament_target_dependencies(aruco_tracker
	"rclcpp"
	"rclcpp_components"
	"sensor_msgs"
	"cv_bridge"
	"image_transport"
	"geometry_msgs"
	# "opencv_"
	)
# build ROS custom messages -> uncomment later
# rosidl_target_interfaces(aruco_tracker ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(aruco_tracker "uwrt_autonomy::TargetTracker")
set(node_plugins "${node_plugins}uwrt_autonomy::TargetTracker;$<TARGET_FILE:aruco_tracker>\n")

# tell where to put binaries
install(TARGETS
  aruco_tracker
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib 
	RUNTIME DESTINATION bin)

ament_package()
