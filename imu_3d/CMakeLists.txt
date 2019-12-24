cmake_minimum_required(VERSION 2.8.3)
project(imu_3d)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")
set (PROJECT_VERSION_MAJOR 0)
set (PROJECT_VERSION_MINOR 7)
set (PROJECT_VERSION_PATCH 0)

#add_subdirectory(../libfreespace libfreespace) ##2018/05/28 Comment. Don't know why use ../
add_subdirectory(libfreespace libfreespace)

set(LIBFREESPACE_LIB_TYPE "${LIBFREESPACE_LIB_TYPE_DEFAULT}" CACHE INTERNAL "")

if (UNIX)
	set(LIBFREESPACE_LIBRARIES ${LIBFREESPACE_LIBRARIES} m)
	set(pthread_lib_name pthread)
elseif (WIN32)
	link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
	set(pthread_lib_name pthreadVSE2)
	# Copy any dlls to the build tree to keep things clean
	FILE(GLOB DLL_LIST "${CMAKE_CURRENT_SOURCE_DIR}/lib/*.dll")
	SET(DLL_DIR "${CMAKE_CURRENT_BINARY_DIR}/dll/")
	FILE(COPY ${DLL_LIST} DESTINATION ${DLL_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  #genmsg    ##可以產生msg
  message_generation
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   inertia.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
	CATKIN_DEPENDS
	message_runtime 
	roscpp rospy 
	std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${LIBFREESPACE_INCLUDE_DIRS}
)

set(APP_CTRL_SRCS src/appControlHandler.c src/appControlHandler.h)
set(MATH_SRCS src/math/quaternion.c src/math/quaternion.h src/math/vec3.h)

## Declare a cpp executable
# imu_3d
add_executable(imu_3d src/imu_3d.cpp ${APP_CTRL_SRCS} ${MATH_SRCS})
add_dependencies(imu_3d imu_3d_gencpp)

target_link_libraries(imu_3d
    ${catkin_LIBRARIES}
    ${pthread_lib_name}
    ${LIBFREESPACE_LIBRARIES}
)

# fsm_9
add_executable(fsm_imu_node src/fsm_imu_node.cpp ${APP_CTRL_SRCS} ${MATH_SRCS})
 add_dependencies(fsm_imu_node fsm_imu_node_gencpp)

 target_link_libraries(fsm_imu_node
   ${catkin_LIBRARIES}
   ${pthread_lib_name}
   ${LIBFREESPACE_LIBRARIES}
 )

# Add this post-build command so that the exe can find the dll

if (CMAKE_COMPILER_IS_GNUCC)
	add_definitions(-Wall) # TODO  should really set cflags
endif()

