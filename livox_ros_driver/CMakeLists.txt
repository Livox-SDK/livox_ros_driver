cmake_minimum_required(VERSION 2.8.3)

project(livox_ros_driver)

set(PROJECT_LIDAR livox_lidar)
set(PROJECT_HUB livox_hub)
set(PROJECT_LIDAR_SRC ./livox_lidar.cpp)
set(PROJECT_HUB_SRC ./livox_hub.cpp)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(Boost REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

## get pointcloud package
#find_package(PCL REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()



################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(FILES
	CustomPoint.msg
	CustomMsg.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
    std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS
	roscpp rospy std_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	${catkin_INCLUDE_DIRS}
	#${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/lidar_sdk.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
##SET(SRC_DIR
##    ./
##)

# traverse all source directories and add include dir
##FOREACH(dir ${SRC_DIR})
##	MESSAGE(STATUS"src_dir: "${dir})
##	include_directories(./include/${dir})
##	include_directories(./${dir})
##	AUX_SOURCE_DIRECTORY(${dir} source_list)
##ENDFOREACH()

find_package(PkgConfig)
pkg_check_modules(APR apr-1)
if (APR_FOUND)
	message(${APR_INCLUDE_DIRS})
	message(${APR_LIBRARIES})
endif (APR_FOUND)

include_directories(
	./
	${APR_INCLUDE_DIRS}
)

## PCL library
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

## make sure the livox_sdk_static library is installed
find_library(LIVOX_SDK_LIBRARY liblivox_sdk_static.a /usr/local/lib)

if((NOT LIVOX_SDK_LIBRARY) OR (NOT EXISTS ${LIVOX_SDK_LIBRARY}))
	# couldn't find the livox sdk library
	message("Coudn't find livox sdk library!")
	message("Download Livox-SDK from github and build&install it please!")
	message("Download Livox-SDK from github and build&install it please!")
	message("Download Livox-SDK from github and build&install it please!")

	message("git clone Livox-SDK from github temporarily, only for ROS distro jenkins build!")
	
	# clone livox sdk source code from github
	execute_process(COMMAND rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/Livox-SDK OUTPUT_VARIABLE cmd_res)
	message("Try to pull the livox sdk source code from github")
	FOREACH(res ${cmd_res})
		MESSAGE(${res})
	ENDFOREACH()

	execute_process(COMMAND git clone https://github.com/Livox-SDK/Livox-SDK.git ${CMAKE_CURRENT_SOURCE_DIR}/Livox-SDK OUTPUT_VARIABLE cmd_res)
	FOREACH(res ${cmd_res})
		MESSAGE(${res})
	ENDFOREACH()
	
	execute_process(COMMAND cmake .. WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Livox-SDK/build OUTPUT_VARIABLE cmd_res)
	FOREACH(res ${cmd_res})
		MESSAGE(${res})
	ENDFOREACH()

	execute_process(COMMAND make WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Livox-SDK/build OUTPUT_VARIABLE cmd_res)
	FOREACH(res ${cmd_res})
		MESSAGE(${res})
	ENDFOREACH()
	
	include_directories(
		./
		${CMAKE_CURRENT_SOURCE_DIR}/Livox-SDK/sdk_core/include
	)
	
	link_directories(
		./
		${CMAKE_CURRENT_SOURCE_DIR}/Livox-SDK/build/sdk_core
	)

	## add lidar project here
	add_executable(${PROJECT_LIDAR}_node
		${PROJECT_LIDAR_SRC}
	)

	add_dependencies(${PROJECT_LIDAR}_node ${PROJECT_NAME}_generate_messages_cpp)

	target_link_libraries(${PROJECT_LIDAR}_node
		livox_sdk_static.a
		${APR_LIBRARIES}
		${catkin_LIBRARIES}
		${Boost_LIBRARIES}
		-lrt
	)

	## add hub project here
	add_executable(${PROJECT_HUB}_node
		${PROJECT_HUB_SRC}
	)

	add_dependencies(${PROJECT_LIDAR}_node ${PROJECT_NAME}_generate_messages_cpp)

	target_link_libraries(${PROJECT_HUB}_node
		livox_sdk_static.a
		${APR_LIBRARIES}
		${catkin_LIBRARIES}
		${Boost_LIBRARIES}
		-lrt
	)

else()
	message("find livox sdk library success")

	## add lidar project here
	add_executable(${PROJECT_LIDAR}_node
		${PROJECT_LIDAR_SRC}
	)

	add_dependencies(${PROJECT_LIDAR}_node ${PROJECT_NAME}_generate_messages_cpp)

	target_link_libraries(${PROJECT_LIDAR}_node
		livox_sdk_static.a
		${APR_LIBRARIES}
		${catkin_LIBRARIES}
		${Boost_LIBRARIES}
		-lrt
	)

	## add hub project here
	add_executable(${PROJECT_HUB}_node
		${PROJECT_HUB_SRC}
	)

	add_dependencies(${PROJECT_HUB}_node ${PROJECT_NAME}_generate_messages_cpp)

	target_link_libraries(${PROJECT_HUB}_node
		livox_sdk_static.a
		${APR_LIBRARIES}
		${catkin_LIBRARIES}
		${Boost_LIBRARIES}
		-lrt
	)

endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lidar_sdk.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

