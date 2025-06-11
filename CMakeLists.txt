cmake_minimum_required(VERSION 3.0.2)
project(linearmpc_panda)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
#     controller_interface
    franka_hw
    franka_msgs 
    franka_control 
    franka_gripper # maybe not needed
    hardware_interface
    pluginlib
    roscpp
    std_msgs
    message_generation
    joint_trajectory_controller
)

find_package(drake REQUIRED)
if(drake_FOUND)
    message(STATUS "drake found")
else()
    message(STATUS "drake not found")
endif()

set(Eigen3_FOUND True)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
if (TARGET Eigen3::Eigen)
    # Use the imported target
    message(STATUS "Eigen library found")
else()
    message(STATUS "could not find eigen library")
endif (TARGET Eigen3::Eigen)

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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

add_message_files(
        FILES
        # DesiredState.msg
        # DesiredTorqueState.msg
        StampedFloat64MultiArray.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS  controller_interface
                        franka_hw
                        franka_msgs
                        franka_control
                        franka_gripper
                        hardware_interface
                        pluginlib
                        roscpp
                        std_msgs
                        message_runtime
)

#################################################################################################
# Add the linear_mpc_prob file
add_library(linear_mpc_prob
    src/linear_mpc_prob.cc
)
target_link_libraries(linear_mpc_prob
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    drake::drake
)
target_include_directories(linear_mpc_prob PUBLIC
        include
        external/cnpy # needed by linear_mpc_prob.cc for matrix sparsity visualization
        )

#################################################################################################
# Add myutils
add_library(myutils
            src/myutils.cc
)
target_link_libraries(myutils
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    drake::drake
    cnpy
)
target_include_directories(myutils PUBLIC
        include
        external/cnpy 
        )
add_subdirectory(external/cnpy) # needed by myutils

#################################################################################################
## this part is just here for testing compilation. The code need to be compiled on panda       ##
## workstation to direct control panda                                                         ##
#################################################################################################
#################################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  src/linearmpc_panda
)
# there is no main() as this is a controller interface. Therefore have to use add_library()
add_library(linearmpc_panda
            src/panda_mpc_torque_controller.cc)

target_link_libraries(linearmpc_panda ${catkin_LIBRARIES} Eigen3::Eigen drake::drake)
add_dependencies(linearmpc_panda ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_include_directories(linearmpc_panda PUBLIC
        include
        external/cnpy # linear_mpc_prob.h still depends on it
        )

install(TARGETS linearmpc_panda
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

###############
## add nodes ##
###############
#################################################################################################
# add mpc_solver_node executable
add_executable(mpc_solver_node 
               src/mpc_solver_node.cc
               )
# link the executable wot the required libraries
target_link_libraries(mpc_solver_node 
                      ${catkin_LIBRARIES} 
                      Eigen3::Eigen 
                      drake::drake
                      linear_mpc_prob
                      myutils)

# add dependencies to ensure proper build order
add_dependencies(mpc_solver_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# include the header files
target_include_directories(mpc_solver_node PUBLIC
        include
        external/cnpy
        )
# install the executable
install(TARGETS mpc_solver_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#################################################################################################
# Add mpc_executor_node executable
add_executable(mpc_executor_node 
               src/mpc_executor_node.cc
               )

# Link the executable to the required libraries
target_link_libraries(mpc_executor_node 
                      ${catkin_LIBRARIES} 
                      Eigen3::Eigen 
                      drake::drake
                      linear_mpc_prob
                      myutils)

# Add dependencies to ensure proper build order
add_dependencies(mpc_executor_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# Include the header files
target_include_directories(mpc_executor_node PUBLIC
        include
        #external/cnpy
        )

# Install the executable
install(TARGETS mpc_executor_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#################################################################################################
# Add drake_state_publisher executable
add_executable(drake_state_publisher
               src/drake_state_publisher.cc
               )

# Link the executable to the required libraries
target_link_libraries(drake_state_publisher
                      ${catkin_LIBRARIES} 
                      Eigen3::Eigen 
                      drake::drake
                      linear_mpc_prob
                      myutils)

# Add dependencies to ensure proper build order
add_dependencies(drake_state_publisher ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# Include the header files
target_include_directories(drake_state_publisher PUBLIC
        include
        external/cnpy
        )

# Install the executable
install(TARGETS drake_state_publisher
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#################################################################################################
# Add move_to_pose node
add_executable(move_to_pose
               src/move_to_pose.cc
               )

# Link the executable to the required libraries
target_link_libraries(move_to_pose
                      ${catkin_LIBRARIES} 
                      Eigen3::Eigen 
                      drake::drake
                      myutils)

# Add dependencies to ensure proper build order
add_dependencies(move_to_pose ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# Include the header files
target_include_directories(move_to_pose PUBLIC
        include
        external/cnpy
        )

# Install the executable
install(TARGETS move_to_pose
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#################################################################################################
# Add move_to_pose_service node
add_executable(move_to_pose_service
               src/move_to_pose.cc
               )

# Link the executable to the required libraries
target_link_libraries(move_to_pose_service
                      ${catkin_LIBRARIES} 
                      Eigen3::Eigen 
                      drake::drake
                      myutils)

# Add dependencies to ensure proper build order
add_dependencies(move_to_pose_service ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# Include the header files
target_include_directories(move_to_pose_service PUBLIC
        include
        external/cnpy
        )

# Install the executable
install(TARGETS move_to_pose_service
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#################################################################################################
# Add all-in-one MPCController
add_executable(linearmpc_controller
               src/linearmpc_controller.cc
               )

# Link the executable to the required libraries
target_link_libraries(linearmpc_controller
                      ${catkin_LIBRARIES} 
                      Eigen3::Eigen 
                      drake::drake
                      linear_mpc_prob
                      myutils)

# Add dependencies to ensure proper build order
add_dependencies(linearmpc_controller ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# Include the header files
target_include_directories(linearmpc_controller PUBLIC
        include
        external/cnpy
        )

#################################################################################################
# Add a simple debug node
add_executable(u_cmd_test
               src/upsampled_ucmd_sub_test.cc
               )

# Link the executable to the required libraries
target_link_libraries(u_cmd_test
                      ${catkin_LIBRARIES} 
                      Eigen3::Eigen 
                      )

# Add dependencies to ensure proper build order
add_dependencies(u_cmd_test ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# Include the header files
target_include_directories(u_cmd_test PUBLIC
        include
        )

# Install the executable
install(TARGETS linearmpc_controller
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#################################################################################################
install(FILES controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )


#################################################################################################
add_executable(fake_u_cmd_debug src/fake_u_cmd_debug.cc)
target_link_libraries(fake_u_cmd_debug ${catkin_LIBRARIES})
add_dependencies(fake_u_cmd_debug ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
