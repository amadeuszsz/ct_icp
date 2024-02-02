cmake_minimum_required(VERSION 3.14)
project(ct_icp_odometry)

# -- The Superbuild install dir is required for the SLAM
get_filename_component(REAL_CURRENT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} REALPATH)

if (NOT SUPERBUILD_INSTALL_DIR)
    set(SUPERBUILD_INSTALL_DIR ${REAL_CURRENT_SOURCE_DIR}/../../install)
    get_filename_component(SUPERBUILD_INSTALL_DIR ${SUPERBUILD_INSTALL_DIR} ABSOLUTE)
    message(INFO "The CMAKE variable `SUPERBUILD_INSTALL_DIR` pointing to the superbuild is not defined. Setting it to:  ${SUPERBUILD_INSTALL_DIR}")
endif ()

# -- Find shared libcmake --build . --target install --config Release --parallel 12rary dependencies
if (NOT CT_INSTALL_DIR)
    set(CT_INSTALL_DIR ${SUPERBUILD_INSTALL_DIR}/CT_ICP)
endif ()

set(SLAM_CMAKE_SCRIPTS_PATH ${CT_INSTALL_DIR}/cmake)
include(${CT_INSTALL_DIR}/lib/cmake/cmake/includes.cmake)
SLAM_DEFAULT_VARIABLES()
SLAM_INCLUDE_SUPERBUILD()
include(${CT_INSTALL_DIR}/lib/cmake/cmake/external.cmake)

set(CT_ICP_DIR ${CT_INSTALL_DIR}/lib/cmake)
set(SlamCore_DIR ${CT_ICP_DIR})
# set(ROSCore_DIR  ${CT_ICP_DIR})

# -- SlamCore dependency
find_package(SlamCore REQUIRED)
find_package(CT_ICP REQUIRED)
# find_package(ROSCore REQUIRED)
find_package(OpenMP)

# -- Find Superbuild dependencies
SLAM_COMPILER_FLAGS()
SLAM_DEFAULT_VARIABLES()
SLAM_INCLUDE_SUPERBUILD()
find_package(Boost REQUIRED)

# -- Find ROS 2 dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(CT_ICP_ODOMETRY_NODE_SRC
  src/ct_icp_odometry_node.cpp
  src/roscore/nav_msgs_conversion.cpp
  src/roscore/pc2_conversion.cpp
)

set(CT_ICP_ODOMETRY_NODE_HEADERS
  include/ct_icp_odometry/ct_icp_odometry_node.hpp
  include/ct_icp_odometry/utils.hpp
  include/ct_icp_odometry/roscore/nav_msgs_conversion.hpp
  include/ct_icp_odometry/roscore/pc2_conversion.hpp
  include/ct_icp_odometry/visibility_control.hpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${CT_ICP_ODOMETRY_NODE_SRC}
  ${CT_ICP_ODOMETRY_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "ct_icp_odometry::CtIcpOdometryNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

target_link_libraries(${PROJECT_NAME}_node
  Slam::SlamCore
  # Slam::ROSCore
  Slam::CT_ICP
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  rviz
)
