cmake_minimum_required(VERSION 3.16...3.26)

unset(CATKIN_INSTALL_INTO_PREFIX_ROOT)
unset(CATKIN_SYMLINK_INSTALL)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# LiDAR input type — selected at build time, no source edits:
#   colcon build --packages-select se3_lio --cmake-args -DLIDAR_TYPE=ouster
set(LIDAR_TYPE "livox" CACHE STRING "LiDAR input type: ouster | livox")
if(LIDAR_TYPE STREQUAL "livox")
    add_compile_definitions(LIDAR_LIVOX)
elseif(LIDAR_TYPE STREQUAL "ouster")
    add_compile_definitions(LIDAR_OUSTER)
else()
    message(FATAL_ERROR "Unknown LIDAR_TYPE '${LIDAR_TYPE}' (expected: ouster | livox)")
endif()
message(STATUS "SE3-LIO LiDAR input type: ${LIDAR_TYPE}")

set(SE3_LIO_CPP_INSTALL_DIR ${PROJECT_SOURCE_DIR}/../cpp/se3_lio/install)
set(SE3_LIO_CPP_DIR ${PROJECT_SOURCE_DIR}/../cpp/se3_lio)
if(EXISTS ${SE3_LIO_CPP_INSTALL_DIR})
    message(STATUS "Using installed SE3-LIO C++ core library from ${SE3_LIO_CPP_INSTALL_DIR}...")
    link_directories(${SE3_LIO_CPP_INSTALL_DIR}/lib/se3_lio)
    add_executable(lio_node src/lio_node.cpp)
    ament_target_dependencies(lio_node
        rclcpp
        nav_msgs
        sensor_msgs
        geometry_msgs
        pcl_conversions
        livox_ros_driver2
        tf2_ros
    )
    set(SE3_LIO_CORE_LIBS core synchronizer pipeline Sophus::Sophus)
    target_link_libraries(lio_node ${SE3_LIO_CORE_LIBS})
    target_include_directories(lio_node
        PRIVATE
        ${SE3_LIO_CPP_INSTALL_DIR}/include/se3_lio
    )
    install(DIRECTORY
        launch
        config
        rviz
        DESTINATION share/${PROJECT_NAME}
    )

    install(TARGETS lio_node
        DESTINATION lib/${PROJECT_NAME}
    )
elseif (EXISTS ${SE3_LIO_CPP_DIR})
    message(STATUS "Building SE3-LIO C++ core library from source...")
    # The core libs (core/synchronizer/pipeline) install side-by-side with
    # lio_node into lib/se3_lio. RPATH=$ORIGIN lets `ros2 run` resolve them at
    # runtime — a from-source colcon install otherwise strips the build RPATH,
    # so the executable can't find libsynchronizer.so/libcore.so/libpipeline.so.
    # Set before add_subdirectory so the core targets inherit it too.
    set(CMAKE_INSTALL_RPATH "$ORIGIN")
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
    add_subdirectory(${PROJECT_SOURCE_DIR}/../cpp/se3_lio ${CMAKE_CURRENT_BINARY_DIR}/se3_lio_cpp)
    include_directories(${PROJECT_SOURCE_DIR}/../cpp/se3_lio)

    add_executable(lio_node src/lio_node.cpp)

    set(SE3_LIO_CORE_LIBS se3_lio::synchronizer se3_lio::pipeline)
    target_link_libraries(lio_node ${SE3_LIO_CORE_LIBS})
    ament_target_dependencies(lio_node
        rclcpp
        nav_msgs
        sensor_msgs
        geometry_msgs
        pcl_conversions
        livox_ros_driver2
        tf2_ros
    )
    target_include_directories(lio_node
        PRIVATE
        ${PCL_INCLUDE_DIRS}
    )

    install(DIRECTORY
        launch
        config
        rviz
        DESTINATION share/${PROJECT_NAME}
    )

    install(TARGETS lio_node
        DESTINATION lib/${PROJECT_NAME}
    )
else()
    message(FATAL_ERROR "SE3-LIO C++ core library not found. Please build the C++ core first.")
endif()

# Fixed-map pure localization (run_loclite_online), interface and cold start as
# in Super-LIO. KISS-Matcher (prior-free global registration) is built from the
# vendored source; its ROBIN dependency is fetched at configure time. Never pick
# up a robin/kiss_matcher left in a shared install prefix by another package
# (lightning-lm / hikari_loclite / Super-LIO vendor the same libraries).
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(TBB REQUIRED)
if(NOT TARGET kiss_matcher::kiss_matcher_core)
    set(USE_SYSTEM_TBB ON CACHE BOOL "Use system TBB for KISS-Matcher" FORCE)
    set(USE_SYSTEM_ROBIN OFF CACHE BOOL "Fetch ROBIN for KISS-Matcher" FORCE)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    add_subdirectory(
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/KISS-Matcher/cpp/kiss_matcher
        ${CMAKE_BINARY_DIR}/_deps/kiss_matcher-build
        EXCLUDE_FROM_ALL)
endif()

# loclite: ROS surface + relocalization shared with the GenZ-LIO fork. The
# vendored GenZ-ICP is compiled against the core's Sophus, so the binary
# carries a single Sophus.
add_library(loclite STATIC
    src/loclite/fc_gate.cpp
    src/loclite/genz_refiner.cpp
    src/loclite/kiss_global_matcher.cpp
    src/loclite/loc_interface.cpp
    src/loclite/relocalizer.cpp
    thirdparty/genz_icp/genz_icp/VoxelHashMap.cpp
    thirdparty/genz_icp/genz_icp/Registration.cpp
)
set_target_properties(loclite PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(loclite
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src ${PCL_INCLUDE_DIRS}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/genz_icp
)
target_link_libraries(loclite
    PUBLIC ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_SEARCH_LIBRARIES}
           ${PCL_KDTREE_LIBRARIES} ${PCL_REGISTRATION_LIBRARIES} Eigen3::Eigen
    PRIVATE kiss_matcher::kiss_matcher_core TBB::tbb Sophus::Sophus
)
# Keyword signature only (works with and without ament_target_dependencies).
foreach(dep rclcpp std_srvs nav_msgs sensor_msgs geometry_msgs std_msgs visualization_msgs
        pcl_conversions tf2_ros)
    if(${dep}_TARGETS)
        target_link_libraries(loclite PUBLIC ${${dep}_TARGETS})
    else()
        target_link_libraries(loclite PUBLIC ${${dep}_LIBRARIES})
        target_include_directories(loclite PUBLIC ${${dep}_INCLUDE_DIRS})
    endif()
endforeach()

add_executable(run_loclite_online src/run_loclite_online.cpp src/loc_node.cpp)
target_link_libraries(run_loclite_online loclite ${SE3_LIO_CORE_LIBS})
if(EXISTS ${SE3_LIO_CPP_INSTALL_DIR})
    target_include_directories(run_loclite_online PRIVATE ${SE3_LIO_CPP_INSTALL_DIR}/include/se3_lio)
endif()
ament_target_dependencies(run_loclite_online
    rclcpp
    nav_msgs
    sensor_msgs
    geometry_msgs
    pcl_conversions
    livox_ros_driver2
    tf2_ros
)
install(TARGETS run_loclite_online DESTINATION lib/${PROJECT_NAME})

# KISS-Matcher and its fetched ROBIN/PMC/xenium carry their own install rules,
# which would put static libs, headers and CMake exports into the prefix and
# collide in dpkg with other packages shipping the same files. Everything is
# linked statically, so drop them after the subtree installed.
# DESTDIR-aware, multiarch-libdir-aware, exact paths only.
include(GNUInstallDirs)
string(CONFIGURE [[
  set(_se3_lio_strip_root "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
  file(REMOVE_RECURSE
    "${_se3_lio_strip_root}/@CMAKE_INSTALL_INCLUDEDIR@/kiss_matcher"
    "${_se3_lio_strip_root}/@CMAKE_INSTALL_INCLUDEDIR@/robin"
    "${_se3_lio_strip_root}/@CMAKE_INSTALL_INCLUDEDIR@/pmc"
    "${_se3_lio_strip_root}/@CMAKE_INSTALL_INCLUDEDIR@/xenium"
    "${_se3_lio_strip_root}/@CMAKE_INSTALL_LIBDIR@/cmake/kiss_matcher"
    "${_se3_lio_strip_root}/@CMAKE_INSTALL_LIBDIR@/cmake/robin"
    "${_se3_lio_strip_root}/@CMAKE_INSTALL_LIBDIR@/cmake/pmc"
    "${_se3_lio_strip_root}/@CMAKE_INSTALL_LIBDIR@/cmake/xenium"
    "${_se3_lio_strip_root}/@CMAKE_INSTALL_LIBDIR@/libkiss_matcher_core.a"
    "${_se3_lio_strip_root}/@CMAKE_INSTALL_LIBDIR@/librobin.a"
    "${_se3_lio_strip_root}/@CMAKE_INSTALL_LIBDIR@/libpmc.a")
]] _se3_lio_strip_code @ONLY)
install(CODE "${_se3_lio_strip_code}")

# Python (rclpy) node — thin wrapper over the pybind `se3_lio` core.
# Run with: ros2 run se3_lio se3_lio_py_node  /  ros2 launch se3_lio run_py.launch.py
install(PROGRAMS
    scripts/se3_lio_node.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME se3_lio_py_node
)

ament_package()

