cmake_minimum_required(VERSION 3.5)
project(lvi_sam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME} "msg/CloudInfo.msg" DEPENDENCIES std_msgs sensor_msgs)

add_executable(${PROJECT_NAME}_featureExtraction src/lidar_odometry/featureExtraction.cpp)
ament_target_dependencies(${PROJECT_NAME}_featureExtraction rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
rosidl_target_interfaces(${PROJECT_NAME}_featureExtraction ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(${PROJECT_NAME}_imageProjection src/lidar_odometry/imageProjection.cpp)
ament_target_dependencies(${PROJECT_NAME}_imageProjection rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
rosidl_target_interfaces(${PROJECT_NAME}_imageProjection ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(${PROJECT_NAME}_imuPreintegration src/lidar_odometry/imuPreintegration.cpp)
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM Eigen)
target_link_libraries(${PROJECT_NAME}_imuPreintegration gtsam)
rosidl_target_interfaces(${PROJECT_NAME}_imuPreintegration ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(${PROJECT_NAME}_mapOptimization src/lidar_odometry/mapOptimization.cpp)
ament_target_dependencies(${PROJECT_NAME}_mapOptimization rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM)
target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam)
rosidl_target_interfaces(${PROJECT_NAME}_mapOptimization ${PROJECT_NAME} "rosidl_typesupport_cpp")

file(GLOB visual_odometry_files
    "src/visual_odometry/visual_estimator/*.cpp"
    "src/visual_odometry/visual_estimator/factor/*.cpp"
    "src/visual_odometry/visual_estimator/initial/*.cpp"
    "src/visual_odometry/visual_estimator/utility/*.cpp")

add_executable(${PROJECT_NAME}_visual_odometry ${visual_odometry_files})
target_link_libraries(${PROJECT_NAME}_visual_odometry ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_visual_odometry rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL Eigen ament_index_cpp)


install(TARGETS 
        ${PROJECT_NAME}_featureExtraction 
        ${PROJECT_NAME}_imageProjection
        ${PROJECT_NAME}_imuPreintegration
        ${PROJECT_NAME}_mapOptimization
        ${PROJECT_NAME}_visual_odometry

        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})
        


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
