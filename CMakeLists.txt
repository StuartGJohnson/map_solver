cmake_minimum_required(VERSION 3.8)
project(map_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(CORE_LIBRARY_TARGET ${PROJECT_NAME}_core)

add_library(${CORE_LIBRARY_TARGET}
  src/localization_search.cpp
  src/map_model.cpp
  src/scan_bag_converter.cpp
  src/scan_matcher.cpp
  src/sim.cpp
)
target_compile_features(${CORE_LIBRARY_TARGET} PUBLIC cxx_std_17)
target_include_directories(${CORE_LIBRARY_TARGET} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${CORE_LIBRARY_TARGET}
  nav_msgs
  geometry_msgs
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
  tf2
)
target_link_libraries(${CORE_LIBRARY_TARGET}
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${CORE_LIBRARY_TARGET}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${CORE_LIBRARY_TARGET})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  nav_msgs
  geometry_msgs
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
  tf2
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_localization
    tests/test_localization.cpp
  )
  if(TARGET test_localization)
    target_compile_features(test_localization PUBLIC cxx_std_17)
    target_link_libraries(test_localization ${CORE_LIBRARY_TARGET})
  endif()

  ament_add_gtest(test_scan_pipeline
    tests/test_scan_pipeline.cpp
  )
  if(TARGET test_scan_pipeline)
    target_compile_features(test_scan_pipeline PUBLIC cxx_std_17)
    target_compile_definitions(test_scan_pipeline PRIVATE
      MAP_SOLVER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(test_scan_pipeline ${CORE_LIBRARY_TARGET})
  endif()
endif()

ament_package()
