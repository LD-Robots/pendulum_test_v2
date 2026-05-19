cmake_minimum_required(VERSION 3.16)
project(pendulum_pvt_policy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# ONNX Runtime — prebuilt, fetched by scripts/init_onnxruntime.sh into
# <workspace>/third_party/onnxruntime. The package lives at src/<pkg>, so
# the workspace root is two levels up.
set(ONNXRUNTIME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/onnxruntime")
if(NOT EXISTS "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so")
  message(FATAL_ERROR
    "ONNX Runtime not found at ${ONNXRUNTIME_DIR}. "
    "Run scripts/init_onnxruntime.sh from the workspace root first.")
endif()

add_executable(${PROJECT_NAME}
  src/policy_node.cpp
  src/main.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${ONNXRUNTIME_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  std_msgs
  trajectory_msgs
)

# RPATH so the executable finds libonnxruntime.so in the source-tree
# third_party dir. The launch file also prepends LD_LIBRARY_PATH as a
# safety net (see launch/policy.launch.py).
set_target_properties(${PROJECT_NAME} PROPERTIES
  INSTALL_RPATH "$ORIGIN/../../../../third_party/onnxruntime/lib"
  INSTALL_RPATH_USE_LINK_PATH FALSE
  BUILD_WITH_INSTALL_RPATH FALSE
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
