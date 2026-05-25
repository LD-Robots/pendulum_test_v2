cmake_minimum_required(VERSION 3.8)
project(pendulum_tools)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/action_position_sender.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME action_position_sender
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
