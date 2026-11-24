cmake_minimum_required(VERSION 3.14.4)
project(ben_project11)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # The tests read the installed files, so they run after colcon build.
  # They find them through the install prefix passed here, and put it first
  # on AMENT_PREFIX_PATH themselves while they run.
  ament_add_pytest_test(test_launch_and_config test/test_launch_and_config.py
    ENV BEN_PROJECT11_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    TIMEOUT 120
  )
endif()

ament_package()
