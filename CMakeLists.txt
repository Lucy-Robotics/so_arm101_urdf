cmake_minimum_required(VERSION 3.10)
project(so_arm101_urdf)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config description docs worlds
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ament_flake8.ini")
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # No C++ sources in this package; uncrustify/cppcheck often missing on RoboStack.
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cppcheck
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(xacro_smoke test/test_xacro_smoke.py
    TIMEOUT 180
    ENV
      "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  ament_add_pytest_test(apply_lerobot_calibration
    test/apply_lerobot_calibration
    TIMEOUT 60
    ENV
      "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
endif()

ament_package()
