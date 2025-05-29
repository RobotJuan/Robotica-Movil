cmake_minimum_required(VERSION 3.8)
project(robotica_movil)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Set the scripts
set(PYTHON_SCRIPTS
  nodes/dead_reckoning_nav.py
  nodes/dead_reckoning_nav_lab_2.py
  nodes/pid_angular_pi.py
  nodes/pid_angular_p.py
  nodes/pid_lineal_pi.py
  nodes/pid_lineal_p.py
  nodes/carrot_follower.py
  nodes/path_giver.py
  nodes/carrot_display.py
  nodes/teleoperacion_turtlebot_simulador.py
  nodes/teleoperacion_turtlebot.py
  nodes/obstacle_detector.py
  nodes/pose_loader.py
  nodes/navegacion_final.py
)

# Mark them executable
foreach(script ${PYTHON_SCRIPTS})
  execute_process(COMMAND chmod +x ${CMAKE_CURRENT_SOURCE_DIR}/${script})
endforeach()

# Install them
install(PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
