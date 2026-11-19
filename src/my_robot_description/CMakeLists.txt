cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

# ROS installs the Python library and executable node adapters. Gazebo loads
# the C++ shared libraries below into its physics process.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz_dartsim_vendor REQUIRED)
find_package(DART REQUIRED)

# One installation rule for every plugin; each call retains its original libs.
function(add_biped_plugin target source)
  add_library(${target} SHARED ${source})
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_link_libraries(${target} ${ARGN})
  install(TARGETS ${target} LIBRARY DESTINATION lib)
endfunction()

add_biped_plugin(biped_physics_probe plugins/physics_probe.cpp
  gz-sim8::gz-sim8 gz-plugin2::gz-plugin2-register)
add_biped_plugin(biped_viscous_damping plugins/viscous_damping.cpp
  gz-sim8::gz-sim8 gz-plugin2::gz-plugin2-register)
add_biped_plugin(biped_dart_probe plugins/dart_probe.cpp dart ${CMAKE_DL_LIBS})
add_biped_plugin(biped_com_probe plugins/com_probe.cpp
  gz-sim8::gz-sim8 gz-plugin2::gz-plugin2-register)
add_biped_plugin(biped_contact_state_probe plugins/contact_state_probe.cpp
  gz-sim8::gz-sim8 gz-plugin2::gz-plugin2-register)

ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY urdf launch worlds config DESTINATION share/${PROJECT_NAME})

# Source folders explain responsibility. Installed executable basenames remain
# unchanged so the ROS launch API and existing service commands still work.
install(PROGRAMS
  nodes/control/robot_command.py
  nodes/control/hold_roll_stance.py
  nodes/sensing/sample_contacts.py
  nodes/sensing/show_center_of_mass.py
  nodes/legacy/move_to_stance.py
  nodes/legacy/plan_stance.py
  nodes/legacy/hold_effort_stance.py
  nodes/legacy/observe_balance.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
