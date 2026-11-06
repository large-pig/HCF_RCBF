cmake_minimum_required(VERSION 3.10)
project(tase_hcf_rcbf LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

option(TASE_CORE_ONLY "Build portable control tests without ROS" OFF)

if(NOT TASE_CORE_ONLY)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    franka_hw
    controller_interface
    hardware_interface
    pluginlib
    realtime_tools
  )

  add_message_files(FILES TrialSample.msg)
  generate_messages(DEPENDENCIES std_msgs)

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES tase_hcf_rcbf_controller
    CATKIN_DEPENDS
      roscpp
      std_msgs
      message_runtime
      franka_hw
      controller_interface
      hardware_interface
      pluginlib
      realtime_tools
  )

  include_directories(include ${catkin_INCLUDE_DIRS})

  add_library(tase_hcf_rcbf_controller src/controller.cpp)
  add_dependencies(
    tase_hcf_rcbf_controller
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(tase_hcf_rcbf_controller ${catkin_LIBRARIES})

  if(CATKIN_ENABLE_TESTING)
    add_executable(test_controller_lifecycle tests/test_controller_lifecycle.cpp)
    add_dependencies(
      test_controller_lifecycle
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )
    target_link_libraries(test_controller_lifecycle
      tase_hcf_rcbf_controller
      ${catkin_LIBRARIES}
    )
    add_test(NAME controller_lifecycle COMMAND test_controller_lifecycle)
    add_test(
      NAME final_experiment_configuration
      COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_final_configuration.py
    )
  endif()

  catkin_install_python(
    PROGRAMS scripts/analyze_trials.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(TARGETS tase_hcf_rcbf_controller
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
  install(DIRECTORY include/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  )
  install(DIRECTORY launch config msg
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
  install(FILES controllers.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endif()

include_directories(include)
enable_testing()

function(add_header_test target source)
  add_executable(${target} ${source})
  add_test(NAME ${target} COMMAND ${target})
endfunction()

add_header_test(test_core tests/test_core.cpp)
add_header_test(test_commissioning tests/test_commissioning.cpp)
add_header_test(test_shared_compensation tests/test_shared_compensation.cpp)
add_header_test(test_controlled_injection tests/test_controlled_injection.cpp)
add_header_test(test_moving_boundary_waveforms tests/test_moving_boundary_waveforms.cpp)
