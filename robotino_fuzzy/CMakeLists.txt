cmake_minimum_required(VERSION 3.8)
project(robotino_fuzzy)

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)


# FuzzyLite configuration

if(NOT fuzzylite_FOUND)
    message(STATUS "FuzzyLite not found via CMake config, trying manual find")
    find_path(FL_INCLUDE_DIR fl/Headers.h
        PATHS /usr/local/include /usr/include
    )
    find_library(FL_LIBRARY fuzzylite
        PATHS /usr/local/lib /usr/lib
    )
    if(FL_INCLUDE_DIR AND FL_LIBRARY)
        set(fuzzylite_FOUND TRUE)
        add_library(fuzzylite::fuzzylite INTERFACE IMPORTED)
        set_target_properties(fuzzylite::fuzzylite PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${FL_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "${FL_LIBRARY}"
        )
    endif()
endif()

# Verify FuzzyLite was found
if(NOT fuzzylite_FOUND)
    message(FATAL_ERROR "FuzzyLite not found! Install with:\n"
        "  git clone https://github.com/fuzzylite/fuzzylite.git\n"
        "  cd fuzzylite && mkdir build && cd build\n"
        "  cmake .. -DFL_BUILD_STATIC=ON\n"
        "  make && sudo make install")
endif()

# --- Install Python Nodes ---

install(PROGRAMS
  robotino_fuzzy/sim_node.py
  robotino_fuzzy/robotino_tcp_bridge.py
  robotino_fuzzy/robotino_viz.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- C++ Nodes ---

install(DIRECTORY resources/
  DESTINATION share/${PROJECT_NAME}/resources
)

add_executable(fuzzy_controller_node src/fuzzy_controller_node.cpp)

# Link libraries (modern ROS 2 + FuzzyLite)
target_link_libraries(fuzzy_controller_node
  fuzzylite::fuzzylite
  ament_index_cpp::ament_index_cpp
)

# ROS 2 dependency handling (keeps ament tools happy)
ament_target_dependencies(fuzzy_controller_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS fuzzy_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install Launch Files ---
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# --- Install Resource Files ---
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()