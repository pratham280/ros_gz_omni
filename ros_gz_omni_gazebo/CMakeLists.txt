cmake_minimum_required(VERSION 3.5)

project(ros_gz_omni_gazebo)

# Core build system
find_package(ament_cmake REQUIRED)
find_package(ros_gz_omni_description REQUIRED)

# Gazebo / GZ dependencies (Jazzy = Harmonic)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

# ✅ Jazzy standard: Gazebo Harmonic
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
message(STATUS "Compiling against Gazebo Harmonic (gz-sim8)")

# -----------------------
# BasicSystem
# -----------------------
add_library(BasicSystem SHARED
  src/BasicSystem.cc
)

target_include_directories(BasicSystem PRIVATE include)

target_link_libraries(BasicSystem PRIVATE
  gz-sim8::gz-sim8
)

# -----------------------
# FullSystem
# -----------------------
add_library(FullSystem SHARED
  src/FullSystem.cc
)

target_include_directories(FullSystem PRIVATE include)

target_link_libraries(FullSystem PRIVATE
  gz-sim8::gz-sim8
)

# -----------------------
# OmniDrive
# -----------------------
add_library(OmniDrive SHARED
  src/OmniDrive.cc
  src/math/OmniMath.cc
)

target_include_directories(OmniDrive PRIVATE include)

target_link_libraries(OmniDrive PRIVATE
  gz-sim8::gz-sim8
)

# -----------------------
# Install rules
# -----------------------
install(
  TARGETS BasicSystem FullSystem OmniDrive
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# -----------------------
# Testing
# -----------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# -----------------------
# Environment hooks
# -----------------------
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
