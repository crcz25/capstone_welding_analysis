cmake_minimum_required(VERSION 3.8)
project(ranger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)  

  # Add the include directories to the project
# Add the additional include directories from the SICK Ranger camera located in the installation folder:
# $(IVP_3D_CAMERA)\icon\inc
include_directories("C:/Program Files (x86)/SICK/3D Cameras/icon/inc")
include_directories("C:/Program Files (x86)/SICK/3D Cameras/icon/inc/filter")

# Add the additional library from the SICK Ranger camera located in the installation folder:
# $(IVP_3D_CAMERA)\icon\lib
link_directories("C:/Program Files (x86)/SICK/3D Cameras/icon/lib")

# Add the header files from the SICK Ranger camera located in the installation folder:
# $(IVP_3D_CAMERA)\icon\inc and $(IVP_3D_CAMERA)\icon\inc\filter
set(SICK_RANGER_HEADERS "C:/Program Files (x86)/SICK/3D Cameras/icon/inc")

file(GLOB 
  HEADER_FILES 
  "${SICK_RANGER_HEADERS}/*.h"
  "${SICK_RANGER_HEADERS}/filter/*.h"
)

# Add the library files from the SICK Ranger camera located in the installation folder:
# $(IVP_3D_CAMERA)\icon\lib
set(SICK_RANGER_LIBS "C:/Program Files (x86)/SICK/3D Cameras/icon/lib")

file(GLOB 
  LIB_FILES 
  "${SICK_RANGER_LIBS}/*.lib"
  "${SICK_RANGER_LIBS}/*.dll"
)

# Set the include directories
# include_directories(${SICK_RANGER_HEADERS})

# Declare a C++ executable
add_executable(scan 
src/scan.cpp
src/DataAccess.cpp
src/dataformatexample.cpp
src/ThreadSafePrint.cpp
${HEADER_FILES} ${LIB_FILES})


target_include_directories(scan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(scan PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  scan
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "interfaces"
)

install(TARGETS
  scan
  DESTINATION lib/${PROJECT_NAME})

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
