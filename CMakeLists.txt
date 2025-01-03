cmake_minimum_required(VERSION 2.8.3)
project(manus_ros)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  # fmt
)


# locate the MANUS SDK in the /ext folder
file(GLOB MANUS_SDK RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "ext/MANUS_Core_*")
if (NOT MANUS_SDK)
  message(FATAL_ERROR "MANUS SDK not found. Please download the SDK and install into the /ext folder.")
endif()

# Locate the SDKMinimalClient_Linux folder in the SDK path
file(GLOB MANUS_LINUX_PATH RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${MANUS_SDK}/SDKMinimalClient_Linux")
if (NOT MANUS_LINUX_PATH)
  message(FATAL_ERROR "SDKMinimalClient_Linux folder not found. Please download the SDK and install into the /ext folder.")
endif()


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# ## Declare a catkin package
# catkin_package()
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs 
  #fmt
)

# Include directories
include_directories(
  # include
  ${MANUS_LINUX_PATH}/ManusSDK/include
  ${catkin_INCLUDE_DIRS}
)

add_library(SDKMinimalClient src/SDKMinimalClient.cpp)

add_executable(manus_ros
  # src/SDKMinimalClient.cpp
  src/manus_ros1.cpp
)

# Specify the directory containing the shared library
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${MANUS_LINUX_PATH}/ManusSDK/lib)
set(LIBRARY_FILE ${LIBRARY_DIR}/libManusSDK.so)

# Link the Manus SDK library and other dependencies
target_link_libraries(manus_ros SDKMinimalClient
    ${catkin_LIBRARIES}
    ${LIBRARY_FILE} # Link the library
)

target_include_directories(manus_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(manus_ros PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  manus_ros
  SDKMinimalClient
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  # DESTINATION lib/${PROJECT_NAME}
)

# # Install the MANUS library SO file
# install(FILES ${LIBRARY_FILE} DESTINATION lib/${PROJECT_NAME})

# # Embed the library into the executable
# set_target_properties(manus_ros PROPERTIES
#     INSTALL_RPATH "$ORIGIN"
# )


# if(BUILD_TESTING)
#   find_package(catkin REQUIRED COMPONENTS
#     catkin_lint
#   )
#   catkin_add_nosetests(test)
# endif()

