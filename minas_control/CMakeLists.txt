cmake_minimum_required(VERSION 2.8.3)
project(minas_control)

find_package(catkin REQUIRED COMPONENTS ethercat_manager soem)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ## The following work around allows SOEM headers to include other SOEM headers.
  ## SOEM headers assume all headers are installed in a flat directory structure
  ## See https://github.com/smits/soem/issues/4 for more information.
  ${soem_INCLUDE_DIRS}/soem
)

## Declare a C++ library
add_library(minas_client src/minas_client.cpp)

## Declare a C++ executable
add_executable(slaveinfo src/slaveinfo.cpp)
add_executable(simple_test src/simple_test.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(slaveinfo ${catkin_LIBRARIES})
target_link_libraries(simple_test ${catkin_LIBRARIES} minas_client)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS slaveinfo simple_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
