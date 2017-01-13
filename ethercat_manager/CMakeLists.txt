cmake_minimum_required(VERSION 2.8.3)
project(ethercat_manager)

find_package(catkin REQUIRED COMPONENTS roscpp soem)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  LIBRARIES ethercat_manager
)

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
add_library(ethercat_manager
  src/ethercat_manager.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(ethercat_manager  ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ethercat_manager
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


#############
## Testing ##
#############

