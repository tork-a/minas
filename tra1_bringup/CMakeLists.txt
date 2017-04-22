cmake_minimum_required(VERSION 2.8.3)
project(tra1_bringup)

find_package(catkin REQUIRED COMPONENTS)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(DIRECTORY config launch test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    message(status "Testing ${LAUNCH_FILE}")
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()

  file(GLOB TEST_FILES test/*.test)
  foreach(TEST_FILE ${TEST_FILES})
    message(status "Testing ${TEST_FILE}")
    add_rostest(${TEST_FILE})
  endforeach()
endif()

#############
## Release ##
#############

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
# debbuild
# catkin build minas_control --make-args debbuild
# ROS_PARALLEL_JOBS='' catkin_make --pkg minas_control -- debbuild
add_custom_target(debbuild_${PROJECT_NAME}
  COMMAND sudo -n true || (>&2 echo "debbuild need sudo access to make debs" && exit 1)
  COMMAND bloom-generate rosdebian --os-name ubuntu --os-version trusty --ros-distro indigo
  COMMAND MAKEFLAGS= fakeroot debian/rules binary
  COMMAND rm -fr debian obj-x86_64-linux-gnu
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  )
