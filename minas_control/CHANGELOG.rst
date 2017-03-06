^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package minas_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2017-03-06)
------------------
* fix error message Failed to lock memory
* add documents for step 3 (support multiple joints)
* add 1/101 gear reduction
* change docbuild/debbuild to docbuild\_${PROJECT_NAME}/debbuild\_${PROJECT_NAME}, Close `#5 <https://github.com/tork-a/minas/issues/5>`_.
* add tra1_bringup
* add ros_control for minas robot
* doc: run debbuild --no-deps-> fix typo
* Contributors: Tokyo Opensource Robotics Developer 534

0.2.1 (2017-02-04)
------------------
* doc: segmentation fault no longer occour
* update doc for step 3/4
* minas_control: change LICENSE to GPL
* simple_client: support multiple clients
* Contributors: Tokyo Opensource Robotics Developer 534

0.2.0 (2017-02-04)
------------------
* fix readOutput
* update simple_test for both pp and csp mode
* fix setInterpolationTimePeriod, now argumetns is [usec]
* add PDO mapping 4 + position offset, use cyclic synchronous position(csp) mode
* setInterpolationTimePeriod: display more info
* add setInterpolationTimePeriod
* add getPDSOperation, getPDSControl, getPDSStatus
* Contributors: Tokyo Opensource Robotics Developer 534

0.1.2 (2017-01-16)
------------------
* conf.py : add C++ API feature, but not enable due to bad layout
* index.rst: use xml.etree to get version number from package.xml
* doc/index.rst segfault was resolved
* doc/index.rst: resolved permission problem
* Contributors: Tokyo Opensource Robotics Developer 534

0.1.1 (2017-01-15)
------------------
* CMakeLists:txt : debbuild : make sure that we can run sudo
* CMakeLists.txt : forget to install minas_client
* Contributors: Tokyo Opensource Robotics Developer 534

0.1.0 (2017-01-14)
------------------
* add doc and inital index.rst
* CMakeLists.txt : add install include directory
* CMakeLists.txt : add debbuild/docbuild target
* CMakeLists.txt : run setcap to run seom apps without sudo
* update simple_test, set velocity profile
* add setTorqueForEmergencyStop, setOveerLoadLevel, setOverSpeedLevel, setMotorWorrkingRange, setProfileVelocity
* MinasClinet: add reset/servoOn/servoOff method
* minus_control.cpp: usetPDO Default maping 4
* CMakeLists.txt,src/reset.cpp: add reset.cpp
* add CMakeLists.txt package.xml include/minas_control/minas_client.h src/minas_client.cpp src/slaveinfo.cpp src/simple_test.cpp
* Contributors: Tokyo Opensource Robotics Developer 534
