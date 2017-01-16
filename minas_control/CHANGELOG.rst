^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package minas_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
