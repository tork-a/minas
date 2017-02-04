^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ethercat_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2017-02-04)
------------------
* return number of ether cat clients
* ec_SDOread fails only when ret is minus
* use hex to display Failed to read from
* add position offsset (60b0) to PDO mappoing 4
* read current SDO syncmode, cycle time
* add writeSDO(char, unsigned char)
* add readSDO
* Contributors: Tokyo Opensource Robotics Developer 534

0.1.2 (2017-01-16)
------------------
* ethercat_manager use pre allcoated iomap\_ to avoid segfault soem/test/ also uses this strategy
* Contributors: Tokyo Opensource Robotics Developer 534

0.1.1 (2017-01-15)
------------------

0.1.0 (2017-01-14)
------------------
* CMakeLists.txt : add debbuild target
* CMakeLists.txt : add install include directory
* add writeSDO method
* readInput/readOutput: add boundary check
* initSoem : set to PDO Default maping 4
* :~EtherCatManager(): move to INIT mode in destructor
* add CMakeLists.txt package.xml include/ethercat_manager/ethercat_manager.h src/ethercat_manager.cpp
* Contributors: Tokyo Opensource Robotics Developer 534
