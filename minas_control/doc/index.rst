MINAS ROS (Robot Operating System) driver
#########################################

Overview
========

The `minas_control` pacakge contains basic control tools for `MINAS-A5B`_ EtherCAT communication driver for indusdtrial robots.

Prerequisite
===============

This simulation software has been testd on the following environment: 

* Ubuntu Linux 14.04.3 "Trusty" 64bit

* `ROS Indigo Igloo <http://wiki.ros.org/indigo>`_

Install Common Components
----------------------------

First install a few components this package needs: `ROS`_ (robotics middleware)

1. Install ROS.

  The following snippet shows a simple way to install `ROS Indigo` on Ubuntu linux 14.04 `Trusty`. For completeness, you're advised to see `ROS wiki <http://wiki.ros.org/indigo/Installation/Ubuntu>`_.

.. code-block:: bash

  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" \
          > /etc/apt/sources.list.d/ros-latest.list'
  wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O -  | sudo apt-key add -
  sudo apt-get update && sudo apt-get install -y python-rosdep
  sudo rosdep init && rosdep update
  
  echo "### For ROS setting" >> ~/.bashrc
  echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
  source ~/.bashrc


Install From Deb
----------------

1. Obtain `minas_control` and `ethercat_driver` deb file (here assumes`ros-indigo-ethercat-driver_0.1.0-0trusty_amd64.deb` and `ros-indigo-minas-control_0.1.0-0trusty_amd64.deb`). Plase it under
   current directoly.

.. code-block:: bash

  sudo apt-get install -y gdebi
  sudo gdebi -n ros-indigo-ethercat-driver_0.1.0-0trusty_amd64.deb
  sudo gdebi -n ros-indigo-minas-control_0.1.0-0trusty_amd64.deb


MINAS-A5B Control Tools
=======================

Before you start we  have to configure `SI1` and `SI2` input selection, Please chenge No. 4.01 from default setting `818181h` to `010101h` and No 4.02 from `28282h` to `020202h` using `PANATERM`_, see page 13 of the `Manual`_.

First new need to know the network adapter neme for the EtherCAT netwok, `ifconfig` will give you the list of network adpater of your fomputer, for example, at a following case, eth0 is your EtherCAT network and we'll use `eth0` here after, if you have different adapter name, please use that neme when you run the application.

.. code-block:: bash

  $ ifconfig            
  eth0      Link encap:Ethernet  HWaddr 68:f7:82:42:0f:bc
            inet6 addr: fe80::6af7:28ff:fe24:fbc/64 Scope:Link
            UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
            RX packets:2901790 errors:0 dropped:124 overruns:0 frame:0
            TX packets:4073359 errors:0 dropped:0 overruns:0 carrier:0
            collisions:0 txqueuelen:1000 
            RX bytes:284659686 (284.6 MB)  TX bytes:516196518 (516.1 MB)
            Interrupt:20 Memory:f0600000-f0620000 
  
  eth1      Link encap:Ethernet  HWaddr 74:03:db:f7:9a:39  
            inet addr:192.169.100.1  Bcast:192.168.100.255  Mask:255.255.255.0
            inet6 addr: fe80::7603:bdff:fe7f:9a39/64 Scope:Link
            UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
            RX packets:38503098 errors:0 dropped:337 overruns:0 frame:0
            TX packets:5419325 errors:0 dropped:0 overruns:0 carrier:0
            collisions:0 txqueuelen:1000 
            RX bytes:4368155082 (4.3 GB)  TX bytes:1391012577 (1.3 GB)
  
  lo        Link encap:Local Loopback  
            inet addr:127.0.0.1  Mask:255.0.0.0
            inet6 addr: ::1/128 Scope:Host
            UP LOOPBACK RUNNING  MTU:65536  Metric:1
            RX packets:11730343164 errors:0 dropped:0 overruns:0 frame:0
            TX packets:11730343164 errors:0 dropped:0 overruns:0 carrier:0
            collisions:0 txqueuelen:0 
            RX bytes:186698529957677 (186.6 TB)  TX bytes:186698529957677 (186.6 TB)

slave_info
----------

Now let's run `salve_info` to show current configuration of your EtherCAT network. Please change `eth4`.

.. code-block:: bash

  $ rosrun minas_control slaveinfo eth0
  SOEM (Simple Open EtherCAT Master)
  Slaveinfo
  Initializing etherCAT master
  wkc = 0
  SOEM found and configured 1 slaves
  RxPDO mapping object index 1 = 1603 ret=3
  TxPDO mapping object index 1 = 1a03 ret=6
  SOEM IOMap size: 46
  
  Slave:1
   Name:MADHT1105B01
   Output size: 168bits
   Input size: 200bits
   State: 8
   Delay: 0[ns]
   Has DC: 1
   DCParentport:0
   Activeports:1.0.0.0
   Configured address: 1001
  
  Finished configuration successfully
  End program

simple_test
-----------

Then let's move to next step. The `simple_test` is the example program to control motors. '-h' or '--help' option will show the usages of this program.

.. code-block:: bash

  $ rosrun minas_control simple_test -h
  MINAS Simple Test using SOEM (Simple Open EtherCAT Master)
  Usage: simple_test [options]
    Available options
      -i, --interface     NIC interface name for EtherCAT network
      -p, --position_mode Sample program using Position Profile (pp) mode (Default)
      -c, --cycliec_mode  Sample program using cyclic synchronous position(csp) mode
      -h, --help          Print this message and exit

On default settings, `simple_test` will servo on, rotate about 360 degree and servo off. The `simple_test` program basically follow the instruction described in the manual, i.e Start up guide in p.3 and Motion of `pp` control mode in p. 107. Basic flow of the cpp program as follows.

.. code-block:: cpp

  minas_control::MinasInput input = client->readInputs();
  int32 current_position = input.position_actual_value;

  // set target position
  minas_control::MinasOutput output;
  output.target_position = (current_position > 0)?
              (current_position - 0x100000):(current_position + 0x100000);

  output.max_motor_speed = 120;  // rad/min
  output.target_torque = 500;    // 0% (unit 0.1%)
  output.max_torque    = 500;    // 50% (unit 0.1%)
  output.controlword   = 0x001f; // move to operation enabled +
                                 // new-set-point (bit4) +
                                 //  change set immediately (bit5)

  output.operation_mode = 0x01; // (pp) position profile mode

  // set profile velocity
  client->setProfileVelocity(0x20000000);

  // pp control model setup (see statusword(6041.h) 3) p.107)
  client->writeOutputs(output);
  while ( ! (input.statusword & 0x1000) ) {// bit12 (set-point-acknowledge)
    input = client->readInputs();
  }
  output.controlword   &= ~0x0010; // clear new-set-point (bit4)
  client->writeOutputs(output);

To run `simple_test` with pp mode, use `-p` option.

.. code-block:: bash

  $ rosrun minas_control simple_test -p -i eth0
  SOEM (Simple Open EtherCAT Master)
  Simple test
  Initializing etherCAT master
  wkc = 1
  SOEM found and configured 1 slaves
  RxPDO mapping object index 1 = 1603 ret=3
  TxPDO mapping object index 1 = 1a03 ret=6
  SOEM IOMap size: 46
  
  Slave:1
   Name:MADHT1105B01
   Output size: 168bits
   Input size: 200bits
   State: 8
   Delay: 0[ns]
   Has DC: 1
   DCParentport:0
   Activeports:1.0.0.0
   Configured address: 1001
  
  Finished configuration successfully
  Switch on disabled
  Ready to switch on
  Switched on
  Switched on
  Switched on
  Switched on
  Switched on
  Switched on
  Operation enabled
  target position = fff35147
  err = 0000, ctrl 001f, status 1237, op_mode =  1, pos = 00035148, vel = 00000000, tor = 00000000
  err = 0000, ctrl 001f, status 1237, op_mode =  1, pos = 00035141, vel = fffffe0c, tor = 0000fffe
  ...
  err = 0000, ctrl 001f, status 1237, op_mode =  1, pos = fff35138, vel = 00000000, tor = 0000000b
  err = 0000, ctrl 001f, status 1637, op_mode =  1, pos = fff35140, vel = 000000fa, tor = 0000000a
  target reached
  Operation enabled
  Switched on
  Ready to switch on
  Switch on disabled
  Segmentation fault

If you run `simple_test` with `-c` option, it will servo on, rotate about 180 degree back and forth with sin curve and servo off. Basic flow of the cpp program as follows.

.. code-block:: cpp

  client->setInterpolationTimePeriod(4000);     // 4 msec

  minas_control::MinasInput input = client->readInputs();
  int32 current_position = input.position_actual_value;

  // set target position
  minas_control::MinasOutput output;
  output.target_position = current_position;

  output.max_motor_speed = 120;  // rad/min
  output.target_torque = 500;    // 0% (unit 0.1%)
  output.max_torque    = 500;    // 50% (unit 0.1%)
  output.controlword   = 0x001f; // move to operation enabled + new-set-point (bit4) + change set immediately (bit5)

  output.operation_mode = 0x08; // (csp) cyclic synchronous position mode

  client->writeOutputs(output);

  struct timespec tick;
  clock_gettime(CLOCK_REALTIME, &tick);

  while ( 1 ) {

    output.position_offset = 0x80000*sin(i/200.0);
    client->writeOutputs(output);

    // sleep for next tick
    timespecInc(tick, period);
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
  }

Note that at this moment, this program exit with `Segmentaiton Fault`. That is expected behavior and you do not have to worried about that.


reset
-----

If you have somethig wrong, you can run reset command. If you still have issue, use `PANATERM`_ to clear alarms.

.. code-block:: bash

  $ rosrun minas_control reset eth0
  SOEM (Simple Open EtherCAT Master)
  Simple test
  Initializing etherCAT master
  wkc = 1
  SOEM found and configured 1 slaves
  RxPDO mapping object index 1 = 1603 ret=3
  TxPDO mapping object index 1 = 1a03 ret=6
  SOEM IOMap size: 46
  
  Slave:1
   Name:MADHT1105B01
   Output size: 168bits
   Input size: 200bits
   State: 8
   Delay: 0[ns]
   Has DC: 1
   DCParentport:0
   Activeports:1.0.0.0
   Configured address: 1001
  
  Finished configuration successfully
  End program

.. API Documents
.. =============

.. .. toctree::
..    :maxdepth: 2

..    api_ethercat_manager
..    api_minas_control

Maintainer Tips
===============

Create DEB file
---------------

Following command will build DEB (binary installer file for Ubuntu with which you can install software by a simple run of `gdebi` command) files.

Before start please add following line to your `/etc/ros/rosdep/sources.list.d/20-default.list` file
.. code-blcok:: bash

  yaml file:///etc/ros/rosdep/ethercat_manager.yaml

and create `ethercat_manager.yaml` file that contains

.. code-block:: bash

  ethercat_manager:
    ubuntu:
      apt: ros-indigo-ethercat-manager

.. code-block:: bash

  catkin b ethercat_manager --no-deps --make-args debbuild
  dpkg -i ros-indigo-ethercat-managerl_0.0.1-0trusty_amd64.deb
  catkin b minas_control --nodeps --make-args debbuild

To install DEB file from command line, please use `gdebi`. Using `apt-get` may fail due to missing dependent deb package, and that breaks your local apt database (wich may fixed by `sudo apt-get install -f install` as reported on the `community site <http://askubuntu.com/questions/58202/how-to-automatically-fetch-missing-dependencies-when-installing-software-from-d>`_)

.. code-block:: bash

  sudo apt-get install gdebi
  gdebi -n ros-indigo-minas-control_0.0.1-0trusty_amd64.deb

Create documents
----------------

Following command will build pdf manual.

.. code-block:: bash

  catkin b minas_control --no-deps --make-args docbuild

To build the manual you have to install following deb packages

.. code-block:: bash

  apt-get install python-bloom sphinx-common python-catkin-shpinx pdflatex \
                  texlive-latex-base  texlive-latex-recommended texlive-lang-cjk

Known Issues
------------

Trouble shooting
----------------

- If you could not initialize ethercat driver as follows,

  .. code-block:: bash

    $ reset eth4
    SOEM (Simple Open EtherCAT Master)
    Simple test
    Initializing etherCAT master
    Could not initialize ethercat driver
    terminate called after throwing an instance of 'ethercat::EtherCatError'
      what():  Could not initialize SOEM
    Aborted (Core dump)

  Please check if your binary have correctly set permissions by

  .. code-block:: bash

    $ getcap /opt/ros/indigo/lib/minas_control/reset
    /opt/ros/indigo/lib/minas_control/reset = cap_net_raw+ep

  If you can any `capability`, please try

  .. code-block:: bash

    $ sudo setcap cap_net_raw+ep /opt/ros/indigo/lib/minas_control/reset


.. _MINAS-A5B:  https://industrial.panasonic.com/ww/products/motors-compressors/fa-motors/ac-servo-motors/minas-a5b

.. _ROS: http://ros.org/

.. _PANATERM: https://industrial.panasonic.com/jp/products/motors-compressors/fa-motors/ac-servo-motors/minas-a5-panaterm

.. _Manual: https://industrial.panasonic.com/content/data/MT/PDF/refer/jp/acs/SX-DSV02469_R4_00J.pdf
