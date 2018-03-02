^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tra1_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.9 (2018-03-02)
------------------
* Set home_encoder_offset zero as default
* Onsite fix for encoder offset
* Contributors: Ryosuke Tajima

1.0.8 (2017-11-28)
------------------
* Add parameter home_encoder_offset (`#63 <https://github.com/tork-a/minas/issues/63>`_)
* Contributors: Ryosuke Tajima

1.0.6 (2017-09-12)
------------------

1.0.5 (2017-09-12)
------------------

1.0.4 (2017-09-11)
------------------
* set license as GPLv2/CC-BY-SA,add LICENSE for meshes (`#55 <https://github.com/tork-a/minas/issues/55>`_)
  * set license as GPLv2/CC-BY-SA,add LICENSE for meshes
  * cleanup code
  * fix wrong license data, add LICENSE information to xacro files
  * cleanup package.xml
* Contributors: Ryosuke Tajima, Tokyo Opensource Robotics Developer 534

1.0.3 (2017-08-04)
------------------
* Fix instruction
* Contributors: 7675t

1.0.2 (2017-05-12)
------------------

1.0.1 (2017-05-12)
------------------
* fix comments on params
* On site fix 5/11 by Tajima
* add joint_trajectory_controller to run_depend
* Contributors: Ryosuke Tajima, Tokyo Opensource Robotics Developer 534

1.0.0 (2017-04-22)
------------------
* add test codes  (`#39 <https://github.com/tork-a/minas/pull/39`_)
  * tra1_bringup/package.xml: add joint_state_controller and position_controllers to package.xml
  * tra1_bringup/CMakeList.txt: add test lauch files
  * tra1_bringup/package.xml: add more run_depends
* make parameters as rosparam (`#33 <https://github.com/tork-a/minas/pull/33`_)
  * tra1_bringup.launch/ joint1->joint5 : fix typo
  * move custom controller parameters for tra1 to tra1_bringup/launch/tra1_bringup.lauch
  * minas_control.launch add eth arg to set ether device name
* Add timeout to shutdown controllers (`#35 <https://github.com/tork-a/minas/pull/35`_)
* Contributors: Ryosuke Tajima, Tokyo Opensource Robotics Developer 534

0.5.2 (2017-04-01)
------------------

0.5.1 (2017-03-26)
------------------
* README.md: add instruction to run simulation(loopback) mode
* README.md: add how to observe joint_states
* Contributors: Tokyo Opensource Robotics Developer 534

0.5.0 (2017-03-06)
------------------
* add doc for tra1_brignup (step 5)
* Contributors: Tokyo Opensource Robotics Developer 534

0.4.0 (2017-03-06)
------------------
* add CHANGELOG for tra1\_ packages
* update CMakeLists.txt : add Relaease
* Contributors: Tokyo Opensource Robotics Developer 534

0.3.0 (2017-03-06)
------------------
* add tra1_moveit.laucnh (move for real robot, run after tra1_bringup.launch
* tra1_bringup.launch: add robot_state_publisher and static tf from /world -> base_link
* add tra1_bringup
* Contributors: Tokyo Opensource Robotics Developer 534
