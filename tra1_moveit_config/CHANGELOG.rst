^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tra1_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.9 (2018-03-02)
------------------

1.0.8 (2017-11-28)
------------------
* add run_depend on moveit_fake_controller_manager
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
* Add end effector to tra1_moveit_config
* Contributors: Ryosuke Tajima

1.0.2 (2017-05-12)
------------------

1.0.1 (2017-05-12)
------------------

1.0.0 (2017-04-22)
------------------
* add test codes  (`#39 <https://github.com/tork-a/minas/pull/39`_)
  * tra1_moveit_config/CMakeLists.txt: check only woking launch files
  * tra1_moveit_config/package.xml: add more run_depends
* set default planner to RRTConnectkConfigDefault (`#37 <https://github.com/tork-a/minas/pull/37`_)
* Contributors: Tokyo Opensource Robotics Developer 534

0.5.2 (2017-04-01)
------------------

0.5.1 (2017-03-26)
------------------

0.5.0 (2017-03-06)
------------------

0.4.0 (2017-03-06)
------------------
* add CHANGELOG for tra1\_ packages
* update CMakeLists.txt : add Relaease
* Contributors: Tokyo Opensource Robotics Developer 534

0.3.0 (2017-03-06)
------------------
* config/tra1_controllers.yaml: add controller_manager_ns: controller_manager
* package.xml add moveit_simple_controller_manager
* config/joint_limits.yaml: enable joint_velocity_limit
* remove world link from urdf (and others)
* add config/tra1_controllers.yaml and tra1_moveit_controller_manager.launch.xml
* Add README.md
* Add tra1_moveit_config package
* Contributors: Ryosuke Tajima, Tokyo Opensource Robotics Developer 534
