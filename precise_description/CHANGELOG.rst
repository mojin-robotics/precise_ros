^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package precise_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2021-05-10)
------------------
* Merge pull request `#23 <https://github.com/mojin-robotics/precise_ros/issues/23>`_ from fmessmer/fix/material_name_clash
  explicit material definition
* explicit material definition
* Merge pull request `#16 <https://github.com/mojin-robotics/precise_ros/issues/16>`_ from benmaidel/feature/mrk_gripper
  move mrk gripper to external xacro
* move gripper description to external xacro
* move gripper mesh z coord to origin
* Merge pull request `#13 <https://github.com/mojin-robotics/precise_ros/issues/13>`_ from fmessmer/kevin_ipa
  Updates from testing at IPA
* apply changes to PF400_short as well
* testing at IPA
* Merge pull request `#10 <https://github.com/mojin-robotics/precise_ros/issues/10>`_ from fmessmer/test_noetic
  test noetic
* Bump CMake version to avoid CMP0048 warning
* Merge pull request `#8 <https://github.com/mojin-robotics/precise_ros/issues/8>`_ from benmaidel/feature/gripper_control
  Add gripper control
* adapt arm_5_joint (gripper) limits to real hardware
* fix gazebo_ros_control
* fix real hw joint limits
* harmonize transmission macro
* fix mesh origin
* use measurments from kinematic table
* rename meshes
* remove V01 suffix
* add new meshes
* remove old meshes
* rename robots
* Merge pull request `#6 <https://github.com/mojin-robotics/precise_ros/issues/6>`_ from fmessmer/testing
  Testing on HW
* additional urdf properties for pf400
* add PF400 urdf, config
* Merge pull request `#5 <https://github.com/mojin-robotics/precise_ros/issues/5>`_ from fmessmer/feature/precise_description_moveit
  precise_description - moveit
* add basic meshes for mrk
* configurable gripper_ee_fixed_link
* configurable finger meshes
* gripper_ee_fixed_joint collision free translation
* adjust gripper_ee_fixed_link origin
* add gripper_ee_fixed_link
* Merge pull request `#2 <https://github.com/mojin-robotics/precise_ros/issues/2>`_ from fmessmer/feature/precise_description
  [WIP] precise_description
* add default_inertia
* rescale meshes
* use limits from specs
* configure joint limits via properties yaml
* configure joint origins via properties yaml
* rename joints
* rename robot
* fix roslaunch checks
* add roslaunch checks
* use Apache 2.0 license
* add authors and maintainers
* provide launch files for pf3400sx
* provide xacro for pf3400sx
* track meshes with lfs
* add initial description package
* Contributors: Benjamin Maidel, Felix Messmer, fmessmer, mojin@cob4-20, tsl
