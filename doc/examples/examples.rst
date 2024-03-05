Examples
========

이 부분은 아직 새로운 구조에 맞게 업데이트되지 않았거나 아직 ROS 1에서부터 포팅되지 않은 페이지들을 위한 임시 보관소입니다.

이 페이지들을 새로운 구조로 옮기는 방법에 대해서는 :doc:`/doc/how_to_contribute/how_to_contribute` 아래 문서를 참고하세요.

MoveGroup - ROS Wrappers in C++
------------------------------------------
스크립팅을 통해 MoveIt을 사용하는 가장 간단한 방법은 ``move_group_interface``를 사용하는 것입니다. 이 인터페이스는 초보자에게 이상적이며 MoveIt의 다양한 기능에 대한 통합된 접근을 제공합니다.

.. toctree::
   :maxdepth: 1

   move_group_interface/move_group_interface_tutorial

Using MoveIt Directly Through the C++ API
------------------------------------------
MoveIt으로 더 복잡한 응용 프로그램을 만들려면 개발자가 MoveIt의 C++ API를 파고드는 것이 필요합니다. 추가적인 장점으로 C++ API를 직접 사용하면 많은 ROS Service/Action 계층을 건너뛰어 결과적으로 성능이 크게 향상됩니다.

.. toctree::
   :maxdepth: 1

   robot_model_and_robot_state/robot_model_and_robot_state_tutorial
   planning_scene/planning_scene_tutorial
   planning_scene_monitor/planning_scene_monitor_tutorial
   planning_scene_ros_api/planning_scene_ros_api_tutorial
   motion_planning_api/motion_planning_api_tutorial
   motion_planning_pipeline/motion_planning_pipeline_tutorial
   creating_moveit_plugins/plugin_tutorial
   visualizing_collisions/visualizing_collisions_tutorial
   time_parameterization/time_parameterization_tutorial
   planning_with_approximated_constraint_manifolds/planning_with_approximated_constraint_manifolds_tutorial
   pick_place/pick_place_tutorial
   moveit_grasps/moveit_grasps_tutorial
   moveit_task_constructor/moveit_task_constructor_tutorial
   moveit_deep_grasps/moveit_deep_grasps_tutorial
   subframes/subframes_tutorial
   moveit_cpp/moveitcpp_tutorial
   bullet_collision_checker/bullet_collision_checker
   mobile_base_arm/mobile_base_arm_tutorial

Integration with a New Robot
----------------------------
MoveIt 2에 새로운 로봇을 통합하려고 시도하기 전에 해당 로봇이 이미 설정되어 있는지 확인하십시오 (`MoveIt을 실행하는 robots 목록 <http://moveit.ros.org/robots/>`_). 그렇지 않은 경우 이 섹션의 안내 절차를 따라 로봇을 MoveIt에 통합하십시오.

.. toctree::
   :maxdepth: 1

   setup_assistant/setup_assistant_tutorial
   urdf_srdf/urdf_srdf_tutorial
   controller_configuration/controller_configuration_tutorial
   perception_pipeline/perception_pipeline_tutorial
   hand_eye_calibration/hand_eye_calibration_tutorial
   ikfast/ikfast_tutorial
   trac_ik/trac_ik_tutorial

Configuration
-------------
.. toctree::
   :maxdepth: 1

   kinematics_configuration/kinematics_configuration_tutorial
   custom_constraint_samplers/custom_constraint_samplers_tutorial
   ompl_interface/ompl_interface_tutorial
   stomp_planner/stomp_planner_tutorial
   trajopt_planner/trajopt_planner_tutorial
   pilz_industrial_motion_planner/pilz_industrial_motion_planner
   planning_adapters/planning_adapters_tutorial.rst
   persistent_scenes_and_states/persistent_scenes_and_states

Miscellaneous
----------------------------

.. toctree::
   :maxdepth: 1

   realtime_servo/realtime_servo_tutorial
   hybrid_planning/hybrid_planning_tutorial
   benchmarking/benchmarking_tutorial
   tests/tests_tutorial
