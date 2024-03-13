Pilz Industrial Motion Planner
==============================

``pilz_industrial_motion_planner`` 는 궤적 생성기로서 MoveIt과 함께 표준 로봇 모션(점대점, 선형, 원형)을
계획합니다.

관련된 planning pipeline( ``*_moveit_config`` 패키지에 있는
``pilz_industrial_motion_planner_planning_planner.yaml`` 파일)을 로딩함으로써, 궤적 생성 기능은 
``move_group`` node가 제공하는 사용자 인터페이스(C++, Python 또는 RViz)를 통해 접근할 수 있습니다.
예를 들면, ``/plan_kinematic_path`` service 와 ``/move_action`` action 등이 있습니다.
자세한 사용법 안내는 :doc:`/doc/examples/moveit_cpp/moveitcpp_tutorial` 및
:doc:`/doc/examples/move_group_interface/move_group_interface_tutorial` 문서를 참조하십시오.

조인트 제한(Joint Limits)
----------------------------

Pilz 플래닝 파이프라인을 실행하는 ROS node의 파라미터에서
플래너는 최대 속도와 가속도를 사용합니다.
MoveIt Setup Assistant를 사용하여
``joint_limits.yaml`` 파일이 적절한 기본값으로 자동 생성되고 시작할때 로드됩니다.

지정된 제한값이 URDF robot description의 제한값보다 우선하게 됩니다.
위치 제한과 속도 제한을 설정은 URDF와 파라미터 파일 모두에서 가능하지만,
가속도 제한은 파라미터 파일만 사용하여 설정할 수 있다는 점에 유의하십시오.
일반적인 ``has_acceleration`` 및 ``max_acceleration`` 파라미터에 외에도
``has_deceleration`` 및 ``max_deceleration``\ (<0.0) 을 설정하는 기능도 추가했습니다.

제한값은 node 파라미터의 제한값이 URDF에서 설정된 파라미터보다 엄격하거나
적어도 같아야 한다는 전제 하에 병합됩니다.

현재 계산된 궤적은 모든 조인트에 대해서 공통된 제한값으로서 모든 제한값의 가장 엄격한
결합을 사용하여 이런 제한들을 준수합니다.

데카르트 제한(Cartesian Limits)
----------------------------------

카테시안 궤적 생성 (LIN/CIRC)의 경우, planner는 3D 데카르트 공간에서의 최대 속도에 대한 정보가 필요합니다. 즉, 평행 이동/회전 속도/가속/감속을 다음과 같이 node 파라미터에 설정해야 합니다.:

.. code:: yaml

    cartesian_limits:
      max_trans_vel: 1
      max_trans_acc: 2.25
      max_trans_dec: -5
      max_rot_vel: 1.57

``*_moveit_config`` 패키지 내에 ``pilz_cartesian_limits.yaml`` 라는 파일에서 데카르트 속도 및 가속 제한을 지정할 수 있습니다.

planners는 평행 이동 및 회전 사다리꼴 모양에 대해 동일한 가속 비율을 가정합니다.
회전 가속도는 다음과 같이 계산됩니다.
``max_trans_acc / max_trans_vel * max_rot_vel`` (감속도도 마찬가지로)

Planning Interface
------------------

이 패키지는 motion planning의 입력 및 출력으로 ``moveit_msgs::msgs::MotionPlanRequest`` 와 ``moveit_msgs::msg::MotionPlanResponse`` 를 사용합니다.
각 계획 알고리즘에 필요한 파라미터는 다음과 같이 설명됩니다.

``MotionPlanRequest`` 를 채우는 방법에 대한 일반적인 소개는
:ref:`move_group_interface-planning-to-pose-goal` 을 참조하십시오.

``MotionPlanRequest`` 의 "planner_id" 로 "PTP", "LIN" 또는 "CIRC" 를 지정할 수 있습니다.

The PTP motion command
----------------------

이 planner는 사다리꼴 조인트 속도 프로파일을 가진 완전히 동기화된 point-to-point 궤적을 생성합니다. 모든 조인트는 동일한 최대 조인트 속도/가속도/감속 제한을 갖는다고 가정합니다. 그렇지 않은 경우 가장 엄격한 한계가 적용됩니다. 목표에 도달하는 데 가장 오랜 시간이 걸리는 축이 리드 축(lead axis)으로 선택됩니다. 다른 축은 리드 축과 동일한 가속/등속/감속 단계를 공유하도록 감속됩니다.

.. image:: ptp.png
   :alt: PTP velocity profile with trapezoidal ramps - the axis with the longest duration
         determines the maximum velocity

``moveit_msgs::MotionPlanRequest`` 에서 PTP Input 파라미터들
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- ``planner_id``: ``"PTP(점대점)"``
- ``group_name``: planning group의 이름
- ``max_velocity_scaling_factor``: 최대 조인트 속도 스케일링 인자
- ``max_acceleration_scaling_factor``: 최대 조인트 가속/감속 스케일링 인자
- ``start_state/joint_state/(name, position and velocity)``: 시작 상태의 조인트 이름/위치/속도 (선택사항)
- ``goal_constraints``: (목표는 조인트 공간 또는 데카라트 공간에서 지정될 수 있음)
- 조인트 공간에서 목표
    - ``goal_constraints/joint_constraints/joint_name``: 목표 조인트 이름
    - ``goal_constraints/joint_constraints/position``: 목표 조인트 위치
- 데카르트 공간에서 목표
    - ``goal_constraints/position_constraints/header/frame_id``: 이 데이터가 연결된 프레임
    - ``goal_constraints/position_constraints/link_name``: 타겟 링크 이름
    - ``goal_constraints/position_constraints/constraint_region``: 타겟 지점의 경계 볼륨(bounding volume)
    - ``goal_constraints/position_constraints/target_point_offset``: (선택사항) 타겟 링크에서 목표 지점에 대한 offset(링크 프레임 기준)


``moveit_msg::MotionPlanResponse`` 에서 PTP Planning 결과
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-  ``trajectory_start``: 계획된 궤적의 첫 번째 로봇 상태
-  ``trajectory/joint_trajectory/joint_names``: 생성된 조인트 궤적의 조인트 이름 목록
-  ``trajectory/joint_trajectory/points/(positions,velocities,accelerations,time_from_start)``:
   생성된 경로 지점 목록. 각 지점은 모든 조인트들의 위치/속도/가속도를 가집니다.(조인트 이름과 같은 순서) 마지막 포인트는 0 속도 및 가속도를 가진다.
-  ``group_name``: planning group의 이름
-  ``error_code/val``: 모션 계획 오류 코드

The LIN motion command
----------------------

이 플래너는 목표 포즈와 시작 포즈 사이의 선형 데카르트 경로를 생성합니다. 데카르트 제한 사항을 사용하여 데카르트 공간에서 사다리꼴 속도 프로파일을 생성합니다. 병진 운동(translational motion)은 시작 위치 벡터와 목표 위치 벡터 사이의 선형 보간입니다. 회전 운동은 시작과 목표 방향 사이의 쿼터니언 slerp(spherical linear interpolation)입니다. 병진 운동과 회전 운동은 시간적으로 동기화됩니다. 이 플래너는 속도가 0인 시작 상태만 받아들입니다. 계획 결과는 조인트 궤적입니다. 조인트 공간 제한 위반으로 인해 모션 계획이 실패하는 경우 사용자는 데카르트 속도/가속도 스케일링 요소를 조정해야 합니다.

``moveit_msgs::MotionPlanRequest`` 에서 LIN Input 파라미터들
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-  ``planner_id``: ``"LIN"``
-  ``group_name``: planning group 이름
-  ``max_velocity_scaling_factor``: 최대 데카르트 병진운동/회전 속도의 스케일링 인자
-  ``max_acceleration_scaling_factor``: 최대 데카르트 병진/회전 가속도/감속도의 스케일링 인자
-  ``start_state/joint_state/(name, position and velocity``: 시작 상태의 조인트 이름/위치
-  ``goal_constraints`` (목표는 조인트 공간 또는 데카라트 공간에서 지정될 수 있음)

   -  조인트 공간에서 목표

      -  ``goal_constraints/joint_constraints/joint_name``: 목표 조인트 이름
      -  ``goal_constraints/joint_constraints/position``: 목표 조인트 위치

   -  데카르트 공간에서 목표

      -  ``goal_constraints/position_constraints/header/frame_id``:
         이 데이터가 연결된 프레임
      -  ``goal_constraints/position_constraints/link_name``: 타겟 링크 이름
      -  ``goal_constraints/position_constraints/constraint_region``:
         타겟 지점의 경계 볼륨
      -  ``goal_constraints/position_constraints/target_point_offset``:
         (선택사항) 타겟 링크에서 목표 지점에 대한 offset(링크 프레임 기준)

``moveit_msg::MotionPlanResponse`` 에서 LIN Planning 결과
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-  ``trajectory_start``: first robot state of the planned trajectory
-  ``trajectory/joint_trajectory/joint_names``: a list of the joint
   names of the generated joint trajectory
-  ``trajectory/joint_trajectory/points/(positions,velocities,accelerations,time_from_start)``:
   a list of generated way points. Each point has
   positions/velocities/accelerations of all joints (same order as the
   joint names) and time from start. The last point will have zero
   velocity and acceleration.
-  ``group_name``: the name of the planning group
-  ``error_code/val``: error code of the motion planning

The CIRC motion command
-----------------------

This planner generates a circular arc trajectory in Cartesian space
between goal and start poses. There are two options for giving a path
constraint:

- the *center* point of the circle: The planner always
  generates the shorter arc between start and goal and cannot generate a
  half circle,
- an *interim* point on the arc: The generated trajectory
  always goes through the interim point. The planner cannot generate a
  full circle.

The Cartesian limits, namely translational/rotational
velocity/acceleration/deceleration need to be set and the planner uses
these limits to generate a trapezoidal velocity profile in Cartesian
space. The rotational motion is quaternion slerp between start and goal
orientation. The translational and rotational motion is synchronized in
time. This planner only accepts start state with zero velocity. The planning
result is a joint trajectory. The user needs to adapt the Cartesian
velocity/acceleration scaling factor if motion plan fails due to
violation of joint limits.

CIRC Input Parameters in ``moveit_msgs::MotionPlanRequest``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-  ``planner_id``: ``"CIRC"``
-  ``group_name``: the name of the planning group
-  ``max_velocity_scaling_factor``: scaling factor of maximal Cartesian
   translational/rotational velocity
-  ``max_acceleration_scaling_factor``: scaling factor of maximal
   Cartesian translational/rotational acceleration/deceleration
-  ``start_state/joint_state/(name, position and velocity``: joint
   name/position of the start state.
-  ``goal_constraints`` (goal can be given in joint space or Cartesian
   space)

   -  for a goal in joint space

      -  ``goal_constraints/joint_constraints/joint_name``: goal joint
         name
      -  ``goal_constraints/joint_constraints/position``: goal joint
         position

   -  for a goal in Cartesian space

      -  ``goal_constraints/position_constraints/header/frame_id``:
         frame this data is associated with
      -  ``goal_constraints/position_constraints/link_name``: target
         link name
      -  ``goal_constraints/position_constraints/constraint_region``:
         bounding volume of the target point
      -  ``goal_constraints/position_constraints/target_point_offset``:
         offset (in the link frame) for the target point on the target
         link (optional)

-  ``path_constraints`` (position of the interim/center point)

   -  ``path_constraints/name``: interim or center
   -  ``path_constraints/position_constraints/constraint_region/primitive_poses/point``:
      position of the point


CIRC Planning Result in ``moveit_msg::MotionPlanResponse``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-  ``trajectory_start``: first robot state of the planned trajectory
-  ``trajectory/joint_trajectory/joint_names``: a list of the joint
   names of the generated joint trajectory
-  ``trajectory/joint_trajectory/points/(positions,velocities,accelerations,time_from_start)``:
   a list of generated way points. Each point has
   positions/velocities/accelerations of all joints (same order as the
   joint names) and time from start. The last point will have zero
   velocity and acceleration.
-  ``group_name``: the name of the planning group
-  ``error_code/val``: error code of the motion planning

Examples
--------

By running

::

    ros2 launch moveit_resources_panda_moveit_config demo.launch.py

you can interact with the planner through the RViz MotionPlanning panel.

.. figure:: rviz_planner.png
   :alt: rviz figure

To use the planner through the MoveGroup Interface, refer to
:codedir:`the MoveGroup Interface C++ example <how_to_guides/pilz_industrial_motion_planner/src/pilz_move_group.cpp>`.
To run this, execute the following commands in separate Terminals:

::

    ros2 launch moveit_resources_panda_moveit_config demo.launch.py
    ros2 run moveit2_tutorials pilz_move_group


To use the planner using MoveIt Task Constructor, refer to
:codedir:`the MoveIt Task Constructor C++ example <how_to_guides/pilz_industrial_motion_planner/src/pilz_mtc.cpp>`.
To run this, execute the following commands in separate Terminals:

::

    ros2 launch moveit2_tutorials mtc_demo.launch.py
    ros2 launch moveit2_tutorials pilz_mtc.launch.py

Using the planner
-----------------

The *pilz_industrial_motion_planner::CommandPlanner* is provided as a MoveIt Motion Planning
Pipeline and, therefore, can be used with all other manipulators using
MoveIt. Loading the plugin requires the param
``/move_group/<pipeline_name>/planning_plugins`` to be set to ``[pilz_industrial_motion_planner/CommandPlanner]``
before the ``move_group`` node is started.
For example, the `panda_moveit_config package
<https://github.com/ros-planning/moveit_resources/blob/ros2/panda_moveit_config>`_
has a ``pilz_industrial_motion_planner`` pipeline set up as follows:


::

    ros2 param get /move_group pilz_industrial_motion_planner.planning_plugins

    String value is: pilz_industrial_motion_planner/CommandPlanner


To use the command planner, Cartesian limits have to be defined. The
limits are expected to be under the namespace
``<robot_description>_planning``, where ``<robot_description>`` refers
to the parameter name under which the URDF is loaded.
For example, if the URDF was loaded into ``/robot_description`` the
Cartesian limits have to be defined at ``/robot_description_planning``.

You can set these using a ``pilz_cartesian_limits.yaml`` file in your
``*_moveit_config`` package.
An example showing this file can be found in `panda_moveit_config
<https://github.com/ros-planning/moveit_resources/blob/ros2/panda_moveit_config/config/pilz_cartesian_limits.yaml>`_.

To verify the limits were set correctly, you can check the parameters for your
``move_group`` node. For example,

::

    ros2 param list /move_group --filter .*cartesian_limits

    /move_group:
        robot_description_planning.cartesian_limits.max_rot_vel
        robot_description_planning.cartesian_limits.max_trans_acc
        robot_description_planning.cartesian_limits.max_trans_dec
        robot_description_planning.cartesian_limits.max_trans_vel


Sequence of multiple segments
-----------------------------

To concatenate multiple trajectories and plan the trajectory at once,
you can use the sequence capability. This reduces the planning overhead
and allows to follow a pre-desribed path without stopping at
intermediate points.

**Please note:** In case the planning of a command in a sequence fails,
non of the commands in the sequence are executed.

**Please note:** Sequences commands are allowed to contain commands for
multiple groups (e.g. "Manipulator", "Gripper")

User interface sequence capability
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A specialized MoveIt functionality known as the
:moveit_codedir:`command list manager<moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/command_list_manager.h>`
takes a ``moveit_msgs::msg::MotionSequenceRequest`` as input.
The request contains a list of subsequent goals as described above and an additional
``blend_radius`` parameter. If the given ``blend_radius`` in meter is
greater than zero, the corresponding trajectory is merged together with
the following goal such that the robot does not stop at the current
goal. When the TCP comes closer to the goal than the given
``blend_radius``, it is allowed to travel towards the next goal already.
When leaving a sphere around the current goal, the robot returns onto
the trajectory it would have taken without blending.

.. figure:: blend_radius.png
   :alt: blend figure

Implementation details are available
:moveit_codedir:`as PDF<moveit_planners/pilz_industrial_motion_planner/doc/MotionBlendAlgorithmDescription.pdf>`.

Restrictions for ``MotionSequenceRequest``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-  Only the first goal may have a start state. Following trajectories
   start at the previous goal.
-  Two subsequent ``blend_radius`` spheres must not overlap.
   ``blend_radius``\ (i) + ``blend_radius``\ (i+1) has to be smaller
   than the distance between the goals.

Action interface
~~~~~~~~~~~~~~~~

In analogy to the ``MoveGroup`` action interface, the user can plan and
execute a ``moveit_msgs::MotionSequenceRequest`` through the action server
at ``/sequence_move_group``.

In one point the ``MoveGroupSequenceAction`` differs from the standard
MoveGroup capability: If the robot is already at the goal position, the
path is still executed. The underlying PlannerManager can check, if the
constraints of an individual ``moveit_msgs::msg::MotionPlanRequest`` are
already satisfied but the ``MoveGroupSequenceAction`` capability doesn't
implement such a check to allow moving on a circular or comparable path.

See the ``pilz_robot_programming`` package for a `ROS 1 Python script
<https://github.com/PilzDE/pilz_industrial_motion/blob/melodic-devel/pilz_robot_programming/examples/demo_program.py>`_
that shows how to use the capability.

Service interface
~~~~~~~~~~~~~~~~~

The service ``plan_sequence_path`` allows the user to generate a joint
trajectory for a ``moveit_msgs::msg::MotionSequenceRequest``.
The trajectory is returned and not executed.
