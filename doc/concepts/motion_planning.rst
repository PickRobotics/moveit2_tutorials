===============
Motion Planning
===============

The Motion Planning Plugin
---------------------------

MoveIt은 **plugin interface**를 통해 motion planners와 함께 작동합니다.
이를 통해 MoveIt은 여러 라이브러리의 다양한 모션 플래너와 통신하고 사용할 수 있게 하며, 이를 통해 MoveIt을 쉽게 확장할 수 있습니다. 모션 플래너와의 인터페이스는 ROS action 또는 service(``move_group`` node 제공)를 통해 이루어집니다.
move_group의 기본 모션 플래너는 OMPL 및 MoveIt Setup Assistant를 통해 MoveIt OMPL에 대한 MoveIt 인터페이스을 사용하여 구성됩니다.
기본적으로 사용할 수 있는 다른 플래너로는 Pilz 산업용 모션 플래너와 CHOMP가 있습니다.

The Motion Plan Request
------------------------

모션 계획 요청(motion plan request)은 motion planner가 수행하길 원하는 작업을 지정합니다.
일반적으로 모션 플래너에게 로봇 팔을 다른 위치(joint space내에)로 이동하거나, end-effector를 새로운 pose로 이동하도록 요청합니다.
충돌 감지는 기본적으로 (self-collisions 및 부착된 물체 포함) 수행됩니다.
또한 ``planning_pipeline`` 와 ``planner_id`` 파라미터를 통해 플래너를 지정하고, 모션 플래너가 확인해야 하는 제약 조건을 지정할 수 있습니다. MoveIt에서 제공하는 기본 제약 조건은 ``kinematic constraints``(운동학적 제약 조건)입니다.:

- **Position constraints**(위치 제약 조건): link의 위치를 특정 공간 영역 내에 있도록 제한합니다.

- **Orientation constraints**(방향 제약 조건): link의 방향을 지정된 roll, pitch, yaw 제한 내에 있도록 제한합니다.

- **Visibility constraints**(가시성 제약 조건): link의 한 점을 특정 센서의 가시성 콘(visibility cone) 내에 있도록 제한합니다.

- **Joint constraints**(관절 제약 조건): joint를 두 값 사이에 있도록 제한합니다.

- **User-specified constraints**(사용자 지정 제약 조건): 사용자 정의 callback을 사용하여 직접 제약 조건을 지정할 수도 있습니다.

The Motion Plan Result
--------------------------

**move_group** node는 사용자의 모션 계획 요청(motion plan request)에 따라 원하는 궤적(trajectory)을 생성합니다.
이 궤적은 팔 (또는 모든 joints 그룹)을 원하는 위치로 이동시킵니다.
**move_group**에서 나오는 결과는 단순히 경로(path)가 아니라 궤적임을 유의하십시오.
**move_group**은 최대 속도 및 가속도(지정되어 있다면)를 사용하여 joint 레벨에서 속도 및 가속도 제약 조건을 준수하는 궤적을 생성합니다.

Motion planning adapters
------------------------

.. image:: /_static/images/motion_planner.png

The complete motion planning pipeline chains together a motion planner with other components called **planning request adapters**.
Planning request adapters allow for pre-processing motion plan requests and post-processing motion plan responses.
Pre-processing is useful in several situations, e.g. when a start state for the robot is slightly outside the specified joint limits for the robot.
Post-processing is needed for several other operations, e.g. to convert paths generated for a robot into time-parameterized trajectories.
MoveIt provides a set of default motion planning adapters that each perform a very specific function.

CheckStartStateBounds
^^^^^^^^^^^^^^^^^^^^^

The fix start state bounds adapter fixes the start state to be within the joint limits specified in the URDF.
The need for this adapter arises in situations where the joint limits for the physical robot are not properly configured.
The robot may then end up in a configuration where one or more of its joints is slightly outside its joint limits.
In this case, the motion planner is unable to plan since it will think that the starting state is outside joint limits.
The "CheckStartStateBounds" planning request adapter will "fix" the start state by moving it to the joint limit.
However, this is obviously not the right solution every time - e.g. where the joint is really outside its joint limits by a large amount.
A parameter for the adapter specifies how much the joint can be outside its limits for it to be "fixable".

ValidateWorkspaceBounds
^^^^^^^^^^^^^^^^^^^^^^^

The fix workspace bounds adapter will specify a default workspace for planning: a cube of size 10 m x 10 m x 10 m.
This workspace will only be specified if the planning request to the planner does not have these fields filled in.

CheckStartStateCollision
^^^^^^^^^^^^^^^^^^^^^^^^

The fix start state collision adapter will attempt to sample a new collision-free configuration near a specified configuration (in collision) by perturbing the joint values by a small amount.
The amount that it will perturb the values by is specified by the **jiggle_fraction** parameter that controls the perturbation as a percentage of the total range of motion for the joint.
The other parameter for this adapter specifies how many random perturbations the adapter will sample before giving up.


AddTimeParameterization
^^^^^^^^^^^^^^^^^^^^^^^

The motion planners will typically generate "kinematic paths", i.e., paths that do not obey any velocity or acceleration constraints and are not time parameterized.
This adapter will "time parameterize" the motion plans by applying velocity and acceleration constraints.

ResolveConstraintFrames
^^^^^^^^^^^^^^^^^^^^^^^

Goal constraints can be set using subframes (e.g. a pose goal in the frame ``cup/handle``, where ``handle`` is a subframe on the object ``cup``).
This adapter changes the frame of constraints to an object or robot frame (e.g. ``cup``).

OMPL
----

OMPL (Open Motion Planning Library) is an open-source motion planning library that primarily implements randomized motion planners.
MoveIt integrates directly with OMPL and uses the motion planners from that library as its primary/default set of planners.
The planners in OMPL are abstract; i.e. OMPL has no concept of a robot.
Instead, MoveIt configures OMPL and provides the back-end for OMPL to work with problems in Robotics.
