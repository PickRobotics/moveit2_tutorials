===============
Motion Planning
===============

The Motion Planning Plugin
---------------------------

MoveIt은 **plugin interface**를 통해 motion planners와 함께 작동합니다. 이를 통해 MoveIt은 여러 라이브러리의 다양한 모션 플래너와 통신하고 사용할 수 있게 하며, 이를 통해 MoveIt을 쉽게 확장할 수 있습니다. 모션 플래너와의 인터페이스는 ROS action 또는 service(``move_group`` node 제공)를 통해 이루어집니다.
move_group의 기본 모션 플래너는 OMPL 및 MoveIt Setup Assistant를 통해 MoveIt 인터페이스와 함께 OMPL을 사용하여 구성됩니다. 기본적으로 사용할 수 있는 다른 플래너로는 Pilz 산업용 모션 플래너와 CHOMP가 있습니다.

The Motion Plan Request
------------------------

모션 계획 요청(motion plan request)은 motion planner가 수행하길 원하는 작업을 지정합니다. 일반적으로 모션 플래너에게 로봇 팔을 다른 위치(joint space내에)로 이동하거나, end-effector를 새로운 pose로 이동하도록 요청합니다.
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

완전한 모션 계획 파이프라인(motion planning pipeline)은 motion planner를 **planning request adapters**라고 불리는 다른 구성 요소와 연결합니다.
Planning request adapters는 모션 계획 요청을 사전 처리하고 모션 계획 응답을 사후 처리할 수 있도록 합니다.
사전 처리는 여러 상황에서 유용한데 예를 들면 로봇의 시작 상태가 로봇에 지정된 joint limits을 약간 벗어나 있는 경우가 있습니다.
사후 처리에는 다른 여러 작업에서 필요로하는데, 예를 들면 로봇을 위해 생성된 path를 시간 매개 변수 궤적(time-parameterized trajectories)으로 변환하는 경우입니다.
MoveIt은 각각 매우 특정한 기능을 수행하는 일련의 기본 motion planning adapters를 제공합니다.

FixStartStateBounds
^^^^^^^^^^^^^^^^^^^

시작 상태 경계 수정 어댑터(fix start state bounds adapter)는 URDF에 지정된 joint limits 내에 있도록 시작 상태를 수정합니다.
이 어댑터가 필요한 경우는 물리적 로봇의 joint limits가 올바르게 구성되지 않은 경우입니다.
이런 경우 로봇은 하나 이상의 관절이 joint limits를 약간 벗어나는 구성에 놓일 수 있습니다.
이 경우 모션 플래너는 시작 상태가 joint limits를 벗어나 있다고 생각하기 때문에 계획을 수행할 수 없습니다.
"FixStartStateBounds" 계획 요청 어댑터는 joint limits로 이동하여 시작 상태를 "fix"합니다.
그러나 이는 항상 올바른 해결책이 될수는 없습니다. 예를 들어 joint가 실제로 관절 한계를 훨씬 벗어난 경우입니다.
어댑터에 대한 파라미터로 "fixable(수정 가능한)" 한계를 얼마나 벗어날 수 있는지를 지정합니다.

FixWorkspaceBounds
^^^^^^^^^^^^^^^^^^

Fix Workspace Bounds 어댑터는 로봇의 작업 공간을 정의합니다. 만약 사용자가 로봇의 작업 공간을 지정하지 않으면, 이 어댑터는 기본적으로 가로 10 미터, 세로 10 미터, 높이 10 미터 크기의 큐브 형태의 작업 공간을 설정합니다.

FixStartStateCollision
^^^^^^^^^^^^^^^^^^^^^^

Fix Start State Collision adapter는 충돌하는 시작 위치를 해결합니다. 이 어댑터는 로봇의 joint 값을 약간씩 움직여 충돌하지 않는 새로운 위치를 찾습니다. 움직이는 양은 **jiggle_fraction** 파라미터로 제어되며, 이 파라미터는 각 joint의 전체 motion 범위의 백분율을 나타냅니다. 이 아답터와 관련된 또 다른 파라미터는 랜덤으로 샘플링하는 시도 횟수를 지정하며, 이 횟수를 넘으면 실패합니다.

FixStartStatePathConstraints
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Fix Start State Path Constraints 어댑터는 모션 계획의 시작 위치가 지정한 path 제약 조건을 만족하지 않을 때 사용됩니다. 이 어댑터는 현재 위치에서 경로 제약 조건을 만족하는 새로운 위치까지의 path를 계획하려고 시도합니다. 이 새로운 위치는 이후 모션 플래닝의 시작 위치가 됩니다.

AddTimeParameterization
^^^^^^^^^^^^^^^^^^^^^^^

motion planners는 일반적으로 속도나 가속도 제약 조건을 따르지 않고 시간 매개변수(time parameterized)가 없는 "kinematic paths"를 생성합니다. Time Parameterization 어댑터는 이러한 경로에 속도 및 가속도 제약 조건을 적용하여 "time parameterize(시간 매개변수화)"합니다.

ResolveConstraintFrames
^^^^^^^^^^^^^^^^^^^^^^^

목표 제약(Goal constraints) 조건은 subframes을 사용하여 설정할 수 있습니다 (예: ``cup/handle`` frame에서 pose goal, 여기서 ``handle``은 객체 ``cup`` 상의 subframe입니다.).
이 어댑터는 제약 조건의 프레임을 객체 또는 로봇 프레임 (예: ``cup``)으로 변경합니다.

OMPL
----

OMPL (Open Motion Planning Library)은 주로 랜덤 모션 플래너를 구현하는 오픈소스 모션 플래닝 라이브러리( motion planning library)입니다.
MoveIt은 OMPL과 직접 통합되며, 기본 플래너 라이브러리로 사용합니다.
OMPL의 플래너는 추상적입니다. 예로 OMPL은 로봇의 개념이 없습니다.
대신 MoveIt은 OMPL을 설정하고 로봇공학에서 문제 해결을 위해서 OMPL이 작동할 수 있도록 백엔드를 제공합니다.
