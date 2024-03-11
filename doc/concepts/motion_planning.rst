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

완전한 모션 계획 파이프라인(motion planning pipeline)은 motion planner를 **planning request adapters** 라고 불리는 다른 구성 요소와 연결합니다.
Planning request adapters는 모션 계획 요청을 사전 처리하고 모션 계획 응답을 사후 처리할 수 있도록 합니다.
사전 처리는 여러 상황에서 유용한데 예를 들면 로봇의 시작 상태가 로봇에 지정된 joint limits을 약간 벗어나 있는 경우가 있습니다.
사후 처리에는 다른 여러 작업에서 필요로하는데, 예를 들면 로봇을 위해 생성된 path를 시간 매개 변수 궤적(time-parameterized trajectories)으로 변환하는 경우입니다.
MoveIt은 각각 매우 특정한 기능을 수행하는 일련의 기본 motion planning adapters를 제공합니다.

CheckStartStateBounds
^^^^^^^^^^^^^^^^^^^^^

fix start state bounds adapter는 URDF 파일에 명시된 joint 한계값 내로 시작 상태를 맞추는 역할을 합니다.
이 어댑터가 필요한 경우는 실제 로봇의 joint 한계가 올바르게 설정되지 않은 경우입니다.
이렇게 되면 로봇의 한개 이상의 joint들이 joint 한계를 약간 벗어나는 구성에 놓이게 될 수 있습니다.
이 경우 모션 플래너는 시작 상태가 joint 한계를 벗어났다고 판단하기 때문에 경로를 계획할 수 없습니다.
"CheckStartStateBounds" planning request adapter는 시작 상태를 joint 한계 위치로 이동시켜 수정합니다.
하지만, 이는 항상 올바른 해결책은 아닙니다. 예를 들어 joint가 실제로 joint 한계를 크게 벗어난 경우에는 말이죠.
어댑터의 매개 변수는 joint가 얼마나 벗어나도 수정 가능한지에 대한 한계를 지정합니다.

ValidateWorkspaceBounds
^^^^^^^^^^^^^^^^^^^^^^^

fix workspace bounds adapter는 로봇의 작업 공간을 기본값으로 설정합니다. 이 기본값은 가로 10 미터, 세로 10 미터, 높이 10 미터 크기의 정육면체 공간입니다.  하지만, 사용자가 계획 요청 시 작업 공간을 설정하지 않은 경우에만 어댑터가 개입하게 됩니다.

CheckStartStateCollision
^^^^^^^^^^^^^^^^^^^^^^^^

fix start state collision adapter는 지정한 관절 설정값(충돌 상태) 근처에서 작은 값만큼 관절 값을 변화시켜 충돌 없는 새로운 관절 설정을 찾으려고 시도합니다.
변화시키는 양은 **jiggle_fraction** 파라미터에 의해 설정되며, 이 파라미터는 관절의 전체 가동 범위에 대한 백분율로 변화량을 제어합니다.
이 어댑터의 다른 파라미터는 샘플링을 포기하기 전에 어댑터가 샘플링할 임의의 변화의 개수를 지정합니다.


AddTimeParameterization
^^^^^^^^^^^^^^^^^^^^^^^

motion planners는 일반적으로 속도나 가속도 제약 조건을 따르지 않고 시간 매개변수(time parameterized)가 없는 "kinematic paths"를 생성합니다.
이 어댑터는 속도 및 가속도 제약 조건을 적용하여 모션 플랜을 "time parameterize(시간 매개변수화)"합니다.

ResolveConstraintFrames
^^^^^^^^^^^^^^^^^^^^^^^

목표 제약(Goal constraints) 조건은 subframes을 사용하여 설정할 수 있습니다 (예: ``cup/handle`` frame에서 pose goal로, 여기서 ``handle`` 은 객체 ``cup`` 상의 subframe입니다.).
이 어댑터는 제약 조건의 프레임을 객체 혹은 로봇 프레임 (예: ``cup`` )으로 변경합니다.

OMPL
----

OMPL (Open Motion Planning Library)은 주로 랜덤 모션 플래너를 구현하는 오픈소스 모션 플래닝 라이브러리( motion planning library)입니다.
MoveIt은 OMPL과 직접 통합되어 있고, 이 라이브러리의 모션 플래너를 기본 플래너로 사용합니다.
OMPL내에 플래너는 추상적입니다. 예로 OMPL은 로봇의 개념이 없습니다.
대신 MoveIt은 OMPL을 설정하고 로봇공학에서 문제 해결을 위해서 OMPL이 작동할 수 있도록 백엔드를 제공합니다.
