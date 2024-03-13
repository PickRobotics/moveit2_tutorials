STOMP Motion Planner
====================

.. image:: stomp.gif
   :width: 700px

Stochastic Trajectory Optimization for Motion Planning (STOMP)STOMP은 확률적 최적화 프레임워크입니다.(`Kalakrishnan et al. 2011 <https://www.researchgate.net/publication/221078155_STOMP_Stochastic_trajectory_optimization_for_motion_planning>`_).
적당한 계획 시간 내에 부드럽고 안정적이며 충돌 없는 경로를 생성합니다.
이 알고리즘은 초기 (실현 가능하지 않을 수도 있는) 기준 궤적 주변 공간을 탐색하기 위해 무작위로 노이즈가 추가된 궤적을 생성하여 더 낮은 비용의 새로운 결합 궤적을 생성하는 방식을 사용합니다.
궤적 비용은 문제-특정(problem-specific) 비용 함수를 통해 계산됩니다. 이 비용 함수는 충돌, waypoint 제약 조건 위반, 부드러움 및 제어 속성에 대한 waypoint 비용 페널티를 계산합니다.
최적화 프로세스는 반복적으로 실행되므로 기준 궤적이 점프없이 지속적으로 개선됩니다.

STOMP는 최적화 알고리즘의 기울기 정보를 필요로 하지 않기 때문에 미분 계산을 지원하지 않는 비용 함수(예: 제약 조건 및 모터 토크에 해당하는 비용)도 포함할 수 있습니다.
STOMP의 주요 장점은 토크 제한, 에너지 및 툴 제약 조건과 같은 추가적인 목적 함수를 통합할 수 있다는 것입니다.
현재 MoveIt의 planning 플러그인 API를 통해 커스텀 비용 함수를 전달하는 기능을 검토하고 있습니다.

시작하기
---------------
:doc:`Getting Started </doc/tutorials/getting_started/getting_started>` 을 먼저 완료합니다.

You should also have gone through the steps in :doc:`Visualization with MoveIt RViz Plugin </doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial>`
 문서도 해봐야 합니다.

사전준비
-------------
 1. ROS 2 배포판을 위한 MoveIt ``main`` 의 최근 빌드입니다. 지원되는 ROS 2 배포판의 경우 STOMP 라이브러리는 별도의 ROS 패키지로 제공되며 ``rosdep`` 를 통해 설치해야 합니다.
 2. 여러분의 로봇에서 STOMP 를 사용하려면 먼저 해당 로봇을 위한 MoveIt 기능 설정 패키지를 사용하는 것이 최선입니다. 테스트 목적으로는 이 튜토리얼에서 설명된 것처럼 `ros-planning/panda_moveit_config <https://github.com/ros-planning/panda_moveit_config>`_ 에서 제공하는 Panda 로봇을 사용할 수도 있습니다.

여러분의 로봇에서 STOMP 사용하기
----------------------------------
**Note:** 만약 `ros-planning/panda_moveit_config <https://github.com/ros-planning/panda_moveit_config>`_ 리포지토리의 ``panda_moveit_config`` 를 사용하여 이 데모를 따라하고 있다면, 다음 단계는 이미 완료되어 있으며 RViz 에서 STOMP 테스트를 위해 데모를 직접 실행할 수 있습니다.

#. 간단히 `stomp_planning.yaml <https://github.com/ros-planning/moveit_resources/blob/ros2/panda_moveit_config/config/stomp_planning.yaml>`__ 설정 파일을 MoveIt 설정 패키지의 config 디렉토리에 추가하면 됩니다. 이 파일에는 플러그인 식별자, 계획 파이프라인 어댑터 목록 및 STOMP 계획 파라미터를 포함하고 있습니다. 설정 파일은 다음과 같은 예제와 유사해야 합니다.: ::

    planning_plugins:
      - stomp_moveit/StompPlanner
    request_adapters:
      - default_planning_request_adapters/ResolveConstraintFrames
      - default_planning_request_adapters/ValidateWorkspaceBounds
      - default_planning_request_adapters/CheckStartStateBounds
      - default_planning_request_adapters/CheckStartStateCollision
    response_adapters:
      - default_planning_response_adapters/AddTimeOptimalParameterization
      - default_planning_response_adapters/ValidateSolution
      - default_planning_response_adapters/DisplayMotionPath

    stomp_moveit:
      num_timesteps: 60
      num_iterations: 40
      num_iterations_after_valid: 0
      num_rollouts: 30
      max_rollouts: 30
      exponentiated_cost_sensitivity: 0.5
      control_cost_weight: 0.1
      delta_t: 0.1

#. MoveIt에서 STOMP planning pipeline을 로드하려면, 기존에 사용하던 "ompl"과 다른 planners 옆에 있는 MoveItConfiguration launch 구문에 "stomp" 문자열을 추가해야 합니다. 설정 예제는 Panda 설정의 `demo.launch.py <https://github.com/ros-planning/moveit_resources/blob/ros2/panda_moveit_config/launch/demo.launch.py#L42>`_ 내에서 찾아 볼 수 있습니다.

데모 실행하기
----------------
만약  `ros-planning/moveit_resources <https://github.com/ros-planning/moveit_resources>`_ 리포지터리에 있는 ``panda_moveit_config`` 패키지를 가지고 있다면, 간단하게 데모 설정을 launch시키고 RVIZ에서 STOMP으로 플래닝을 시작시킬 수 있습니다. ::

  ros2 launch moveit_resources_panda_moveit_config demo.launch.py

STOMP 파라미터들
----------------
STOMP의 파라미터는 `stomp_planning.yaml <https://github.com/ros-planning/moveit_resources/blob/ros2/panda_moveit_config/config/stomp_planning.yaml>`__ 통해 설정할 수 있습니다. 모든 파라미터는 `stomp_moveit.yaml <https://github.com/ros-planning/moveit2/blob/main/moveit_planners/stomp/res/stomp_moveit.yaml>`_ 에 정의되어 있으며, 이 파일은 `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ 에 입력으로 사용됩니다. 해당 파일에는 기본값과 허용 범위와 같은 추가 정보를 찾아볼 수 있습니다.:

**최적화 파라미터들(Optimization Parameters)**:

- *num_timesteps*: 궤도에 사용되는 timesteps의 수 - 이 값은 궤도 초기화, 계획 기간 및 솔루션 출력을 고려한 waypoint 개수로 바로 변환된다.

- *num_iterations*: 최적화 과정에서 planner가 좋은 솔루션을 찾는데 걸리는 총 반복 횟수입니다.

- *num_iterations_after_valid*: 이미 유효한 솔루션을 찾은 후 planner가 계속 최적화를 반복하는 횟수입니다.

- *num_rollouts*: 반복마다 생성되는 노이즈가 포함된 궤도의 수입니다.

- *max_rollouts*: 반복에서 최적화를 위해 고려되는 새 롤아웃과 이전 롤아웃의 총 최대 수입니다.

- *exponentiated_cost_sensitivity:* 확률 계산에 사용되는 지수 비용을 조정하는 인수입니다. 값을 높이면 STOMP가 더 빠르게 최적화되고 수렴하지만 강건성이 떨어지고 반복가능한 결과가 낮아지는 단점이 있습니다.

- *control_cost_weight*:  총 비용 계산에 적용되는 제어 비용 인수입니다. STOMP는 가정한 동일한 waypoint 타임스텝에 필요한 제곱 가속도 합을 최소화하여 결과 궤도를 부드럽게 만들려고 시도합니다.

- *delta_t*: 연속적인 지점 사이의 가정하는 시간 변화입니다.

- *path_marker_topic*: RViZ가 선택적 경로 시각화를 위해 구독하는 topic 이름입니다. 설정하지 않으면 경로가 시각화되지 않습니다.

제공된 기본 파라미터는 대부분의 환경에서 STOMP가 양호하게 작동하도록 합니다. 하지만 더 복잡한 환경에서 STOMP이 잘 동작하게 하기 위해서는 타임스텝 수 또는 롤아웃 수를 늘릴 수 있습니다. 이렇게 하려면 STOMP가 종료하는데 허용되는 planning 시간을 늘려야 할 수도 있습니다.


STOMP, CHOMP, OMPL로 얻어진 계획들 사이의 차이점
-----------------------------------------------------------

이 섹션에서는 STOMP, CHOMP, OMPL로부터 얻은 경로 간의 차이점을 살펴보겠습니다.
일부 MoveIt 플래너들은  jerky(경직적인) 궤적을 생성하고 불필요한 로봇 움직임을 일으킬 수 있습니다.
따라서 보통은 후처리 매끄럽게 만드는 (smoothing) 단계가 필요합니다.
반면에 STOMP는 짧은 시간 안에 부드럽고 안정적인 모션 플랜을 생성하는 경향이 있어 다른 모션 플래너와 달리 후처리 단계가 필요하지 않을 수도 있습니다.

CHOMP는 공분산(covariant) 및 함수 기울기(functional gradient) 접근 방식을 기반으로 주어진 초기 단순 궤적을 최적화시키는 최적화 플래너입니다.

OMPL은 랜덤 샘플링과 그래프 탐색에 주로 의존하는 샘플링 기반 모션 플래닝 알고리즘(sampling-based motion planning algorithms)용 오픈 소스 라이브러리입니다.
샘플링 기반 알고리즘은 확률적으로 완전합니다. : 즉, 솔루션이 존재한다면 결국 찾을 수 있지만, 솔루션이 존재하지 않는다는 것을 보고할 수는 없습니다.
이 알고리즘들은 효율적이며 보통 빠르게 솔루션을 찾습니다.

아래는 서로 다른 접근 방식 비교를 통한 플래너 품질에 대한 짧은 개요입니다.:

- **Local Minima Handling**: STOMP는 확률적 특성으로 인해 국부 최소값을 피할 수 있습니다. 하지만 CHOMP는 국부 최소값에 쉽게 빠지고 종종 최적 해를 찾지 못할 수 있습니다. STOMP과 CHOMP 논문에 따르면, 대부분의 경우 STOMP가 더 우수합니다.

- **Planning Time**: STOMP와 CHOMP의 계획 시간은 비슷하지만, CHOMP는 성공하기 위해 더 많은 반복 작업을 필요로 합니다. 이는 주로 STOMP의 각 반복은 궤적 비용 평가를 여러 번 수행해야 하지만 CHOMP의 기울기 업데이트 규칙보다 더 안정적인 방식으로 더 큰 단계를 밟을 수 있기 때문입니다. OMPL 알고리즘 (적어도 솔루션을 최적화하지 않는 알고리즘)은 일반적으로 더 빠르며, 경로 길이 또는 부드러움과 같은 품질을 잠시 저하시키더라도 어려운 계획 시나리오에서도 비교적 적은 시간 안에 솔루션을 찾을 수 있습니다.

- **Parameter Tuning**: CHOMP는 일반적으로 성공적인 솔루션을 얻기 위해 STOMP보다 더 많은 파리미터 튜닝이 필요합니다.
  OMPL은 일반적으로 많은 파라미터 튜닝이 필요하지 않는다.; 기본 파라미터로도 대부분의 상황에서 잘 작동합니다.

- **Obstacle Handling**: 장애물이 있는 scenes에서 STOMP는 종종 확률적 특성으로 인해 장애물을 성공적으로 피할 수 있습니다.
  하지만 CHOMP는 로봇의 동적 변수 (가속도, 속도 등)에 대한 비용 함수에서 일부 잡음(*ridge_factor*)을 추가하여 부드러운 궤적이 아닌 경로를 생성합니다. OMPL은 또한 장애물이 존재하는 경우 충돌 없는 부드러운 경로를 생성합니다.
