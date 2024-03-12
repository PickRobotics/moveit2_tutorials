===============
Hybrid Planning
===============

MoveIt의 motion planning 아키텍처는 "인식-계획-행동(Sense-Plan-Act)" 접근 방식을 따릅니다. 즉, 로봇의 동작을 계획하고 실행하기 위해서는 먼저 환경과 로봇의 상태를 "인식(Sense)"하고, 그 다음 플래너("계획"(Plan))를 통해 로봇의 경로를 계산한 후, 마지막으로 트래젝토리 컨트롤러를 사용하여 단일 실행으로 "행동(Act)"하게 합니다.

하지만 이러한 솔루션은 정적이고 잘 알려진 환경에서의 global 모션 계획에는 효과적이지만, 불안정하거나 동적인 환경에서의 많은 real-world 응용 프로그램에는 적용하기 어렵습니다. 예를 들어, 사람에게 물 한잔을 제공하거나 요철난 칠판에 글씨를 쓰는 작업과 같은 경우 예기치 못한 변화에 반응할 수 있게 보다 정교한 방법이 필요합니다. 즉, 로봇의 환경이 동적으로 변화하거나, 작업 자체에 불확실성이 있을 수 있습니다. 예를 들어, 칠판에 글씨를 쓸 때는 칠판 위에 가하는 압력을 조절해야 하며, 사용함에 따라 분필도 점점 짧아집니다.


이러한 과제를 해결하기 위해서는 실행 중인 모션을 현재 조건에 맞게 조정하거나 예상치 못한 환경 변화가 발생했을 때 재계획을 통해 반응할 수 있는 방법이 필요합니다. 하이브리드 플래닝 아키텍처는 이러한 문제를 해결하기 위해 반복적인 전역 플래너와 로컬 플래너를 쌍으로 결합하여 시도합니다.

Hybrid Planning이란?
------------------------

하이브리드 계획(Hybrid Planning)은 다른 종류의 모션 플래너를 결합하여 보다 강건하고 즉각적인 해결책을 도출하는 모션 계획 방법을 일컫는 용어입니다. 이 일반적인 접근 방식은 이미 네비게이션(Navigation) 분야에서 널리 사용되고 있으며 navigation2와 같은 대중적인 프로젝트에서 성공적으로 구현되었습니다.

MoveIt의 하이브리드 계획 아키텍처는 서로 다른 계획 속도와 문제 범위를 가지고 병렬 및 반복적으로 실행되는 global planners와 local planners 쌍을 결합합니다.

글로벌 플래너는 "감지-계획-행동(Sense-Plan-Act)" 응용 프로그램에서 사용되는 플래너와 매우 유사하게 전체적인 모션 계획 문제를 해결하는 임무를 가지고 있습니다. 사용되는 플래너 알고리즘은 완전해야 하며 따라서 계산 시간 측면에서 상대적으로 느리다고 가정됩니다. 또한, 글로벌 플래너는 실시간 안전을 보장하지 않아 특정 기한 내에 해결책을 찾을 수 있다는 보장이 없습니다. 플래너 구현에 따라 글로벌 플래너는 하나의 초기 해결책을 생성하거나 실행되는 동안에 반복적으로 최적화된 해결책들을 생성할 수 있습니다.

로컬 플래너는 실행 중에 지속적으로 동작하고 있으며 글로벌 궤적을 따라가기 위해서 반복적인 로봇 명령을 생성합니다. 어떤 면에서 로컬 플래너는 컨트롤러와 유사하지만, 아키텍처는 더 복잡한 문제와 제약 조건을 해결할 수 있도록 합니다. 이 플래너는 world에 대한 추론이 가능하고 내부 상태를 가질 수 있다는 점에서 매우 다목적이며, 다음과 같은 임의의 로컬 계획 문제 조합을 해결하는 데 사용할 수 있습니다.:

* 후속(subsequent) 글로벌 참조(reference) 궤적의 풀림(unwinding), 혼합(blending) 또는 연결
* 글로벌 경로를 따라가면서 동적으로 근접 충돌 방지
* 글로벌 궤적을 로컬 제약 조건에 적응 (예: 거친 표면에서 원하는 힘 압력, 시각 피드백을 기반으로 도구 재조정)
* 로컬 궤적 최적화 및 시간 매개 변수화(time parameterization) (로컬 환경에서 궤적을 최적화하는 것이 계산 비용이 저렴하고 빠름)

이러한 로컬 문제를 해결하기 위해 로컬 플래너는 빠르고, 센서 피드백에 반응할 수 있어야 하며 많은 경우 실시간성을 가져야 합니다. 또한 갑작스럽거나 예측할 수 없는 동작을 피하기 위해 결정론적이어야 합니다.

일반적으로 로컬 플래너는 로컬 최소값(local minimum)에 갇히지 않도록 글로벌 플래너가 생성한 기준 궤적(reference trajectory)을 사용합니다. 로컬 최소값을 여전히 완전히 배제할 수 없는 경우 원하는 목표에 도달하기 위해 글로벌 플래너를 다시 계획하도록 트리거할 수 있습니다. 이런 동작은 planner events를 통신해서 처리하는 특별한 방식이 필요합니다. 이런 목적으로 하이브리드 계획 아키텍처는 특정 use case와 planner type에 따라서 사용자가 정의할 수 있는 이벤트 기반 로직(event-based logic)을 구현할 수 있도록 허용합니다.

+-------------------------------------------+-------------------------------------------+
| Global Planner                            | Local Planner                             |
+===========================================+===========================================+
| * Solve global solution trajectory        | * Follow global reference trajectory      |
| * Optimize trajectory path (continuously) | * Solve local problem constraints         |
|                                           | * May process sensor input                |
|                                           | * Optimize solution locally               |
|                                           | * Compute controller commands             |
+-------------------------------------------+-------------------------------------------+
| * Complete                                | * Can get stuck in local minima           |
| * No restricted  computation time         | * Low computation time                    |
| * Not real-time safe                      | * Realtime-safe (depends on solver)       |
| * Not necessarily deterministic           | * Deterministic                           |
+-------------------------------------------+-------------------------------------------+
| * OMPL planner                            | * IK solver, Jacobian                     |
| * STOMP                                   | * Potential field planner                 |
| * TrajOpt                                 | * Trajectory optimization algorithm       |
| * Cartesian motion planner                | * Model Predictive Control (MPC)          |
| * Pilz Industrial Motion Planner          | * Sensor-based Optimal Control            |
| * MTC                                     |                                           |
+-------------------------------------------+-------------------------------------------+

하이브리드 플래닝은 광범위한 use case에서 유용합니다. 대부분의 응용 프로그램은 다음 세 가지 시나리오로 그룹화할 수 있습니다.

* *Online motion planning*(온라인 모션 플래닝): 전역 플래너는 초기 전역 솔루션을 생성하고 지속적으로 최적화합니다. 동시에 로컬 플래너는 참조 궤적을 실행하고 업데이트된 궤적 세그먼트를 참조 궤적에 혼합합니다.
* *Reactive Motion*(반응형 모션): 전역 플래너는 무효화된 솔루션(재플래닝)을 수정하는 데 사용된다.(로컬 플래너는 충돌 전에 속도를 줄이거나 정지)
* *Adaptive Motion*(적응형 모션): 로컬 플래너는 동적으로 변하는 조건에 맞게 전역 솔루션을 조정하는 데 사용됩니다. (불균등한 표면을 일정한 도구 접촉으로 유지하는 것과 같은)


Hybrid Planning 아키텍쳐
--------------------------------

아래 다이어그램은 하이브리드 플래닝 아키텍처를 구성하는 기본 플러그인 유형과 ROS 인터페이스를 보여줍니다.

.. image:: hybrid_planning_architecture.png
   :width: 700px
   :align: center

아키텍처는 세 가지 ROS 컴포넌트 node로 구성됩니다.:

* **Hybrid Planning Manager(하이브리드 플래닝 관리자)**

  * 하이브리드 플래닝 요청에 대해서 ROS action을 제공합니다.
  * 플래닝 로직을 실행하고 플래너를 조정합니다.

* **Global Planner(글로벌 플래너)**

  * 전역 플래닝 문제를 해결하고 솔루션 궤적을 publish합니다.

* **Local Planner(로컬 플래너)**

  * 수신된 전역 궤적 업데이트를 처리합니다.
  * 로봇 상태, world 및 참조 궤적을 기반으로 로컬 플래닝 문제를 해결합니다.
  * 로봇 드라이버에게 위치/속도 명령을 보냅니다.


아키텍처 구성 요소는 일반적이고 사용자 정의가 가능하도록 설계되었습니다. component는 ROS 2 message interface를 통해서만 상호 작용하기 때문에 아키텍처의 각 component 또는 플러그인 구현을 쉽게 교체할 수 있습니다. 플러그인 인터페이스는 최소화되도록 설계되었으며 실제 알고리즘 구현에서 가능한 한 추상화됩니다. 따라서 개발자는 인프라의 다른 부분을 구현하지 않고 격리된 로직이나 솔버에만 완전히 집중할 수 있습니다. 또한 동일한 component를 다른 설정이나 플래닝 문제에 재사용할 수도 있습니다.


Hybrid Planning Manager
^^^^^^^^^^^^^^^^^^^^^^^

.. image:: hybrid_planner_manager_small.png
   :width: 400px
   :align: center

이 component는 아키텍처의 "두뇌" 역할을 합니다. 주요 목적은 하이브리드 플래너  action request을 처리하고, 플래닝 로직 플러그인을 기반으로 모션 플래닝 및 실행 프로세스를 조정하는 것입니다. 플래닝 로직은 PlanningLogic 플러그인에 구현되어 있으며 이벤트 기반으로 설계되었습니다. 이벤트는 문자열 식별자로 정의되며 전역 또는 로컬 플래너를 대상으로 하는 작업 호출 또는 취소를 트리거할 수 있습니다. 간단한 플래닝 로직에 대한 예시 이벤트 로그가 아래 다이어그램에 표시됩니다.:

.. image:: hybrid_planning_event_logic.png
   :width: 400px
   :align: center

이벤트는 트리거는 하이브리드 플래닝 acstion request와 전역 및 로컬 플래너 액션 피드백 메시지에 의해 동작한다. 이 예에서 Hybrid Planning Manager는 hybrid planning request을 받은 후 전역 플래너를 시작합니다. 전역 궤적이 도착하면 로컬 플래너가 시작되고 로컬 플래너가 완료되면 하이브리드 플래너 관리자가 Hybrid Planning response을 반환합니다. 

Planning Logic 플러그인의 커스텀 구현은 “Start global planning(전역 플래닝 시작)”, “Stop trajectory execution(궤적 실행 중지)”, or “Switch to local planner constraint x(로컬 플래너 제약 x로 전환)”과 같이 아키텍처에서 제공하는 사용 가능한 actions에 일반 이벤트를 매핑하는 것을 지원합니다. 이를 통해 motion planning behavior은 매우 쉽게 사용자 정의 및 적응이 가능합니다.


Global Planner
^^^^^^^^^^^^^^

.. image:: global_planner_small.png
   :width: 500px
   :align: center

글로벌 플래너는 전체 아키텍처에서 가장 간단한 component입니다. action server를 제공하여 GlobalPlanner 요청을 처리합니다. 이 요청에는 Global Planner 플러그인에서 처리하는 일반적인 MotionPlanRequest가 포함됩니다. 기본적으로 이는 MoveIt의 간단한 planning pipeline이지만 기술적으로는 모든 종류의 플래너 또는 심지어 MTC도 여기에서 사용할 수 있습니다. 플래닝 결과는 action feedback을 사용하여 보고되고, 솔루션 궤적은 Local Planner에게 publish되어 추가적인 처리를 수행합니다.


Local Planner
^^^^^^^^^^^^^

Local Planner는 Hybrid Planning Manager로부터의 요청을 처리하는 action server도 실행합니다. 이 action은 실행 시작 및 종료를 위해 사용되며 제약 조건 또는 솔버 유형과 같은 런타임 파라미터를 설정할 수도 있습니다.

.. image:: local_planner_small.png
   :width: 500px
   :align: center

로컬 플래너 구현은 다음 두 가지 플러그인을 기반으로 합니다.:

* **궤적 연산자(Trajectory Operator)**: 이 플러그인은 글로벌 참조 궤적을 유지 관리하고, 글로벌 플래너로부터 궤적 업데이트를 처리하며, 현재 로봇 상태의 프로세스를 모니터링합니다.
* **로컬 제약 솔버(Local Constraint Solver)**: 이 플러그인은 반복 솔버 알고리즘을 구현하는데, 기준 궤적과 로컬 제약 조건에 따라 로봇 명령을 생성하는 알고리즘이다. 센서 입력 또는 이벤트 업데이트를 동적으로 처리하기 위한 추가적인 인터페이스를 포함할 수 있습니다.

아래 다이어그램은 Hybrid Planning Manager의 액션 요청시에 로컬 플래너의 예제 loop cycle을 보여줍니다.:

.. image:: local_planner_loop.png
   :width: 700px
   :align: center

각 반복마다 로컬 플래너는 현재의 planning scene을 요청하며, 참조 궤적 내에서 현재 로봇 상태를 일치시킵니다. 목표에 도달하면 로컬 플래닝 action이 성공적으로 완료됩니다. 그렇지 않으면 현재 로봇 상태를 기반으로 현재 로컬 플래닝 문제를 식별하고 나서 해결합니다. 마지막으로 최종 제어 명령이 로봇 컨트롤러에게 publish됩니다.


어떻게 동작하는 것일까?
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Hybrid Planner의 런타임 동작을 이해하기 위해서는 다양한 구성 요소 간의 커뮤니케이션 채널과 이벤트를 시각화하는 워크플로우 다이어그램을 그리는 것이 가장 좋습니다.

아래 다이어그램은 성공적인 궤적 실행의 런타임 로직을 보여줍니다.

.. image:: hybrid_planner_logic.png
   :width: 700px
   :align: center

플래너는 hybrid planning request에 의해 호출되며, 이 요청은 Hybrid Planning Manager가 반응하는 첫 번째 이벤트이기도 합니다.
이 예시에서 플래너 로직은 단순히 양쪽 플래너를 순차적으로 실행합니다. 초기 하이브리드 플래닝 요청 후에 Hybrid Planning Manager가 global planner를 호출합니다.
글로벌 플래너는 궤적를 계산하고 publish하며, 이 궤적은 하이브리드 플래닝 관리자와 Local Planner Component가 수신합니다.
중요한 것은 Local Planner Component는 새로운 궤적을 처리만 하며, 하이브리드 플래닝 관리자에 의해 호출될 때까지 실행을 시작하지 않는다는 점입니다. 일단 하이브리드 플래너 관리자가 요청하면 로컬 플래너 구성 요소는 참조 경로(reference trajectory)를 풀기(unwinding) 시작하고 최종 상태에 도달하면 성공적으로 action response을 반환합니다. 그 후 하이브리드 플래닝 관리자는 성공적인 HybridPlanningResponse를 반환합니다.

이제 좀 더 어려운 시나리오를 살펴보겠습니다. 여기서는 하이브리드 플래너가 실행 중 장애물을 피하기 위해 재계획을 수행하도록 설계되었습니다.
아래 애니메이션은 충돌 물체가 변경됨에 따라 런타임에 수정되는 간단한 모션을 보여줍니다.

.. image:: replanning_example.gif
   :width: 500px
   :align: center


여기서는 글로벌 계획 프로세스 중에 존재하던 충돌 물체는 글로벌 궤적을 계산하고 나면  사라집니다. 대신 초기 글로벌 경로를 무효화하는 2개의 새로운 충돌 물체가 나타납니다. 로컬 플래너는 임박한 충돌을 감지하고 글로벌 플래너가 업데이트된 충돌 없는 경로를 제공할 때까지 실행을 일시 중지합니다.

아래에서 설명한 behavior의 워크플로를 볼 수 있습니다.

.. image:: hybrid_planner_logic_2.png
   :width: 700px
   :align: center

시작은 첫 번째 예시와 동일하지만, 참조 궤적을 풀어내는(unwinding) 과정에서 로컬 플래너가 충돌을 감지합니다. 여기에서 플래너 로직은 글로벌 플래너를 다시 호출하여 반응합니다. 새로운 글로벌 해결책을 계산하는 동안, 로컬 플래너는 로봇이 충돌 객체와 충돌하지 않도록 현재 위치를 유지해야 합니다. 즉, 로컬 플래너는 로봇의 현재 위치를 유지해야 합니다. 글로벌 플래너가 계산을 완료하면 새로운 글로벌 솔루션이 로컬 플래너에게 전달되고, 로컬 플래너의 Trajectory Operator Plugin은 업데이트를 참조 궤적에 혼합합니다. 그 후, Local Planner Component는 업데이트된 솔루션을 통해 충돌 객체를 피해 조종할 수 있으므로 참조 궤적을 계속 따릅니다.

Hybrid Planning을 애플리케이션에서 사용하거나 실험하고 싶다면, :doc:`Hybrid Planning Example Tutorial </doc/examples/hybrid_planning/hybrid_planning_tutorial>` 을 확인하세요.
