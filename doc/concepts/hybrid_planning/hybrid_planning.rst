===============
Hybrid Planning
===============

MoveIt의 motion planning 아키텍처는 "인식-계획-행동(Sense-Plan-Act)" 접근 방식을 따릅니다. 즉, 로봇의 동작을 계획하고 실행하기 위해서는 먼저 환경과 로봇의 상태를 "인식(Sense)"하고, 그 다음 플래너("계획"(Plan))를 통해 로봇의 경로를 계산한 후, 마지막으로 트래젝토리 컨트롤러를 사용하여 단일 실행으로 "행동(Act)"하게 합니다.

하지만 이러한 솔루션은 정적이고 잘 알려진 환경에서의 global 모션 계획에는 효과적이지만, 불안정하거나 동적인 환경에서의 많은 real-world 응용 프로그램에는 적용하기 어렵습니다. 예를 들어, 사람에게 물 한잔을 제공하거나 요철난 칠판에 글씨를 쓰는 작업과 같은 경우 예기치 못한 변화에 반응할 수 있게 보다 정교한 방법이 필요합니다.
즉, 로봇의 환경이 동적으로 변화하거나, 작업 자체에 불확실성이 있을 수 있습니다. 예를 들어, 칠판에 글씨를 쓸 때는 칠판 위에 가하는 압력을 조절해야 하며, 사용함에 따라 분필도 점점 짧아집니다.

이러한 과제를 해결하기 위해서는 실행 중인 모션을 현재 조건에 맞게 조정하거나 예상치 못한 환경 변화가 발생했을 때 재계획을 통해 반응할 수 있는 방법이 필요합니다. 하이브리드 플래닝 아키텍처는 이러한 문제를 해결하기 위해 반복적인 전역 플래너와 로컬 플래너를 쌍으로 결합하여 시도합니다.

What is Hybrid Planning?
------------------------

하이브리드 계획(Hybrid Planning)은 다른 종류의 모션 플래너를 결합하여 보다 강건하고 즉각적인 해결책을 도출하는 모션 계획 방법을 일컫는 용어입니다. 이 일반적인 접근 방식은 이미 네비게이션(Navigation) 분야에서 널리 사용되고 있으며 navigation2와 같은 대중적인 프로젝트에서 성공적으로 구현되었습니다.

MoveIt의 하이브리드 계획 아키텍처는 서로 다른 계획 속도와 문제 범위를 가지고 병렬 및 반복적으로 실행되는 글로벌 플래너와 로컬 플래너 쌍을 결합합니다.

글로벌 플래너는 "감지-계획-행동(Sense-Plan-Act)" 응용 프로그램에서 사용되는 플래너와 매우 유사하게 전체적인 모션 계획 문제를 해결하는 임무를 가지고 있습니다. 사용되는 플래너 알고리즘은 완전해야 하며 따라서 계산 시간 측면에서 상대적으로 느리다고 가정됩니다. 또한, 글로벌 플래너는 실시간 안전을 보장하지 않아 특정 기한 내에 해결책을 찾을 수 있다는 보장이 없습니다. 플래너 구현에 따라 글로벌 플래너는 실행 중에 하나의 초기 해결책을 생성하거나 실행되는 동안에 반복적으로 최적화된 해결책들을 생성할 수 있습니다.

로컬 플래너는 실행 중에 지속적으로 동작하고 있으며 글로벌 궤적을 따라가기 위해서 반복적인 로봇 명령을 생성합니다. 어떤 면에서 로컬 플래너는 컨트롤러와 유사하지만, 아키텍처는 더 복잡한 문제와 제약 조건을 해결할 수 있도록 합니다. 이 플래너는 world에 대한 추론이 가능하고 내부 상태를 가질 수 있다는 점에서 매우 다목적이며, 다음과 같은 임의의 로컬 계획 문제 조합을 해결하는 데 사용할 수 있습니다.:

* 후속(subsequent) 글로벌 참조(reference) 궤적의 풀림, 혼합(unwinding, blending) 또는 연결
* 글로벌 경로를 따라가면서 동적으로 근접 충돌 방지
* 글로벌 궤적을 로컬 제약 조건에 적응 (예: 거친 표면에서 원하는 힘 압력, 시각 피드백을 기반으로 도구 재조정)
* 로컬 궤적 최적화 및 시간 매개 변수화(time parameterization) (로컬 환경에서 궤적을 최적화하는 것이 계산 비용이 저렴하고 빠름)

이러한 로컬 문제를 해결하기 위해 로컬 플래너는 빠르고 센서 피드백에 반응할 수 있어야 하며 많은 경우 실시간성을 가져야 합니다. 또한 갑작스럽거나 예측할 수 없는 동작을 피하기 위해 결정론적이어야 합니다.

일반적으로 로컬 플래너는 로컬 최소값에 갇히지 않도록 글로벌 플래너가 생성한 기준 궤적(reference trajectory)을 사용합니다. 로컬 최소값을 여전히 완전히 배제할 수 없는 경우 원하는 목표에 도달하기 위해 글로벌 플래너를 다시 계획하도록 트리거할 수 있습니다. 이런 동작은 planner events를 통신해서 처리하는 특별한 방식이 필요합니다. 이런 목적으로 하이브리드 계획 아키텍처는 특정 use case와 planner type에 따라서 사용자가 정의할 수 있는 이벤트 기반 로직을 구현할 수 있도록 허용합니다.

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


The Hybrid Planning Architecture
--------------------------------

아래 다이어그램은 하이브리드 플래닝 아키텍처를 구성하는 기본 플러그인 유형과 ROS 인터페이스를 보여줍니다.

.. image:: hybrid_planning_architecture.png
   :width: 700px
   :align: center

아키텍처는 세 가지 ROS 컴포넌트 node로 구성됩니다.

* **Hybrid Planning Manager** (하이브리드 플래닝 관리자)
  * 하이브리드 플래닝 요청에 대해서 ROS action을 제공합니다.
  * 플래닝 로직을 실행하고 플래너를 조정합니다.
* **Global Planner** (전역 플래너)
  * 전역 플래닝 문제를 해결하고 솔루션 궤적을 publish합니다.
* **Local Planner** (로컬 플래너)
  * 수신된 전역 궤적 업데이트를 처리합니다.
  * 로봇 상태, world 및 참조 궤적을 기반으로 로컬 플래닝 문제를 해결합니다.
  * 로봇 드라이버에게 위치/속도 명령을 보냅니다.


아키텍처 구성 요소는 일반적이고 사용자 정의가 가능하도록 설계되었습니다. component는 ROS 2 message interface를 통해서만 상호 작용하기 때문에 아키텍처의 각 component 또는 플러그인 구현을 쉽게 교체할 수 있습니다. 플러그인 인터페이스는 최소화되도록 설계되었으며 실제 알고리즘 구현에서 가능한 한 추상화됩니다. 따라서 개발자는 인프라의 다른 부분을 구현하지 않고 격리된 로직이나 솔버에만 완전히 집중할 수 있습니다. 또한 동일한 component를 다른 설정이나 플래닝 문제에 재사용할 수도 있습니다.


Hybrid Planning Manager
^^^^^^^^^^^^^^^^^^^^^^^

.. image:: hybrid_planner_manager_small.png
   :width: 400px
   :align: center

This component is “The Brain” of the architecture. Its main purpose is to process HybridPlanner action  requests and to coordinate the motion planning and execution process based on the planning logic plugin. The planning logic is implemented in the PlanningLogic plugin and is event-driven by design. Events are defined by string identifiers and may trigger action calls or cancellations targeting the global or local planners. An example event log for a simple planning logic is shown in the diagram below:

.. image:: hybrid_planning_event_logic.png
   :width: 400px
   :align: center

Events are triggered by the Hybrid Planning action request and by both of the global and local planners action feedback messages. In this example, the Hybrid Planning Manager starts the global planner after the hybrid planning request is received. Upon arrival of the global trajectory the local planner is started and when the local planner is finished the Hybrid Planning Manager returns a Hybrid Planning response. 

A custom implementation of the Planning Logic plugin supports mapping generic events to available actions provided by the architecture like “Start global planning”, “Stop trajectory execution”, or “Switch to local planner constraint x”. With this, the motion planning behavior becomes highly customizable and adaptable.


Global Planner
^^^^^^^^^^^^^^

.. image:: global_planner_small.png
   :width: 500px
   :align: center

The Global Planner is the simplest component of the architecture. It provides an action server which processes GlobalPlanner requests which include the common MotionPlanRequests which are processed by the Global Planner plugin. By default, this is simply MoveIt’s planning pipeline, but any kind of planner or even MTC could technically be used here. The planning result is reported using the action feedback and the solution trajectory is published to the Local Planner for further processing.


Local Planner
^^^^^^^^^^^^^

The Local Planner also runs an action server that handles requests from the Hybrid Planning Manager. The action is used for starting and stopping execution and may also configure runtime parameters like constraints or solver types.

.. image:: local_planner_small.png
   :width: 500px
   :align: center

The local planner implementation is based on two plugins:

* **Trajectory Operator**: This plugin maintains the global reference trajectory, handles trajectory updates from the global planner, and monitors the process of the current robot state.
* **Local Constraint Solver**: This plugin implements the iterative solver algorithm that produces the robot commands based on the reference trajectory and the local constraints. It may include additional interfaces for dynamically processing sensor input or event updates.

The diagram below shows an example loop cycle of the Local Planner upon action request by the Hybrid Planning Manager:

.. image:: local_planner_loop.png
   :width: 700px
   :align: center

Each iteration the local planner requests the current planning scene and matches the current robot state within the reference trajectory. If the goal is reached, the local planning action successfully finishes. Otherwise, the current local planning problem is identified based on the current robot state and solved afterwards. Finally, the resulting control commands are published to the robot controller.


How does it work?
^^^^^^^^^^^^^^^^^

The runtime behavior of a Hybrid Planner can best be understood by drawing a workflow diagram that visualizes the communication channels and events of the different components.

Below is a diagram that shows the runtime logic of a successful trajectory execution.

.. image:: hybrid_planner_logic.png
   :width: 700px
   :align: center

The planner is invoked by a hybrid planning request which is also the first event the Hybrid Planning Manager reacts to.
In this example, the planner logic simply runs both planners in sequence. After the initial hybrid planning request, the Hybrid Planning Manager invokes the global planner.
The global planner computes and publishes a trajectory which is received by the Hybrid Planning Manager and the Local Planner Component.
Important to notice is, that the Local Planner Component just processes the new trajectory and does not start executing until it is invoked by the Hybrid Planning Manager. Once requested by the Hybrid Planning Manager, the Local Planner Component starts unwinding the reference trajectory and returns the action response successfully when it reaches the final state. After that, the Hybrid Planning Manager returns a successful HybridPlanningResponse.

Now let's consider a more difficult scenario where the hybrid planner is designed to avoid an obstacle during execution by replanning.
The animation below shows a simple motion that is being fixed at runtime because of changing collision objects.

.. image:: replanning_example.gif
   :width: 500px
   :align: center


Here, the collision object present during the global planning process disappears after the global trajectory is computed. Instead two new collision objects appear that invalidate the initial global trajectory. The local planner detects the imminent collision and pauses the execution until the global planner has provided an updated collision free trajectory.

Below you can see the workflow of the described behavior.

.. image:: hybrid_planner_logic_2.png
   :width: 700px
   :align: center

The startup is the same as in the first example, but during unwinding the reference trajectory the local planner detects a collision. Here, the planner logic reacts by re-invoking the global planner. During the calculation of the new global solution, the local planner must prevent the robot from colliding with the collision object i.e. by keeping its current position. After the global planner finishes its calculation, the new global solution is published to the local planner and the local planner’s Trajectory Operator Plugin blends the update into the reference trajectory. Afterwards, the Local Planner Component continues to follow the reference trajectory as the updated solution enables it to steer around the collision object.

If you want to use Hybrid Planning in your application or just want to experiment with it, check out the :doc:`Hybrid Planning Example Tutorial </doc/examples/hybrid_planning/hybrid_planning_tutorial>`.
