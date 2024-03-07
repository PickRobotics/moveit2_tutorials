.. _MoveIt Task Constructor:

#######################
MoveIt Task Constructor
#######################

What is MoveIt Task Constructor?
--------------------------------

| MoveIt Task Constructor (MTC) 프레임워크는 복잡한 planning tasks을 서로 내부적으로 의존하는 subtasks로 분할하는 것을 돕습니다.
| MTC는 MoveIt을 사용하여 subtasks를 해결합니다. subtasks의 정보는 InterfaceState 객체를 통해서 전달됩니다.

.. image:: ./_static/images/mtc_task.png

MTC Stages
-----------
| MTC 단계란 작업 실행 파이프라인에서의 component 또는 단계를 의미합니다.
| stages는 임의의 순서로 배열할 수 있으며, 계층 구조는 개별 stage 유형에 의해서만 제한됩니다.
| stage를 배열할 수 있는 순서는 결과가 전달되는 방향에 따라 제한됩니다.

결과 흐름에 관련해서 3가지 가능한 stages가 있습니다:

* Generators

* Propagators

* Connectors

Generator Stage
^^^^^^^^^^^^^^^
.. image:: ./_static/images/generating_stage.png

| Generator stages는 인접 stages로부터 입력을 받지 않습니다. 이 stage들은 결과를 계산하여 양방향(전방향 및 후방향)으로 전달합니다.
| MTC task 실행은 Generator stages에서 시작됩니다.
| 가장 중요한 generator stage는 ``CurrentState``이며, 현재 로봇의 상태를 계획 파이프라인(planning pipeline)의 시작 지점으로 가져옵니다.

| Monitoring Generator는 다른 stage(인접하지 않은 stage)의 해결책을 모니터링하여 계획에 해당 해결책을 사용하는 stage입니다.
| Monitoring Generator의 예제 - ``GeneratePose``. 이 stage는 일반적으로 ``CurrentState`` 혹은 ``ModifyPlanningScene`` stage를 모니터링합니다. ``CurrentState``의 해결책을 모니터링하여 ``GeneratePose`` stage는 pose를 생성해야 하는 객체 또는 프레임을 찾을 수 있습니다.

| MTC가 제공하는 generator stages에 대한 추가 정보는 여기에서 확인하세요 - :ref:`Generating Stages`

Propagating Stage
^^^^^^^^^^^^^^^^^
.. image:: ./_static/images/propagating_stage.png

| Propagators는 하나의 이웃 상태에서 해결 방법을 받고 문제를 해결한 다음 반대쪽 이웃에게 결과를 전파합니다.
| 구현 방법에 따라 이 stage는 해결 방법을 앞으로, 뒤로 또는 양 방향으로 전달할 수 있습니다.
| propagating stage 예시 - 포즈(pose)에 대한 ``Move Relative``. 이 stage는 일반적으로 물체를 집기 위해 가까이 다가갈 때 사용됩니다.

| 전파 단계(propagating stages)에 대한 추가 정보는 여기에서 확인하세요 - :ref:`Propagating Stages`

Connecting Stage
^^^^^^^^^^^^^^^^
.. image:: ./_static/images/connecting_stage.png

| Connectors는 결과를 전파하지 않고 인접 stages에서 제공된 시작 입력과 목표 입력을 연결하려고 시도합니다.
| connect stage는 종종 시작 상태와 목표 상태 사이의 실행 가능한 궤적을 해결합니다.

| 연결 단계(connecting stages)에 대한 추가 정보는 여기에서 확인하세요 - :ref:`Connecting Stages`

Wrapper
^^^^^^^
| Wrappers는 다른 stage를 캡슐화하여 결과를 수정 또는 필터링합니다.
| wrapper 예시 - ``Generate Grasp Pose`` stage에 대한 ``Compute IK``입니다. 생성 집 ``Generate Grasp Pose`` stage는 카테시안 포즈 솔루션을 생성합니다. ``Generate Pose`` stage 주위에 ``Compute IK`` stage를 래핑하면 ``Generate Pose`` stage의 카테시안 포즈 솔루션을 사용하여 해당 pose가 되기 위한 IK 솔루션(즉, 로봇의 joint state configuration)을 생성할 수 있습니다.

| 래퍼(wrappers)에 대한 추가 정보는 여기에서 확인하세요 - :ref:`Wrappers`

MTC Containers
---------------
| MTC 프레임워크는 컨테이너를 사용하여 stage의 계층적 조직을 활성화하여 순차적 및 병렬 구성을 허용합니다.
| MTC 컨테이너는 stage의 실행 순서를 조직하는데 도움이 됩니다.
| 프로그램적으로 다른 컨테이너 내에 컨테이너를 추가할 수 있습니다.

현재 유효한 컨테이너들:

* Serial

* Parallel

Serial Container
^^^^^^^^^^^^^^^^
| 직렬 컨테이너는 stage를 선형적으로 구성하며, 결과로 end-to-end 솔루션만 고려합니다.
| MTC 태스크는 기본적으로 직렬 컨테이너로 저장됩니다.

Parallel Container
^^^^^^^^^^^^^^^^^^
병렬 컨테이너는 일련의 stages를 결합하여 대체 솔루션을 계획할 수 있도록 합니다.

| 병렬 컨테이너에 대한 자세한 내용은 여기에서 확인할 수 있습니다 - :ref:`Parallel Containers`

Initializing a MTC Task
-----------------------

최상위 계획 문제는 MTC task으로 지정되고 Stages에 의해 지정된 subproblems(하위 문제)는 MTC task object에 추가됩니다.

.. code-block:: c++

  auto node = std::make_shared<rclcpp::Node>();
  auto task = std::make_unique<moveit::task_constructor::Task>();
  task->loadRobotModel(node);
  // Set controllers used to execute robot motion. If not set, MoveIt has controller discovery logic.
  task->setProperty("trajectory_execution_info", "joint_trajectory_controller gripper_controller");


Adding containers and stages to a MTC Task
-------------------------------------------

stage를 MTC task에 추가하기

.. code-block:: c++

  auto current_state = std::make_unique<moveit::task_constructor::stages::CurrentState>("current_state");
  task->add(std::move(current_state));

컨테이너는 Stage에서 파생되므로 컨테이너는 마찬가지로 MTC task에 추가할 수 있습니다.

.. code-block:: c++

  auto container = std::make_unique<moveit::task_constructor::SerialContainer>("Pick Object");
  // TODO: Add stages to the container before adding the container to MTC task
  task->add(std::move(container));

Setting planning solvers
------------------------

motion planning을 수행하는 stages에서는 솔버 정보가 필요합니다.

Solvers available in MTC

* ``PipelinePlanner`` - MoveIt의 planning pipeline을 사용합니다.

* ``JointInterpolation`` - 시작과 목표 조인트 상태 사이를 보간합니다. 복잡한 motion은 지원하지 않습니다.

* ``CartesianPath`` - end-effector를 카테시안 공간에서 직선으로 이동시킵니다.

Code Example on how to initialize the solver

.. code-block:: c++

  const auto mtc_pipeline_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(
      node, "ompl", "RRTConnectkConfigDefault");
  const auto mtc_joint_interpolation_planner =
      std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();
  const auto mtc_cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();

이 솔버들은 ``MoveTo``, ``MoveRelative``, ``Connect``와 같은 stages에 전달됩니다.

Setting Properties
------------------

| 각 MTC stage는 설정 가능한 속성을 가집니다. 예시 - planning group, timeout, 목표 상태 등
| 다른 타입의 속성은 아래 함수를 사용하여 설정할 수 있습니다.

.. code-block:: c++

  void setProperty(const std::string& name, const boost::any& value);

| 자식 stages는 부모로부터 속성을 쉽게 상속받을 수 있으므로 설정 오버헤드를 줄일 수 있습니다.

Cost calculator for Stages
---------------------------

CostTerm은 MTC stage의 솔루션 비용을 계산하는 기본 인터페이스입니다.

MTC에서 사용 가능한 CostTerm 구현

* ``Constant`` - 각 솔루션에 상수 비용 추가

* ``PathLength`` - 비용은 각 joints에 대해서 옵션 가중치를 가지는 경로 길이에 따라 다름

* ``TrajectoryDuration`` - 비용은 전체 궤적의 실행 시간에 따라 다름

* ``TrajectoryCostTerm`` - SubTrajectory 솔루션에서만 작동하는 비용 항

* ``LambdaCostTerm`` - 비용 계산을 위해서 람다 식을 전달

* ``DistanceToReference`` - reference 지점까지의 weighted joint space 거리에 따른 비용

* ``LinkMotion`` - link의 카테시안 경로 길이에 따른 비용

* ``Clearance`` - 비용은 충돌까지의 거리의 역수(inverse)

``LambdaCostTerm``을 사용하여 CostTerm 설정 방법에 대한 예제 코드

.. code-block:: c++

  stage->setCostTerm(moveit::task_constructor::LambdaCostTerm(
        [](const moveit::task_constructor::SubTrajectory& traj) { return 100 * traj.cost(); }));

MTC가 제공하는 모든 stages는 기본 비용 terms을 가지고 있습니다. 궤적을 생성하는 stages는 해결방법으로서 경로 길이(path length)를 비용으로 사용합니다.

Planning and Executing a MTC Task
---------------------------------

MTC 작업을 계획하면 ``MoveItErrorCode``가 반환됩니다. 여기에서 :moveit_msgs_codedir:`here<msg/MoveItErrorCodes.msg>`를 참조하여 다양한 오류 유형을 확인할 수 있습니다.
planning이 성공하면 plan function은 ``moveit_msgs::msg::MoveItErrorCodes::SUCCESS`` 을 반환합니다.

.. code-block:: c++

  auto error_code = task.plan()

After planning, extract the first successful solution and pass it to the execute function. This will create an ``execute_task_solution`` action client.
The action server resides in the ``execute_task_solution_capability`` plugin provided by MTC.
The plugin extends ``MoveGroupCapability``. It constructs a ``MotionPlanRequest`` from the MTC solution and uses MoveIt's ``PlanExecution`` to actuate the robot.

.. code-block:: c++

  auto result = task.execute(*task.solutions().front());


Links to Additional Information
--------------------------------

Here is a :doc:`tutorial </doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor>` on how to create a Pick and Place pipeline using MTC.

The links listed below contain more information on stages and containers provided by MTC

.. toctree::
    :maxdepth: 1

    generating_stages.rst
    propagating_stages.rst
    connecting_stages.rst
    wrappers.rst
    parallel_containers.rst
    debugging_mtc_task.rst