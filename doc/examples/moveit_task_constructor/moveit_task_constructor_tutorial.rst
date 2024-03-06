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
Parallel containers combine a set of stages to allow planning alternate solutions.

| More information on parallel containers can be found here - :ref:`Parallel Containers`.

Initializing a MTC Task
-----------------------

The top-level planning problem is specified as a MTC Task and the subproblems which are specified by Stages are added to the MTC task object.

.. code-block:: c++

  auto node = std::make_shared<rclcpp::Node>();
  auto task = std::make_unique<moveit::task_constructor::Task>();
  task->loadRobotModel(node);
  // Set controllers used to execute robot motion. If not set, MoveIt has controller discovery logic.
  task->setProperty("trajectory_execution_info", "joint_trajectory_controller gripper_controller");


Adding containers and stages to a MTC Task
-------------------------------------------

Adding a stage to MTC task

.. code-block:: c++

  auto current_state = std::make_unique<moveit::task_constructor::stages::CurrentState>("current_state");
  task->add(std::move(current_state));

Containers derive from Stage and hence containers can be added to MTC task similarly

.. code-block:: c++

  auto container = std::make_unique<moveit::task_constructor::SerialContainer>("Pick Object");
  // TODO: Add stages to the container before adding the container to MTC task
  task->add(std::move(container));

Setting planning solvers
------------------------

Stages that do motion planning need solver information.

Solvers available in MTC

* ``PipelinePlanner`` - Uses MoveIt's planning pipeline

* ``JointInterpolation`` - Interpolates between the start and goal joint states. It does not support complex motions.

* ``CartesianPath`` - Moves the end effector in a straight line in Cartesian space.

Code Example on how to initialize the solver

.. code-block:: c++

  const auto mtc_pipeline_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(
      node, "ompl", "RRTConnectkConfigDefault");
  const auto mtc_joint_interpolation_planner =
      std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();
  const auto mtc_cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();

These solvers will be passed into stages like ``MoveTo``, ``MoveRelative``, and ``Connect``.

Setting Properties
------------------

| Each MTC stage has configurable properties. Example - planning group, timeout, goal state, etc.
| Properties of different types can be set using the function below.

.. code-block:: c++

  void setProperty(const std::string& name, const boost::any& value);

| Children stages can easily inherit properties from their parents, thus reducing the configuration overhead.

Cost calculator for Stages
---------------------------

CostTerm is the basic interface to compute costs for solutions for MTC stages.

CostTerm implementations available in MTC

* ``Constant`` - Adds a constant cost to each solution

* ``PathLength`` - Cost depends on trajectory length with optional weight for different joints

* ``TrajectoryDuration`` - Cost depends on execution duration of the whole trajectory

* ``TrajectoryCostTerm`` - Cost terms that only work on SubTrajectory solutions

* ``LambdaCostTerm`` - Pass in a lambda expression to calculate cost

* ``DistanceToReference`` - Cost depends on weighted joint space distance to a reference point

* ``LinkMotion`` - Cost depends on length of Cartesian trajectory of a link

* ``Clearance`` - Cost is inverse of distance to collision

Example code on how to set CostTerm using ``LambdaCostTerm``

.. code-block:: c++

  stage->setCostTerm(moveit::task_constructor::LambdaCostTerm(
        [](const moveit::task_constructor::SubTrajectory& traj) { return 100 * traj.cost(); }));

All stages provided by MTC have default cost terms. Stages which produce trajectories as solutions usually use path length to calculate cost.

Planning and Executing a MTC Task
---------------------------------

Planning an MTC task will return a ``MoveItErrorCode``.
Refer :moveit_msgs_codedir:`here<msg/MoveItErrorCodes.msg>` to identity the different error types.
If planning succeeds, you can expect the plan function to return ``moveit_msgs::msg::MoveItErrorCodes::SUCCESS``.

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