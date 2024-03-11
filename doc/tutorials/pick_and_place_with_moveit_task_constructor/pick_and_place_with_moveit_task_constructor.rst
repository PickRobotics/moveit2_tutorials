MoveIt Task Constructor로 Pick and Place 수행하기
============================================================
.. raw:: html

    <video width="700px" controls="true" autoplay="true" loop="true">
        <source src="../../../_static/videos/mtc-demo.webm" type="video/webm">
        MoveIt Task Constructor Pick and Place example
    </video>

이 튜터리얼은 `MoveIt Task Constructor <https://github.com/ros-planning/moveit_task_constructor/tree/ros2/>`_ 사용하여 pick and place 작업을 계획하는 패키지 생성 과정을 소개합니다. MoveIt Task Constructor는 여러 개의 subtasks(stages라고 함)으로 구성된 작업을 계획하는 방법을 제공합니다. 튜토리얼만 실행하고 싶다면 :doc:`Docker Guide </doc/how_to_guides/how_to_setup_docker_containers_in_ubuntu>` 를 따라하면 완성된 튜토리얼이 있는 컨테이너를 시작할 수 있습니다.

1 기본 개념
----------------

MTC의 기본적인 아이디어는 복잡한 모션 계획 문제를 더 간단한 하위 문제 세트로 분할할 수 있다는 것입니다.
최상위 계획 문제는 **태스크(Task)**로 지정되며, 모든 하위 문제는 **스테이지(Stages)**로 지정됩니다.
스테이지는 임의의 순서로 배열할 수 있으며, 계층 구조는 개별 스테이지 유형에 의해서만 제한됩니다.
스테이지 배열 순서는 결과가 전달되는 방향에 따라 제한됩니다.
결과 흐름과 관련된 세 가지 가능한 스테이지 유형은 생성기(generator), 전파기(propagator), 연결기(connector) 스테이지입니다.:

**발생기 (Generators)**는 이웃하는 stages와 독립적으로 결과를 계산하고 양방향으로, 즉 앞뒤로 전달합니다.
예를 들어, 접근 및 이탈 모션 (이웃 stages)이 솔루션에 따라 달라지는 기하학적 포즈에 대한 IK sampler가 있습니다.

**전파기 (Propagators)**는 한쪽 이웃 stage의 결과를 받고, 하위 문제를 해결한 다음 반대편 이웃에게 결과를 전파합니다.
구현 방법에 따라 전파 단계는 해결책을 앞으로, 뒤로 또는 양 방향으로 별도로 전달할 수 있습니다.
예를 들어, 시작 상태 또는 목표 상태를 기반으로 데카르트 경로를 계산하는 stage가 있습니다.

**연결기 (Connectors)**는 결과를 전파하지 않고, 두 이웃의 결과 상태 간의 간격을 연결하려고 시도합니다.
예를 들어, 주어진 한 상태에서 다른 상태로 가는 free-motion 계획의 계산입니다.

위의 순서 유형 외에도 하위 단계를 캡슐화 할 수 있는 다른 계층 유형(hierarchy types)이 있습니다.
하위 단계가 없는 단계를 **기본 단계 (primitive stages)**라고 하고, 상위 단계를 **컨테이너 단계 (container stages)**라고 합니다.
컨테이너 유형은 세 가지가 있습니다.:

**래퍼(Wrapper)**는 단일 하위 단계를 캡슐화하고 결과를 수정 또는 필터링합니다.
예를 들어, 자식 단계의 해결책 중 특정 제약 조건을 만족하는 것만 수용하는 필터 단계는 래퍼가 될 수 있습니다.
이러한 유형의 또 다른 표준적인 사용 사례로는 포즈 타겟 속성(pose target property)으로 주석 처리된 계획 장면(planning scenes)을 기반으로 역 운동학(inverse kinematics) 솔루션을 생성하는 IK wrapper stage가 있습니다.

**직렬 컨테이너(Serial Container)**는 하위 단계 시퀀스를 보유하고 결과로서 끝에서 끝까지의 솔루션(end-to-end solution)만 고려합니다.
일련의 일관된 단계로 구성된 집는 동작(picking motion)이 한 예입니다.

**병렬 컨테이너(Parallel Container)**는 하위 단계 집합을 결합하고, 대안 결과 중 최상의 결과를 전달하거나, fallback solvers를 실행하거나, 여러 독립적인 해결책을 병합하는 데 사용할 수 있습니다.
예를 들어, 자유 이동 계획(free-motion plan)에 대한 대안 계획자(alternative planners) 실행, 오른손 또는 왼손으로 물건 집기 (fallback처럼) 혹은 동시에 팔 이동 및 그리퍼 열기 등이 있습니다.

.. image:: mtc_stage_types.png
   :width: 700px

stages는 단순히 모션 계획 문제를 푸는 것을 지원하는 것이 아닙니다.
모든 종류의 상태 변환(state transitions)에도 사용할 수 있습니다. 예를 들어 계획 장면(planning scene) 수정과 같이 말입니다.
클래스 상속을 사용하는 가능성이 결합되면, 잘 정리된 primitive stages 세트만 사용하면서도 매우 복잡한 behavior을 구성할 수 있습니다.

MTC에 대한 좀더 상세하는 정보는 :doc:`MoveIt Task Constructor concepts page </doc/concepts/moveit_task_constructor/moveit_task_constructor>` 를 참고하세요.

2 시작하기
-----------------
먼저 :doc:`Getting Started </doc/tutorials/getting_started/getting_started>` 에서 설명된 단계를 완료하세요.

2.1 MoveIt Task Constructor 다운받기
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

colcon 워크스페이스로 이동하고 MoveIt Task Constructor 소스를 가져옵니다.: ::

    cd ~/ws_moveit/src
    git clone git@github.com:ros-planning/moveit_task_constructor.git -b ros2

3 Trying It Out
------------------

MoveIt Task Constructor 패키지에는 몇 가지 기본 예제와 pick-and-place 데모가 포함되어 있습니다.
모든 데모를 위해서는 기본 환경을 launch해야 합니다: ::

  ros2 launch moveit_task_constructor_demo demo.launch.py

이어서 다음 개별 데모들을 실행할 수 있습니다: ::

  ros2 run moveit_task_constructor_demo cartesian.launch.py
  ros2 run moveit_task_constructor_demo modular.launch.py
  ros2 launch moveit_task_constructor_demo pickplace.launch.py

오른쪽에 **Motion Planning Tasks** 패널을 확인할 수 있습니다. 이 패널은 태스크의 계층적 stage 구조를 보여줍니다.
특정 stage를 선택하면, 성공한 솔루션과 실패한 솔루션 목록이 가장 오른쪽 창에 표시됩니다.
해당 솔루션 중 하나를 선택하면 시각화가 시작됩니다.

.. image:: mtc_show_stages.gif
   :width: 700px

4 MoveIt Task Constructor로 프로젝트 설정하기
-------------------------------------------------------

이 섹션에서는 MoveIt Task Constructor를 사용하여 간단한 태스크를 만드는 데 필요한 단계를 안내합니다.

4.1 Create a New Package
^^^^^^^^^^^^^^^^^^^^^^^^

다음 명령을 사용하여 새 package를 만드세요: ::

    ros2 pkg create \
    --build-type ament_cmake \
    --dependencies moveit_task_constructor_core rclcpp \
    --node-name mtc_node mtc_tutorial

이 코드는 ``mtc_tutorial`` 이라는 새로운 package와 폴더를 생성합니다. 이 패키지는 ``moveit_task_constructor_core`` 에 의존하며, ``src/mtc_node`` 에서 hello world 예제 코드도 제공합니다.

4.2 코드
^^^^^^^^^^^^

원하는 편집기에서 ``mtc_node.cpp`` 파일을 열고, 다음 코드를 붙여넣으세요.

.. code-block:: c++

    #include <rclcpp/rclcpp.hpp>
    #include <moveit/planning_scene/planning_scene.h>
    #include <moveit/planning_scene_interface/planning_scene_interface.h>
    #include <moveit/task_constructor/task.h>
    #include <moveit/task_constructor/solvers.h>
    #include <moveit/task_constructor/stages.h>
    #if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
    #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
    #else
    #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
    #endif
    #if __has_include(<tf2_eigen/tf2_eigen.hpp>)
    #include <tf2_eigen/tf2_eigen.hpp>
    #else
    #include <tf2_eigen/tf2_eigen.h>
    #endif

    static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
    namespace mtc = moveit::task_constructor;

    class MTCTaskNode
    {
    public:
      MTCTaskNode(const rclcpp::NodeOptions& options);

      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

      void doTask();

      void setupPlanningScene();

    private:
      // Compose an MTC task from a series of stages.
      mtc::Task createTask();
      mtc::Task task_;
      rclcpp::Node::SharedPtr node_;
    };

    MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
      : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
    {
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
    {
      return node_->get_node_base_interface();
    }

    void MTCTaskNode::setupPlanningScene()
    {
      moveit_msgs::msg::CollisionObject object;
      object.id = "object";
      object.header.frame_id = "world";
      object.primitives.resize(1);
      object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      object.primitives[0].dimensions = { 0.1, 0.02 };

      geometry_msgs::msg::Pose pose;
      pose.position.x = 0.5;
      pose.position.y = -0.25;
      pose.orientation.w = 1.0;
      object.pose = pose;

      moveit::planning_interface::PlanningSceneInterface psi;
      psi.applyCollisionObject(object);
    }

    void MTCTaskNode::doTask()
    {
      task_ = createTask();

      try
      {
        task_.init();
      }
      catch (mtc::InitStageException& e)
      {
        RCLCPP_ERROR_STREAM(LOGGER, e);
        return;
      }

      if (!task_.plan(5))
      {
        RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
        return;
      }
      task_.introspection().publishSolution(*task_.solutions().front());

      auto result = task_.execute(*task_.solutions().front());
      if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
        return;
      }

      return;
    }

    mtc::Task MTCTaskNode::createTask()
    {
      mtc::Task task;
      task.stages()->setName("demo task");
      task.loadRobotModel(node_);

      const auto& arm_group_name = "panda_arm";
      const auto& hand_group_name = "hand";
      const auto& hand_frame = "panda_hand";

      // Set task properties
      task.setProperty("group", arm_group_name);
      task.setProperty("eef", hand_group_name);
      task.setProperty("ik_frame", hand_frame);

    // Disable warnings for this line, as it's a variable that's set but not used in this example
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
      mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
    #pragma GCC diagnostic pop

      auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
      current_state_ptr = stage_state_current.get();
      task.add(std::move(stage_state_current));

      auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
      auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

      auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
      cartesian_planner->setMaxVelocityScalingFactor(1.0);
      cartesian_planner->setMaxAccelerationScalingFactor(1.0);
      cartesian_planner->setStepSize(.01);

      auto stage_open_hand =
          std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage_open_hand->setGroup(hand_group_name);
      stage_open_hand->setGoal("open");
      task.add(std::move(stage_open_hand));

      return task;
    }

    int main(int argc, char** argv)
    {
      rclcpp::init(argc, argv);

      rclcpp::NodeOptions options;
      options.automatically_declare_parameters_from_overrides(true);

      auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
      rclcpp::executors::MultiThreadedExecutor executor;

      auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
        executor.add_node(mtc_task_node->getNodeBaseInterface());
        executor.spin();
        executor.remove_node(mtc_task_node->getNodeBaseInterface());
      });

      mtc_task_node->setupPlanningScene();
      mtc_task_node->doTask();

      spin_thread->join();
      rclcpp::shutdown();
      return 0;
    }


4.3 Code Breakdown
^^^^^^^^^^^^^^^^^^

코드 상단에 이 package에서 사용하는 ROS 및 MoveIt 라이브러리 헤더를 include합니다.

 * ``rclcpp/rclcpp.hpp`` : ROS 2 core 기능
 * ``moveit/planning_scene/planning_scene.h`` 와 ``moveit/planning_scene_interface/planning_scene_interface.h`` : robot model과 충돌 객체 인터페이스 기능
 * ``moveit/task_constructor/task.h``, ``moveit/task_constructor/solvers.h``,  ``moveit/task_constructor/stages.h`` : 예제에서 사용하는 MoveIt Task Constructor의 다른 components
 * ``tf2_geometry_msgs/tf2_geometry_msgs.hpp`` 와 ``tf2_eigen/tf2_eigen.hpp`` : 초기 예제에서는 사용하지 않지만 MoveIt Task Constructor 태스크에 stages를 추가할때 pose 생성을 위해서 사용하게 됩니다.

.. code-block:: c++

    #include <rclcpp/rclcpp.hpp>
    #include <moveit/planning_scene/planning_scene.h>
    #include <moveit/planning_scene_interface/planning_scene_interface.h>
    #include <moveit/task_constructor/task.h>
    #include <moveit/task_constructor/solvers.h>
    #include <moveit/task_constructor/stages.h>
    #if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
    #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
    #else
    #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
    #endif
    #if __has_include(<tf2_eigen/tf2_eigen.hpp>)
    #include <tf2_eigen/tf2_eigen.hpp>
    #else
    #include <tf2_eigen/tf2_eigen.h>
    #endif

다음 줄에서는 새로운 node에 대한 로거를 가져옵니다. 편의를 위해 ``moveit::task_constructor`` 에 대해서 namespace alias도 생성합니다.

.. code-block:: c++

    static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
    namespace mtc = moveit::task_constructor;

이제 MoveIt Task Constructor의 주요 기능을 포함하는 클래스를 정의합니다. 또한 클래스의 멤버 변수로 MoveIt Task Constructor 태스크 객체를 선언합니다.: 이는 특정 응용 프로그램에 반드시 필요한 것은 아니지만, 나중에 시각화 목적으로 작업을 저장하는 데 도움이 됩니다. 개별 함수에 대해서는 나중에 자세히 살펴보겠습니다.

.. code-block:: c++

    class MTCTaskNode
    {
    public:
      MTCTaskNode(const rclcpp::NodeOptions& options);

      rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

      void doTask();

      void setupPlanningScene();

    private:
      // Compose an MTC task from a series of stages.
      mtc::Task createTask();
      mtc::Task task_;
      rclcpp::Node::SharedPtr node_;
    };

이 코드는 노드를 지정된 옵션으로 초기화합니다.( ``MTCTaskNode`` 클래스의 생성자 입니다.)

.. code-block:: c++

    MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
      : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
    {
    }

다음 코드는 getter 함수를 정의합니다. 이 함수는 나중에 실행자(executor)를 위해 사용할 node base 인터페이스를 얻어옵니다.

.. code-block:: c++

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
    {
      return node_->get_node_base_interface();
    }

이 클래스 메서드는 예제에서 사용되는 planning scenes을 설정하는 데 사용됩니다. ``object.primitives[0].dimensions`` 로 dimensions을,  ``pose.position.x`` 와 ``pose.position.y`` 로 위치를 지정하여 실린더를 만듭니다.
이 값을 변경하여 실린더의 크기를 조절하고 이동시킬 수 있습니다. 실린더를 로봇의 도달 범위 밖으로 이동시키면 계획은 실패하게 됩니다.

.. code-block:: c++

    void MTCTaskNode::setupPlanningScene()
    {
      moveit_msgs::msg::CollisionObject object;
      object.id = "object";
      object.header.frame_id = "world";
      object.primitives.resize(1);
      object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      object.primitives[0].dimensions = { 0.1, 0.02 };

      geometry_msgs::msg::Pose pose;
      pose.position.x = 0.5;
      pose.position.y = -0.25;
      object.pose = pose;

      moveit::planning_interface::PlanningSceneInterface psi;
      psi.applyCollisionObject(object);
    }

이 함수는 MoveIt Task Constructor 태스크 객체와 연결하는 역할을 합니다. 먼저 태스크를 생성하는데, 이는 일부 속성 설정과 stages 추가를 포함합니다. 이에 대한 자세한 내용은 ``createTask`` 함수 정의에서 설명합니다. 다음으로, ``task.init()`` 은 태스크를 초기화하고, ``task.plan(5)`` 은 5개의 성공적인 계획을 찾을때까지 계획을 생성합니다. 다음 줄은 RViz에서 시각화되도록 솔루션을 publish합니다. 만약 시각화를 원하지 않으면 이 줄을 삭제하면 됩니다. 마지막으로, ``task.execute()`` 는 계획을 실행합니다. 실행은 RViz 플러그인과의 action server 인터페이스를 통해 발생합니다.

.. code-block:: c++

    void MTCTaskNode::doTask()
    {
      task_ = createTask();

      try
      {
        task_.init();
      }
      catch (mtc::InitStageException& e)
      {
        RCLCPP_ERROR_STREAM(LOGGER, e);
        return;
      }

      if (!task_.plan(5))
      {
        RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
        return;
      }
      task_.introspection().publishSolution(*task_.solutions().front());

      auto result = task_.execute(*task_.solutions().front());
      if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
        return;
      }

      return;
    }

위에서 언급했듯이, 이 함수는 MoveIt Task Constructor 객체를 생성하고 몇 가지 초기 속성을 설정합니다. 이 경우에는 태스크 이름을 "demo_task"로 설정하고, robot model을 로딩하고, 몇 가지 유용한 프레임의 이름을 정의하고, 해당 프레임 이름은 ``task.setProperty(property_name, value)`` 를 사용하여 태스크의 속성으로 설정합니다. 다음 몇 개의 코드 블록은 이 함수 몸통을 채울 것입니다.

.. code-block:: c++

    mtc::Task MTCTaskNode::createTask()
    {
      moveit::task_constructor::Task task;
      task.stages()->setName("demo task");
      task.loadRobotModel(node_);

      const auto& arm_group_name = "panda_arm";
      const auto& hand_group_name = "hand";
      const auto& hand_frame = "panda_hand";

      // Set task properties
      task.setProperty("group", arm_group_name);
      task.setProperty("eef", hand_group_name);
      task.setProperty("ik_frame", hand_frame);

이제 해당 node에 예제 stage를 추가합니다. 첫 번째 줄은 ``current_state_ptr`` 을 ``nullptr`` 로 설정합니다.; 이는 특정 시나리오에서 stage 정보를 재사용할 수 있도록 stage를 가리키는 포인터를 생성합니다. 이 줄은 현재 시점에서는 사용되지 않지만, 나중에 태스크에 더 많은 stages가 추가될 때 사용됩니다. 다음으로 ``current_state`` stage (생성기 단계(generator stage))를 만들고 태스크에 추가합니다. - 이렇게 하면 로봇이 현재 상태에서 시작됩니다. 이제 ``CurrentState`` stage를 만들었으므로 나중에 사용하기 위해 포인터를 ``current_state_ptr`` 에 저장합니다.

.. code-block:: c++

      mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
      auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
      current_state_ptr = stage_state_current.get();
      task.add(std::move(stage_state_current));

solvers는 robot motion 타입을 정의하는데 사용됩니다. MoveIt Task Constructor는 solver에 대해서 3가지 옵션을 제공합니다:


  **PipelinePlanner** uses MoveIt's planning pipeline, which typically defaults to `OMPL <https://github.com/ompl/ompl>`_.

  .. code:: c++

        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);

  **JointInterpolation** is a simple planner that interpolates between the start and goal joint states. It is typically used for simple motions as it computes quickly but doesn't support complex motions.

  .. code:: c++

        auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  **CartesianPath** is used to move the end effector in a straight line in Cartesian space.

  .. code:: c++

        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();

Feel free to try out the different solvers and see how the robot motion changes. For the first stage we will use the Cartesian planner, which requires the following properties to be set:

.. code-block:: c++

      auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
      cartesian_planner->setMaxVelocityScalingFactor(1.0);
      cartesian_planner->setMaxAccelerationScalingFactor(1.0);
      cartesian_planner->setStepSize(.01);

Now that we added in the planners, we can add a stage that will move the robot.
The following lines use a ``MoveTo`` stage (a propagator stage). Since opening the hand is a relatively simple movement, we can use the joint interpolation planner.
This stage plans a move to the "open hand" pose, which is a named pose defined in the :moveit_resources_codedir:`SRDF<panda_moveit_config/config/panda.srdf>` for the Panda robot.
We return the task and finish with the ``createTask()`` function.

.. code-block:: c++

      auto stage_open_hand =
          std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage_open_hand->setGroup(hand_group_name);
      stage_open_hand->setGoal("open");
      task.add(std::move(stage_open_hand));

      return task;
    }

Finally, we have ``main``: the following lines create a node using the class defined above, and calls the class methods to set up and execute a basic MTC task. In this example, we do not cancel the executor once the task has finished executing to keep the node alive to inspect the solutions in RViz.

.. code-block:: c++

    int main(int argc, char** argv)
    {
      rclcpp::init(argc, argv);

      rclcpp::NodeOptions options;
      options.automatically_declare_parameters_from_overrides(true);

      auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
      rclcpp::executors::MultiThreadedExecutor executor;

      auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
        executor.add_node(mtc_task_node->getNodeBaseInterface());
        executor.spin();
        executor.remove_node(mtc_task_node->getNodeBaseInterface());
      });

      mtc_task_node->setupPlanningScene();
      mtc_task_node->doTask();

      spin_thread->join();
      rclcpp::shutdown();
      return 0;
    }


5 Running the Demo
------------------

5.1 Launch Files
^^^^^^^^^^^^^^^^

We will need a launch file to launch the ``move_group``, ``ros2_control``, ``static_tf``, ``robot_state_publisher``, and ``rviz`` nodes that provide us the environment to run the demo. The one we will use for this example can be found :codedir:`here<tutorials/pick_and_place_with_moveit_task_constructor/launch/mtc_demo.launch.py>`.

To run the MoveIt Task Constructor node, we will use a second launch file to start the ``mtc_tutorial`` executable with the proper parameters. Here we can load URDF, SRDF, and OMPL parameters, or use MoveIt Configs Utils to do so. Your launch file should look something like the one found in this tutorial package :codedir:`here <tutorials/pick_and_place_with_moveit_task_constructor/launch/pick_place_demo.launch.py>` (pay close attention to the ``package`` and ``executable`` arguments below as they are different from the launch file linked) :

.. code-block:: python

    from launch import LaunchDescription
    from launch_ros.actions import Node
    from moveit_configs_utils import MoveItConfigsBuilder

    def generate_launch_description():
        moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

        # MTC Demo node
        pick_place_demo = Node(
            package="mtc_tutorial",
            executable="mtc_node",
            output="screen",
            parameters=[
                moveit_config,
            ],
        )

        return LaunchDescription([pick_place_demo])

Save a launch file as ``pick_place_demo.launch.py`` or download one to the package's launch directory. Make sure to edit the ``CMakeLists.txt`` so it includes the launch folder by adding the following lines: ::

    install(DIRECTORY launch
      DESTINATION share/${PROJECT_NAME}
      )

Now we can build and source the colcon workspace. ::

    cd ~/ws_moveit
    colcon build --mixin release
    source ~/ws_moveit/install/setup.bash

Start by launching the first launch file. If you want to use the one provided by the tutorials: ::

    ros2 launch moveit2_tutorials mtc_demo.launch.py

RViz will now load. If you're using your own launch file and haven't included an rviz config :codedir:`such as this<tutorials/pick_and_place_with_moveit_task_constructor/launch/mtc.rviz>`, you will need to configure RViz before you see anything displayed. If you're using the launch file from the tutorials package, RViz will already be configured for you and you can jump to the end of the next section.

5.2 RViz Configuration
^^^^^^^^^^^^^^^^^^^^^^

If you are not using the RViz configuration provided, we'll have to make some changes to the RViz configuration to see your robot and the MoveIt Task Constructor solutions. First, start RViz. The following steps will cover how to set up RViz for MoveIt Task Constructor solution visualization.

1. If the **MotionPlanning** display is active, uncheck it to hide it for now.
2. Under **Global Options**, change the **Fixed Frame** from ``map`` to ``panda_link0`` if not already done.
3. On the bottom left of the window, click the **Add** button.
4. Under ``moveit_task_constructor_visualization`` select **Motion Planning Tasks** and click OK. The **Motion Planning Tasks** display should appear on the bottom left.
5. In the **Displays**, under **Motion Planning Tasks**,  change **Task Solution Topic** to ``/solution``

You should see the panda arm in the main view with Motion Planning Tasks display open in the bottom left and nothing in it. Your MTC task will show up in this panel once you launch the ``mtc_tutorial`` node. If you're using ``mtc_demo.launch.py`` from the tutorials, jump back in here.

5.3 Launching the Demo
^^^^^^^^^^^^^^^^^^^^^^

Launch the ``mtc_tutorial`` node with  ::

    ros2 launch mtc_tutorial pick_place_demo.launch.py

You should see the arm execute the task with the single stage to open the hand, with the cylinder in green in front of it. It should look something like this:

.. image:: first_stages.png
   :width: 700px

If you haven't made your own package, but still want to see what this looks like, you can launch this file from the tutorials: ::

    ros2 launch moveit2_tutorials mtc_demo_minimal.launch.py

6 Adding Stages
---------------

So far, we've walked through creating and executing a simple task, which runs but does not do much. Now, we will start adding the pick-and-place stages to the task. The image below shows an outline of the stages we will use in our task.

.. image:: stages.png
   :width: 700px

We will start adding stages after our existing open hand stage. Open ``mtc_node.cpp`` and locate the following lines:

.. code-block:: c++

      auto stage_open_hand =
          std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage_open_hand->setGroup(hand_group_name);
      stage_open_hand->setGoal("open");
      task.add(std::move(stage_open_hand));
      // Add the next lines of codes to define more stages here

6.1 Pick Stages
^^^^^^^^^^^^^^^

We need to move the arm to a position where we can pick up our object. This is done with a ``Connect`` stage, which as its name implies, is a Connector stage. This means that it tries to bridge between the results of the stage before and after it. This stage is initialized with a name, ``move_to_pick``, and a ``GroupPlannerVector`` that specifies the planning group and the planner. We then set a timeout for the stage, set the properties for the stage, and add it to our task.

.. code-block:: c++

      auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
          "move to pick",
          mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
      stage_move_to_pick->setTimeout(5.0);
      stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage_move_to_pick));


Next, we create a pointer to a MoveIt Task Constructor stage object, and set it to ``nullptr`` for now. Later, we will use this to save a stage.

.. code-block:: c++

      mtc::Stage* attach_object_stage =
          nullptr;  // Forward attach_object_stage to place pose generator

This next block of code creates a ``SerialContainer``.
This is a container that can be added to our task and can hold several substages.
In this case, we create a serial container that will contain the stages relevant to the picking action.
Instead of adding the stages to the task, we will add the relevant stages to the serial container. We use ``exposeTo()`` to declare the task properties from the parent task in the new serial container, and use ``configureInitFrom()`` to initialize them.
This allows the contained stages to access these properties.

.. code-block:: c++

      {
        auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
        grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                              { "eef", "group", "ik_frame" });



We then create a stage to approach the object. This stage is a ``MoveRelative`` stage, which allows us to specify a relative movement from our current position. ``MoveRelative`` is a propagator stage: it receives the solution from its neighbouring stages and propagates it to the next or previous stage. Using ``cartesian_planner`` finds a solution that involves moving the end effector in a straight line. We set the properties, and set the minimum and maximum distance to move. Now we create a ``Vector3Stamped`` message to indicate the direction we want to move - in this case, in the Z direction from the hand frame. Finally, we add this stage to our serial container

.. code-block:: c++

        {
          auto stage =
              std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
          stage->properties().set("marker_ns", "approach_object");
          stage->properties().set("link", hand_frame);
          stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
          stage->setMinMaxDistance(0.1, 0.15);

          // Set hand forward direction
          geometry_msgs::msg::Vector3Stamped vec;
          vec.header.frame_id = hand_frame;
          vec.vector.z = 1.0;
          stage->setDirection(vec);
          grasp->insert(std::move(stage));
        }

Now, create a stage to generate the grasp pose.
This is a generator stage, so it computes its results without regard to the stages before and after it.
The first stage, ``CurrentState`` is a generator stage as well - to connect the first stage and this stage, a connecting stage must be used, which we already created above.
This code sets the stage properties, sets the pose before grasping, the angle delta, and the monitored stage.
Angle delta is a property of the ``GenerateGraspPose`` stage that is used to determine the number of poses to generate; when generating solutions, MoveIt Task Constructor will try to grasp the object from many different orientations, with the difference between the orientations specified by the angle delta. The smaller the delta, the closer together the grasp orientations will be. When defining the current stage, we set ``current_state_ptr``, which is now used to forward information about the object pose and shape to the inverse kinematics solver.
This stage won't be directly added to the serial container like previously, as we still need to do inverse kinematics on the poses it generates.

.. code-block:: c++

        {
          // Sample grasp pose
          auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
          stage->properties().configureInitFrom(mtc::Stage::PARENT);
          stage->properties().set("marker_ns", "grasp_pose");
          stage->setPreGraspPose("open");
          stage->setObject("object");
          stage->setAngleDelta(M_PI / 12);
          stage->setMonitoredStage(current_state_ptr);  // Hook into current state



Before we compute inverse kinematics for the poses generated above, we first need to define the frame. This can be done with a ``PoseStamped`` message from ``geometry_msgs`` or in this case, we define the transform using Eigen transformation matrix and the name of the relevant link. Here, we define the transformation matrix.

.. code-block:: c++

          Eigen::Isometry3d grasp_frame_transform;
          Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
          grasp_frame_transform.linear() = q.matrix();
          grasp_frame_transform.translation().z() = 0.1;

Now, we create the ``ComputeIK`` stage, and give it the name ``generate pose IK`` as well as the ``generate grasp pose`` stage defined above. Some robots have multiple inverse kinematics solutions for a given pose - we set the limit on the amount of solutions to solve for up to 8. We also set the minimum solution distance, which is a threshold on how different solutions must be: if the joint positions in a solution are too similar to a previous solution, it will be marked as invalid. Next, we configure some additional properties, and add the ``ComputeIK`` stage to the serial container.

.. code-block:: c++

          // Compute IK
          auto wrapper =
              std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
          wrapper->setMaxIKSolutions(8);
          wrapper->setMinSolutionDistance(1.0);
          wrapper->setIKFrame(grasp_frame_transform, hand_frame);
          wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
          wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
          grasp->insert(std::move(wrapper));
        }

To pick up the object, we must allow collision between the hand and the object. This can be done with a ``ModifyPlanningScene`` stage. The ``allowCollisions`` function lets us specify which collisions to disable.
``allowCollisions`` can be used with a container of names, so we can use ``getLinkModelNamesWithCollisionGeometry`` to get all the names of links with collision geometry in the hand group.

.. code-block:: c++

        {
          auto stage =
              std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
          stage->allowCollisions("object",
                                task.getRobotModel()
                                    ->getJointModelGroup(hand_group_name)
                                    ->getLinkModelNamesWithCollisionGeometry(),
                                true);
          grasp->insert(std::move(stage));
        }

With collisions allowed, we now can close the hand. This is done with a ``MoveTo`` stage, similarly to the ``open hand`` stage from above, except moving to the ``close`` position as defined in the SRDF.

.. code-block:: c++

        {
          auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
          stage->setGroup(hand_group_name);
          stage->setGoal("close");
          grasp->insert(std::move(stage));
        }

We now use a ``ModifyPlanningScene`` stage again, this time to attach the object to the hand using ``attachObject``. Similarly to what we did with the ``current_state_ptr``, we get a pointer to this stage for later use when generating the place pose for the object.

.. code-block:: c++

        {
          auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
          stage->attachObject("object", hand_frame);
          attach_object_stage = stage.get();
          grasp->insert(std::move(stage));
        }

Next, we lift the object with a ``MoveRelative`` stage, similarly to the ``approach_object`` stage.

.. code-block:: c++

        {
          auto stage =
              std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
          stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
          stage->setMinMaxDistance(0.1, 0.3);
          stage->setIKFrame(hand_frame);
          stage->properties().set("marker_ns", "lift_object");

          // Set upward direction
          geometry_msgs::msg::Vector3Stamped vec;
          vec.header.frame_id = "world";
          vec.vector.z = 1.0;
          stage->setDirection(vec);
          grasp->insert(std::move(stage));
        }

With this, we have all the stages needed to pick the object. Now, we add the serial container (with all its substages) to the task. If you build the package as-is, you can see the robot plan to pick up the object.

.. code-block:: c++

        task.add(std::move(grasp));
      }

To test out the newly created stage, build the code and execute: ::

  ros2 launch mtc_tutorial pick_place_demo.launch.py

6.2 Place Stages
^^^^^^^^^^^^^^^^

Now that the stages that define the pick are complete, we move on to defining the stages for placing the object. Picking up where we left off, we add a ``Connect`` stage to connect the two, as we will soon be using a generator stage to generate the pose for placing the object.

.. code-block:: c++

      {
        auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
            "move to place",
            mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                      { hand_group_name, interpolation_planner } });
        stage_move_to_place->setTimeout(5.0);
        stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage_move_to_place));
      }

We also create a serial container for the place stages. This is done similarly to the pick serial container.
The next stages will be added to the serial container rather than the task.

.. code-block:: c++

      {
        auto place = std::make_unique<mtc::SerialContainer>("place object");
        task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
        place->properties().configureInitFrom(mtc::Stage::PARENT,
                                              { "eef", "group", "ik_frame" });

This next stage generates the poses used to place the object and compute the inverse kinematics for those poses - it is somewhat similar to the ``generate grasp pose`` stage from the pick serial container.
We start by creating a stage to generate the poses and inheriting the task properties.
We specify the pose where we want to place the object with a ``PoseStamped`` message from ``geometry_msgs`` - in this case, we choose ``y = 0.5`` in the ``"object"`` frame.
We then pass the target pose to the stage with ``setPose``.
Next, we use ``setMonitoredStage`` and pass it the pointer to the ``attach_object`` stage from earlier.
This allows the stage to know how the object is attached.
We then create a ``ComputeIK`` stage and pass it our ``GeneratePlacePose`` stage - the rest follows the same logic as above with the pick stages.

.. code-block:: c++

        {
          // Sample place pose
          auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
          stage->properties().configureInitFrom(mtc::Stage::PARENT);
          stage->properties().set("marker_ns", "place_pose");
          stage->setObject("object");

          geometry_msgs::msg::PoseStamped target_pose_msg;
          target_pose_msg.header.frame_id = "object";
          target_pose_msg.pose.position.y = 0.5;
          target_pose_msg.pose.orientation.w = 1.0;
          stage->setPose(target_pose_msg);
          stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

          // Compute IK
          auto wrapper =
              std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
          wrapper->setMaxIKSolutions(2);
          wrapper->setMinSolutionDistance(1.0);
          wrapper->setIKFrame("object");
          wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
          wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
          place->insert(std::move(wrapper));
        }

Now that we're ready to place the object, we open the hand with ``MoveTo`` stage and the joint interpolation planner.

.. code-block:: c++

        {
          auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
          stage->setGroup(hand_group_name);
          stage->setGoal("open");
          place->insert(std::move(stage));
        }

We also can re-enable collisions with the object now that we no longer need to hold it.
This is done using ``allowCollisions`` almost exactly the same way as disabling collisions, except setting the last argument to ``false`` rather than ``true``.

.. code-block:: c++

        {
          auto stage =
              std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
          stage->allowCollisions("object",
                                task.getRobotModel()
                                    ->getJointModelGroup(hand_group_name)
                                    ->getLinkModelNamesWithCollisionGeometry(),
                                false);
          place->insert(std::move(stage));
        }

Now, we can detach the object using ``detachObject``.

.. code-block:: c++

        {
          auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
          stage->detachObject("object", hand_frame);
          place->insert(std::move(stage));
        }

We retreat from the object using a ``MoveRelative`` stage, which is done similarly to the ``approach object`` and ``lift object`` stages.

.. code-block:: c++

        {
          auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
          stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
          stage->setMinMaxDistance(0.1, 0.3);
          stage->setIKFrame(hand_frame);
          stage->properties().set("marker_ns", "retreat");

          // Set retreat direction
          geometry_msgs::msg::Vector3Stamped vec;
          vec.header.frame_id = "world";
          vec.vector.x = -0.5;
          stage->setDirection(vec);
          place->insert(std::move(stage));
        }

We finish our place serial container and add it to the task.

.. code-block:: c++

        task.add(std::move(place));
      }

The final step is to return home: we use a ``MoveTo`` stage and pass it the goal pose of ``ready``, which is a pose defined in the Panda SRDF.

.. code-block:: c++

      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setGoal("ready");
        task.add(std::move(stage));
      }

All these stages should be added above these lines.

.. code-block:: c++

      // Stages all added to the task above this line

      return task;
    }

Congratulations! You've now defined a pick and place task using MoveIt Task Constructor! To try it out, build the code and execute: ::

  ros2 launch mtc_tutorial pick_place_demo.launch.py


7 Further Discussion
--------------------

The task with each comprising stage is shown in the Motion Planning Tasks pane. Click on a stage and additional information about the stage will show up to the right. The right pane shows different solutions as well as their associated costs. Depending on the stage type and the robot configuration, there may only be one solution shown.

Click one of the solution costs to see an animation of the robot following the plan for that stage. Click the "Exec" button in the upper-right portion of the pane to execute the motion.

To run the complete MoveIt Task Constructor example included with the MoveIt tutorials: ::

    ros2 launch moveit2_tutorials mtc_demo.launch.py

And in a second terminal: ::

    ros2 launch moveit2_tutorials pick_place_demo.launch.py

7.1 Debugging Information Printed to the Terminal
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When running MTC, it prints a diagram like this to terminal:

.. code-block:: bash

    [demo_node-1]     1  - ←   1 →   -  0 / initial_state
    [demo_node-1]     -  0 →   0 →   -  0 / move_to_home

This example^ shows two stages. The first stage ("initial_state") is a ``CurrentState`` type of stage, which initializes a ``PlanningScene`` and captures any collision objects that are present at that moment.
A pointer to this stage can be used to retrieve the state of the robot.
Since ``CurrentState`` inherits from  ``Generator``, it propagates solutions both forward and backward.
This is denoted by the arrows in both directions.

- The first ``1`` indicates that one solution was successfully propagated backwards to the previous stage.
- The second ``1``, between the arrows, indicates that one solution was generated.
- The ``0`` indicates that a solution was not propagated forward successfully to the next stage, because the next stage failed.

The second stage ("move_to_home") is a ``MoveTo`` type of stage. It inherits its propagation direction from the previous stage, so both arrows point forward. The ``0``'s indicate that this stage failed completely. From left to right, the ``0``'s mean:

- The stage did not receive a solution from the previous stage
- The stage did not generate a solution
- The stage did not propagate a solution forward to the next stage

In this case, we could tell that "move_to_home" was the root cause of the failure. The problem was a home state that was in collision. Defining a new, collision-free home position fixed the issue.

7.2 Stages
^^^^^^^^^^

Information about individual stages can be retrieved from the task. For example, here we retrieve the unique ID for a stage: ::

    uint32_t const unique_stage_id = task_.stages()->findChild(stage_name)->introspectionId();

A ``CurrentState`` type stage does not just retrieve the current state of the robot.
It also initializes a ``PlanningScene`` object, capturing any collision objects that are present at that moment.

MTC stages can be propagated in forward and backward order.
You can easily check which direction a stage propagates by the arrow in the RViz GUI.
When propagating backwards, the logic of many operations is reversed.
For example, to allow collisions with an object in a ``ModifyPlanningScene`` stage, you would call ``allowCollisions(false)`` rather than ``allowCollisions(true)``. There is a discussion to be read `here. <https://github.com/ros-planning/moveit_task_constructor/issues/349>`_
