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
최상위 계획 문제는 **태스크(Task)** 로 지정되며, 모든 하위 문제는 **스테이지(Stages)** 로 지정됩니다.
스테이지는 임의의 순서로 배열할 수 있으며, 계층 구조는 개별 스테이지 유형에 의해서만 제한됩니다.
스테이지 배열 순서는 결과가 전달되는 방향에 따라 제한됩니다.
결과 흐름과 관련된 세 가지 가능한 스테이지 유형은 생성기(generator), 전파기(propagator), 연결기(connector) 스테이지입니다.:

**발생기 (Generators)** 는 이웃하는 stages와 독립적으로 결과를 계산하고 양방향으로, 즉 앞뒤로 전달합니다.
예를 들어, 접근 및 이탈 모션 (이웃 stages)이 솔루션에 따라 달라지는 기하학적 포즈에 대한 IK sampler가 있습니다.

**전파기 (Propagators)** 는 한쪽 이웃 stage의 결과를 받고, 하위 문제를 해결한 다음 반대편 이웃에게 결과를 전파합니다.
구현 방법에 따라 전파 단계는 해결책을 앞으로, 뒤로 또는 양 방향으로 별도로 전달할 수 있습니다.
예를 들어, 시작 상태 또는 목표 상태를 기반으로 데카르트 경로를 계산하는 stage가 있습니다.

**연결기 (Connectors)** 는 결과를 전파하지 않고, 두 이웃의 결과 상태 간의 간격을 연결하려고 시도합니다.
예를 들어, 주어진 한 상태에서 다른 상태로 가는 free-motion 계획의 계산입니다.

위의 순서 유형 외에도 하위 단계를 캡슐화 할 수 있는 다른 계층 유형(hierarchy types)이 있습니다.
하위 단계가 없는 단계를 **기본 단계 (primitive stages)** 라고 하고, 상위 단계를 **컨테이너 단계 (container stages)** 라고 합니다.
컨테이너 유형은 세 가지가 있습니다.:

**래퍼(Wrapper)** 는 단일 하위 단계를 캡슐화하고 결과를 수정 또는 필터링합니다.
예를 들어, 자식 단계의 해결책 중 특정 제약 조건을 만족하는 것만 수용하는 필터 단계는 래퍼가 될 수 있습니다.
이러한 유형의 또 다른 표준적인 사용 사례로는 포즈 타겟 속성(pose target property)으로 주석 처리된 계획 장면(planning scenes)을 기반으로 역 운동학(inverse kinematics) 솔루션을 생성하는 IK wrapper stage가 있습니다.

**직렬 컨테이너(Serial Container)** 는 하위 단계 시퀀스를 보유하고 결과로서 끝에서 끝까지의 솔루션(end-to-end solution)만 고려합니다.
일련의 일관된 단계로 구성된 집는 동작(picking motion)이 한 예입니다.

**병렬 컨테이너(Parallel Container)** 는 하위 단계 집합을 결합하고, 대안 결과 중 최상의 결과를 전달하거나, fallback solvers를 실행하거나, 여러 독립적인 해결책을 병합하는 데 사용할 수 있습니다.
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


  **PipelinePlanner** 는 MoveIt의 planning pipeline을 사용하며, 이것은 `OMPL <https://github.com/ompl/ompl>`_ 에 기본값입니다.

  .. code:: c++

        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);

  **JointInterpolation** 은 시작과 목표 joint 상태 사이를 보간하는 간단한 planner입니다. 계산은 빠르지만 복잡한 동작을 지원하지 않기 때문에 일반적으로 간단한 motions에 사용됩니다.

  .. code:: c++

        auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  **CartesianPath** 는 end-effector를 데카르트 공간 내에서 직선 경로를 따라 이동하는 데 사용됩니다.

  .. code:: c++

        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();

다양한 solvers를 사용해보고 로봇 움직임이 어떻게 변하는지 확인해보세요. 첫 번째 stage에서는 데카르트 planner를 사용할 것이며, 다음과 같은 속성을 설정해야 합니다.:

.. code-block:: c++

      auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
      cartesian_planner->setMaxVelocityScalingFactor(1.0);
      cartesian_planner->setMaxAccelerationScalingFactor(1.0);
      cartesian_planner->setStepSize(.01);

이제 planner에 추가했으므로 로봇을 이동시키는 stage를 추가할 수 있습니다.
다음 줄은 ``MoveTo`` stage (propagator stage)를 사용합니다. 손을 펴는 것은 비교적 간단한 동작이므로 joint interpolation planner를 사용할 수 있습니다.
이 stage는 Panda 로봇의 :moveit_resources_codedir:`SRDF<panda_moveit_config/config/panda.srdf>` 내에 정의된 pose의 이름인 "open hand" 포즈로의 이동을 계획합니다.
``createTask()`` 함수를 사용하여 태스크를 반환하고 완료합니다.

.. code-block:: c++

      auto stage_open_hand =
          std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage_open_hand->setGroup(hand_group_name);
      stage_open_hand->setGoal("open");
      task.add(std::move(stage_open_hand));

      return task;
    }

마지막으로 ``main``이 있습니다. 다음 줄은 위에서 정의한 클래스를 사용하여 node를 생성하고 클래스 메서드를 호출하여 기본 MTC 태스크를 설정하고 실행합니다. 이 예에서는 태스트가 실행을 완료하더라도 실행자(executor)를 취소하지 않고 RViz에서 솔루션을 검사하기 위해 node를 유지 관리합니다.

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


5 데모 실행하기
------------------

5.1 Launch Files
^^^^^^^^^^^^^^^^

데모 실행을 위한 환경을 제공하는 ``move_group``, ``ros2_control``, ``static_tf``, ``robot_state_publisher``, ``rviz`` nodes들을 실행하기 위해서는 launch 파일이 필요합니다. 이 예제에서 사용할 파일은 :codedir:`here<tutorials/pick_and_place_with_moveit_task_constructor/launch/mtc_demo.launch.py>` 에 위치합니다.

MoveIt Task Constructor node를 실행하기 위해서는 두 번째 실행 파일을 사용하여 적절한 파라미터와 함께 ``mtc_tutorial`` 실행 파일을 시작합니다. 여기서는 URDF, SRDF, OMPL 파라미터를 로딩하거나 MoveIt Configs Utils를 사용하여 로딩할 수 있습니다. 사용자의 launch 파일은 이 튜토리얼 패키지에서 찾을 수 있는 것과 비슷해야 합니다. :codedir:`here <tutorials/pick_and_place_with_moveit_task_constructor/launch/pick_place_demo.launch.py>` (링크된 launch 파일과 아래의 ``package`` 와 ``executable`` 인수가 다르므로 주의하십시오) :

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

패키지의 launch 디렉토리에 ``pick_place_demo.launch.py`` 이름으로 launch 파일을 저장하거나 다운로드하세요. 다음과 같은 라인을 추가하여 ``CMakeLists.txt`` 파일을 수정하여 launch 폴더를 포함하도록 해야 합니다. : ::

    install(DIRECTORY launch
      DESTINATION share/${PROJECT_NAME}
      )

이제 colcon 워크스페이스를 빌드하고 source할 수 있습니다. ::

    cd ~/ws_moveit
    colcon build --mixin release
    source ~/ws_moveit/install/setup.bash

첫 번째 launch 파일을 실행하는 것부터 시작합니다. 튜토리얼에서 제공하는 파일을 사용하려는 경우: ::

    ros2 launch moveit2_tutorials mtc_demo.launch.py

이제 RViz가 로드됩니다. 여러분이 만든 launch 파일을 사용하고 있고  :codedir:`such as this<tutorials/pick_and_place_with_moveit_task_constructor/launch/mtc.rviz>` 과 같은 rviz 설정을 포함하지 않은 경우, 어떤 것이든 화면에 표시되기 전에 RViz를 구성해야 합니다. 튜토리얼 패키지의 launch 파일을 사용하는 경우라면 RViz는 이미 설정되어 있으므로 다음 섹션의 끝으로 건너뛸 수 있습니다.

5.2 RViz 설정
^^^^^^^^^^^^^^^^^^^^^^

제공된 RViz 설정을 사용하지 않는 경우라면 로봇과 MoveIt Task Constructor 솔루션을 보기 위해 RViz 설정을 약간 변경해야 합니다. 다음은 RViz를 MoveIt Task Constructor 솔루션 시각화를 위해 설정하는 방법입니다.

1. **모션 플래닝(MotionPlanning)** 디스플레이가 활성화되어 있으면 일시적으로 숨기도록 체크 해제하십시오.
2. **전역 옵션(Global Options)** 에서 **고정 프레임(Fixed Frame)** 을 아직 ``panda_link0``으로 변경하지 않은 경우 ``map`` 에서 ``panda_link0`` 으로 변경하십시오.
3. 창 하단 왼쪽에서 **Add** 버튼을 클릭하십시오.
4. ``moveit_task_constructor_visualization`` 아래 **모션 플래닝 작업(Motion Planning Task)** 을 선택하고 OK를 누르십시오. **Motion Planning Tasks** 디스플레이가 하단 왼쪽에 나타나야 합니다.
5. **Displays** 에서 **Motion Planning Tasks** 아래 **Task Solution Topic** 을 ``/solution`` 으로 변경하십시오.

메인 뷰에서 panda arm과 하단 왼쪽에 열린 Motion Planning Tasks display가 표시되며 아무 내용도 표시되지 않아야 합니다.  ``mtc_tutorial`` node를 launch 시키면 MTC task가 이 패널에 표시됩니다.  튜토리얼에서 ``mtc_demo.launch.py`` 를 사용하고 있다면 여기로 돌아오십시오.

5.3 데모 런치시키기
^^^^^^^^^^^^^^^^^^^^^^

``mtc_tutorial`` node를 아래 명령으로 Launch 시킵니다.  ::

    ros2 launch mtc_tutorial pick_place_demo.launch.py

녹색 실린더가 앞에 있으며, arm은 손을 벌리기 위해서 하나의 stage로 작업을 수행하는 것을 볼 수 있습니다. 대략 다음과 같이 보일 것입니다:

.. image:: first_stages.png
   :width: 700px

여러분은 자체 패키지를 만들지 않았지만, 어떤 모습인지 보고 싶다면 튜토리얼에서 이 파일을 launch 시킬 수 있습니다.: ::

    ros2 launch moveit2_tutorials mtc_demo_minimal.launch.py

6 Stages 추가하기
---------------------

지금까지 간단한 태스크를 만들고 실행하는 과정을 살펴봤으며 실행하더라도 실제로는 많은 일을 수행하지 않습니다. 이제 태스크에 pick-and-place stages를 추가하기 시작할 것입니다. 아래 이미지는 태스크에 사용할 stages의 개요를 보여줍니다.

.. image:: stages.png
   :width: 700px

기존의 open hand stage 이후에 stages를 추가하기 시작할 것입니다. ``mtc_node.cpp`` 파일을 열고 다음과 같은 코드를 찾으세요.:

.. code-block:: c++

      auto stage_open_hand =
          std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage_open_hand->setGroup(hand_group_name);
      stage_open_hand->setGoal("open");
      task.add(std::move(stage_open_hand));
      // Add the next lines of codes to define more stages here

6.1 Pick Stages
^^^^^^^^^^^^^^^

물체를 집을 수 있는 위치로 arm을 이동시켜야 합니다. 이 작업은 ``Connect`` stage(이름에서 알 수 있듯이 Connector stage)를 사용하여 수행됩니다. 즉, 이 stage 앞뒤 stage의 결과를 연결하려고 시도합니다. 이 stage는 이름 ``move_to_pick`` 와  planning group과 planner를 지정하는 ``GroupPlannerVector`` 로 초기화됩니다. 그런 다음 stage에 대한 타임아웃을 설정하고, 스테이지 속성을 설정한 다음 태스크에 추가합니다.

.. code-block:: c++

      auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
          "move to pick",
          mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
      stage_move_to_pick->setTimeout(5.0);
      stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage_move_to_pick));


다음으로, MoveIt Task Constructor stage 객체에 대한 포인터를 만들고, 현재로서는 ``nullptr`` 로 설정합니다. 나중에 이것을 사용하여 stage를 저장할 것입니다.

.. code-block:: c++

      mtc::Stage* attach_object_stage =
          nullptr;  // Forward attach_object_stage to place pose generator

다음 코드 블록은 ``SerialContainer(직렬 컨테이너)`` 를 만듭니다. 이는 태스크에 추가할 수 있고 여러 하위 stages를 보유할 수 있는 컨테이너입니다. 이 경우, 직렬 컨테이너를 만들어서 집기 동작(picking action)과 관련된 stages를 포함할 수 있습니다.
stages를 태스크에 추가하는 대신에 관련 stages를 직렬 컨테이너에 추가합니다. ``exposeTo()`` 를 사용하여 새 직렬 컨테이너에서 상위 태스크의 태스크 속성을 선언하고, ``configureInitFrom()`` 을 사용하여 초기화합니다.
이를 통해서 포함된 stages가 이러한 속성에 액세스할 수 있습니다.

.. code-block:: c++

      {
        auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
        grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                              { "eef", "group", "ik_frame" });



우리는 먼저 객체에 접근하는 stage를 만듭니다. 이 stage는  ``MoveRelative`` stage이며, 현재 위치에서 상대적인 이동을 지정할 수 있게 해줍니다.  ``MoveRelative`` 은 전파자(propagator) stage입니다. : 즉, 이웃 stages로부터 솔루션을 받아 다음 단계 또는 이전 stage로 전파합니다. ``cartesian_planner`` 를 사용하여 end-effector를 직선으로 이동시키는 솔루션을 찾습니다. 우리는 속성을 설정하고 이동할 최소 및 최대 거리를 설정합니다. 이제 이동하고자 하는 방향을 가리키는 ``Vector3Stamped`` 메시지를 만듭니다. 이 경우에는 hand frame에서 Z 축 방향으로 이동합니다. 마지막으로 이 stage를 직렬 컨테이너(serial container)에 추가합니다.

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

이제 잡기 포즈(grasp pose)를 생성하는 stage를 생성합니다.
이 단계는 생성기 단계(generator stage) 이므로 앞뒤 stage와 관계없이 결과를 계산합니다.
첫 번째 단계인  ``CurrentState`` 도 생성기 단계입니다. 첫 번째 stage와 이 stage를 연결하려면 위에서 이미 생성한 connecting stage를 사용해야만 합니다.
이 코드는 stage 속성, grasping 전의 포즈, 각도 변화량(angle delta), 모니터링되고 있는 stage를 설정합니다.
각도 변화량은 생성할 포즈 수를 결정하는 ``GenerateGraspPose`` stage의 속성입니다.; MoveIt Task Constructor는 솔루션을 생성할 때 각도 변화량으로 지정된 방향 차이를 가진 여러 방향에서 객체를 잡으려고 시도합니다. 각도 변화량이 작을수록 그립 방향이 더 가까워집니다. 현재 stage를 정의할 때 객체 포즈 및 모양에 대한 정보를 역 운동학 솔버(inverse kinematics solver)에게 전달하는 데 사용되는 ``current_state_ptr`` 을 설정합니다.
이 stage는 생성하는 포즈에 대한 역 운동학(inverse kinematics)을 수행해야 하므로 이전처럼 직렬 컨테이너에 직접 추가되지 않습니다.

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



c

.. code-block:: c++

          Eigen::Isometry3d grasp_frame_transform;
          Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
          grasp_frame_transform.linear() = q.matrix();
          grasp_frame_transform.translation().z() = 0.1;

이제  ``generate pose IK`` 라는 이름과 앞서 정의한 ``generate grasp pose`` stage를 함께 사용하는 ``ComputeIK`` stage를 생성합니다. 일부 로봇은 지정된 포즈에 대해 여러 inverse kinematics 솔루션을 가질 수 있습니다. 솔루션 갯수 제한을 최대 8개로 설정합니다. 또한 최소 솔루션 거리도 설정합니다. 이는 솔루션 간의 차이 임계값입니다. 솔루션의 관절 위치가 이전 솔루션과 너무 유사하면 유효하지 않다고 표시됩니다. 다음으로 몇 가지 추가 속성을 구성하고 "ComputeIK" 스테이지를 직렬 컨테이너에 추가합니다.
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
