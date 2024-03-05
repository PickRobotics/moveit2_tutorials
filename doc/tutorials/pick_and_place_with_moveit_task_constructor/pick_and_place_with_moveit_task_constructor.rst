Pick and Place with MoveIt Task Constructor
===========================================
.. raw:: html

    <video width="700px" controls="true" autoplay="true" loop="true">
        <source src="../../../_static/videos/mtc-demo.webm" type="video/webm">
        MoveIt Task Constructor Pick and Place example
    </video>

이 안내 문서는 `MoveIt Task Constructor <https://github.com/ros-planning/moveit_task_constructor/tree/ros2/>`_ 사용하여 pick and place 작업을 계획하는 패키지 생성 과정을 소개합니다. MoveIt Task Constructor는 여러 개의 subtasks(stages라고 함)으로 구성된 작업을 계획하는 방법을 제공합니다. 튜토리얼만 실행하고 싶다면 :doc:`Docker Guide </doc/how_to_guides/how_to_setup_docker_containers_in_ubuntu>`를 따라 완성된 튜토리얼이 있는 컨테이너를 시작할 수 있습니다. 이 튜토리얼은 MoveIt과 MoveIt Task Constructor :ref:`concepts <moveit_task_constructor_concepts>`에 대한 기본적인 이해가 있는 사용자를 대상으로 합니다. 이에 대한 자세한 내용은 :doc:`MoveIt examples </doc/examples/examples>`를 읽고, :doc:`MoveIt Task Constructor </doc/examples/moveit_task_constructor/moveit_task_constructor_tutorial>` 예시 페이지를 포함한 관련 문서를 참조하십시오.

Getting Started
---------------
아직 완료하지 않았다면, :doc:`Getting Started </doc/tutorials/getting_started/getting_started>`의 단계를 완료하세요.

Download MoveIt Task Constructor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

colcon workspace으로 이동하여 MoveIt Task Constructor 소스를 가져옵니다: ::

    cd ~/ws_moveit2/src
    git clone https://github.com/ros-planning/moveit_task_constructor.git -b ros2

Create a New Package
^^^^^^^^^^^^^^^^^^^^

다음 명령으로 새로운 패키지를 만듭니다: ::

    ros2 pkg create --build-type ament_cmake --node-name mtc_tutorial mtc_tutorial

이 명령은 ``src/mtc_node``에 hello world 예제가 있는 ``mtc_tutorial``이라는 새 폴더를 생성합니다. 다음으로, ``package.xml``에 의존성을 추가합니다. 다음과 비슷한 형태입니다.: ::

    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="3">
    <name>mtc_tutorial</name>
    <version>0.0.0</version>
    <description>TODO: Package description</description>
    <maintainer email="youremail@domain.com">user</maintainer>
    <license>TODO: License declaration</license>

    <buildtool_depend>ament_cmake</buildtool_depend>

    <depend>moveit_task_constructor_core</depend>
    <depend>rclcpp</depend>

    <test_depend>ament_lint_auto</test_depend>
    <test_depend>ament_lint_common</test_depend>

    <export>
        <build_type>ament_cmake</build_type>
    </export>
    </package>

또한, ``CMakeLists.txt``에 대한 종속성을 추가하세요. 파일은 다음과 같은 형태입니다.: ::

    cmake_minimum_required(VERSION 3.8)
    project(mtc_tutorial)

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    # find dependencies
    find_package(ament_cmake REQUIRED)
    find_package(moveit_task_constructor_core REQUIRED)
    find_package(rclcpp REQUIRED)
    # uncomment the following section in order to fill in
    # further dependencies manually.
    # find_package(<dependency> REQUIRED)

    add_executable(mtc_tutorial src/mtc_tutorial.cpp)
    ament_target_dependencies(mtc_tutorial moveit_task_constructor_core rclcpp)
    target_include_directories(mtc_tutorial PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
    target_compile_features(mtc_tutorial PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

    install(TARGETS mtc_tutorial
    DESTINATION lib/${PROJECT_NAME})

    if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
    endif()

    ament_package()


Setting up a Project with MoveIt Task Constructor
-------------------------------------------------

이 섹션에서는 MoveIt Task Constructor를 사용하여 최소 작업을 빌드하는 데 필요한 코드를 안내합니다.

The Code
^^^^^^^^

편집기에서 ``mtc_tutorial.cpp``를 열고 다음 코드를 붙여넣으십시오.

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

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
    {
      return node_->get_node_base_interface();
    }

    MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
      : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
    {
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


Code Breakdown
^^^^^^^^^^^^^^

코드 상단에는 이 패키지가 사용하는 ROS 및 MoveIt 라이브러리를 포함합니다.

 * ``rclcpp/rclcpp.hpp``에는 ROS2 코어 기능이 포함합니다.
 * ``moveit/planning_scene/planning_scene.h`` 와 ``moveit/planning_scene_interface/planning_scene_interface.h``에는 robot model과 충돌 객체와 인터페이스하는 기능을 포함합니다.
 * ``moveit/task_constructor/task.h``, ``moveit/task_constructor/solvers.h``, ``moveit/task_constructor/stages.h``에는 예제에서 사용되는 MoveIt Task Constructor의 여러 구성 요소가 포함합니다.
 * ``tf2_geometry_msgs/tf2_geometry_msgs.hpp`` 와 ``tf2_eigen/tf2_eigen.hpp``는 초기 예제에서 사용되지 않지만, MoveIt Task Constructor 작업에 더 많은 단계를 추가할 때 pose 생성을 위해 사용됩니다.

다음 라인에서 새 node에 대한 logger를 얻습니다. 편의를 위해 ``moveit::task_constructor``에 대한 namespace alias도 생성합니다.

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

우리는 MoveIt Task Constructor의 주요 기능을 포함하는 클래스를 정의하는 것으로 시작합니다. 또한 클래스의 멤버 변수로 MoveIt Task Constructor 작업 객체를 선언합니다. 이는 해당 응용 프로그램에 반드시 필요하지는 않지만, 뒷날 시각화를 위한 작업을 저장하는데 도움이 됩니다. 각 함수는 아래에서 개별적으로 살펴볼 것입니다.

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

이 코드는 나중에 executor에 사용될 node base interface를 가져오는 getter 함수를 정의합니다.

.. code-block:: c++

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
    {
      return node_->get_node_base_interface();
    }

다음 코드는 지정된 옵션으로 node를 초기화합니다.

.. code-block:: c++

    MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
      : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
    {
    }

이 클래스 method는 예제에서 사용되는 planning scene을 설정하는 데 사용됩니다. ``object.primitives[0].dimensions``에 의해 지정된 치수와 ``pose.position.z`` 와 ``pose.position.x``에 의해 지정된 위치를 가진 실린더를 생성합니다. 이러한 숫자를 변경하여 cylinder의 크기를 조정하고 cylinder를 이동시킬 수 있습니다. cylinder를 로봇이 닿을 수 없는 위치로 이동하면 계획이 실패합니다.

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

이 함수는 MoveIt Task Constructor task 객체와 상호 작용합니다. 먼저 하나의 task를 생성하는데, 여기에는 일부 속성 설정과 stages 추가를 포함하고 있습니다. 이에 대한 자세한 내용은 ``createTask`` 함수 정의에서 설명합니다. 다음으로, ``task.init()``은 작업을 초기화하고, ``task.plan(5)``은 하나의 계획을 생성하는데 5개의 성공적인 계획이 발견된 후에 중지하는 것이다. 다음 줄은 RViz에서 시각화하기 위해서 해당 솔루션을 publish합니다. - 시각화를 원하지 않으면 이 줄을 제거할 수 있습니다. 마지막으로, ``task.execute()``는 해당 계획을 실행합니다. RViz 플러그인과 함께 action server interface를 통해서 execution이 발생합니다.

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

위에서 설명된 대로, 이 함수는 MoveIt Task Constructor 객체를 생성하고 몇 가지 초기 속성을 설정합니다. 이 예시에서는 작업 이름을 "demo_task"로 설정하고, robot model을 로드하며, 몇 가지 유용한 프레임의 이름을 정의하고 이러한 프레임 이름을 ``task.setProperty(property_name, value)``를 사용하여 작업 속성으로 설정합니다. 다음 몇 개의 코드 블록에서는 이 함수 본문을 채울 것입니다.

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

이제 node에 예제 stage를 추가합니다. 첫 번째 줄은 ``current_state_ptr``를  ``nullptr``로 설정합니다. 이는 특정 시나리오에서 stage 정보를 재사용할 수 있도록 stage를 가리키는 포인터를 만듭니다. 이 라인은 현재 사용되지는 않지만 나중에 작업에 더 많은 stages가 추가될 때 사용됩니다. 다음으로 ``current_state`` stage (generator stage)를 만들어 작업에 추가합니다. - 이렇게 하면 로봇이 현재 상태에서 시작됩니다. ``CurrentState`` stage를 만들었으므로 나중에 사용할 수 있도록 ``current_state_ptr``에 대한 포인터를 저장합니다.

.. code-block:: c++

      mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
      auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
      current_state_ptr = stage_state_current.get();
      task.add(std::move(stage_state_current));

로봇 motion을 계획하려면 솔버를 지정해야 합니다. MoveIt Task Constructor는 세 가지 솔버 옵션을 제공합니다.:

 * ``PipelinePlanner``은 MoveIt의 planning pipeline을 사용하며 일반적으로 OMPL로 기본값이 설정됩니다.
 * ``CartesianPath``는 end-effector를 데카르트 공간내에서 직선으로 이동하는 데 사용됩니다.
 * ``JointInterpolation``는 시작 및 목표 joint stages 사이를 보간하는 간단한 planner입니다. 일반적으로 간단한 motion에 대해서 빠른 연산을 수행하지만 복잡한 motion은 지원하지 않는다.

우리는 데카르트 planner에 맞게 일부 속성을 설정한다.

.. code-block:: c++

      auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
      auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

      auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
      cartesian_planner->setMaxVelocityScalingFactor(1.0);
      cartesian_planner->setMaxAccelerationScalingFactor(1.0);
      cartesian_planner->setStepSize(.01);

이제 planner에 추가했으므로, 이제 로봇을 이동시키는 stage를 추가할 수 있습니다. 다음 코드는 ``MoveTo`` stage (propagator stage)를 사용합니다. 손을 펴는 것은 비교적 간단한 동작이므로 관절 보간 플래너(joint interpolation planner)를 사용할 수 있습니다. 이 stage는 "open hand" pose로 움직임을 plan하는 것으로서, panda robot을 위해 :moveit_resources_codedir:`SRDF<panda_moveit_config/config/panda.srdf>` 파일에 정의된 pose 이름이다. 우리는 task를 반환하고 createTask() 함수를 사용하여 작업을 완료합니다.

.. code-block:: c++

      auto stage_open_hand =
          std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage_open_hand->setGroup(hand_group_name);
      stage_open_hand->setGoal("open");
      task.add(std::move(stage_open_hand));

      return task;
    }

마지막으로 ``main`` 부분이 있습니다. 다음 코드는 위에서 정의한 클래스를 사용하여 node를 만들고 클래스 메서드를 호출하여 기본 MTC 작업을 설정하고 실행합니다. 이 예제에서는 task 실행이 완료되면 ececutor를 취소하지 않고 RViz에서 해당 솔루션을 검사하기 위해 node를 살아 있는 상태로 유지합니다.

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


Running the Demo
----------------

Launch files
^^^^^^^^^^^^

``move_group``, ``ros2_control``, ``static_tf``, ``robot_state_publisher``, ``rviz`` 를 실행하려면 launch 파일이 필요합니다. :codedir:`Here <tutorials/pick_and_place_with_moveit_task_constructor/launch/mtc_demo.launch.py>` 는 튜토리얼 패키지에서 사용하는 launch 파일입니다. 이 파일을 여러분의 패키지의 launch 디렉토리에 넣으세요.

MoveIt Task Constructor node를 실행하려면, 적절한 파라미터를 가지고 ``mtc_tutorial`` 실행자를 구동시킬 수 있는 2번째 launch 파일이 필요합니다. URDF, SRDF, OMPL 파라미터를 직접 로딩하거나 MoveIt Configs Utils를 사용하여 로딩하십시오. launch 파일은 다음과 형태입니다.:

.. code-block:: python

    from launch import LaunchDescription
    from launch_ros.actions import Node
    from moveit_configs_utils import MoveItConfigsBuilder

    def generate_launch_description():
        moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

        # MTC Demo node
        pick_place_demo = Node(
            package="mtc_tutorial",
            executable="mtc_tutorial",
            output="screen",
            parameters=[
                moveit_config,
            ],
        )

        return LaunchDescription([pick_place_demo])

패키지의 launch 디렉토리에 이 파일을 ``pick_place_demo.launch.py``라는 이름으로 저장하십시오. launch 파일이 올바르게 설치되도록 ``CMakeLists.txt`` 파일에 다음 행을 추가하십시오. ::

   install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

이제 colcon workspace를 빌드하고 소스하십시오. ::

    cd ~/ws_moveit2
    colcon build --mixin release
    source ~/ws_moveit2/install/setup.bash

첫 번째 launch 파일을 실행하는 것부터 시작하십시오. 튜토리얼에서 제공하는 것을 사용하려면: ::

    ros2 launch moveit2_tutorials mtc_demo.launch.py

RViz가 로드되어야 합니다. 자체 launch 파일을 사용하는 경우 아무것도 볼 수 없기 전에 RViz를 구성해야 합니다. 튜토리얼 패키지의 launch 파일을 사용하는 경우 이것은 이미 설정되어 있습니다.

RViz Configuration
^^^^^^^^^^^^^^^^^^

RViz에서 여러분의 robot과 MoveIt Task Constructor 솔루션을 확인하려면 RViz 설정을 약간 변경해야 합니다. 먼저 RViz를 시작하십시오. 다음 단계에서는 MoveIt Task Constructor 솔루션 시각화를 위해 RViz를 어떻게 설정하는지 방법을 설명합니다.

1. **MotionPlanning** 표시가 활성화되어 있으면 지금은 숨기려면 체크를 해제하세요.
2. **Global Options** 아래에 있는 **Fixed Frame**을 ``map``에서 ``panda_link0``으로 변경하십시오.(아직 설정되어 있지 않은 경우)
3. 창 하단 왼쪽에서 **Add** 버튼을 클릭하십시오.
4. ``moveit_task_constructor_visualization`` 아래에서 **Motion Planning Tasks**을 선택하고 OK를 클릭하십시오. **Motion Planning Tasks** 표시가 왼쪽 하단에 나타납니다.
5. **Displays**에서 **Motion Planning Tasks** 아래에서 **Task Solution Topic**을 ``/solution``으로 변경하십시오

메인 뷰에서 Motion Planning Tasks 표시가 하단 왼쪽에 열려 있고 그 안에 아무것도 없는 panda arm이 표시되어야 합니다. ``mtc_tutorial`` node를 실행하면 MTC task가 이 패널에 표시됩니다. 튜토리얼에 있는 ``mtc_demo.launch.py``를 사용하는 경우 여기로 다시 돌아가십시오.

Launching the Demo
^^^^^^^^^^^^^^^^^^

다음 명령으로 ``mtc_tutorial`` node를 실행 ::

    ros2 launch mtc_tutorial pick_place_demo.launch.py

실린더가 arm 앞에 녹색으로 표시되어 있고, arm은 손을 펴서 단일 stage로 작업을 수행하는 것을 확인할 수 있습니다. 다음과 같이 보입니다.:

.. image:: first_stages.png
   :width: 700px

여러분이 패키지를 별도로 만들지 않았지만 어떻게 동작하는지 보고싶다면 다음 명령을 사용하여 튜토리얼에서 이 파일을 실행할 수 있습니다: ::

    ros2 launch moveit2_tutorials mtc_demo_minimal.launch.py

Adding Stages
-------------

지금까지 우리는 별다른 일을 수행하지 않는 간단한 task을 만들고 실행하는 과정을 살펴보았습니다. 이제는 pick-and-place stages를 task에 추가하는 것을 시작할 것입니다. 아래 이미지는 task에서 사용할 stages의 개요를 보여줍니다. MoveIt Task Constructor의 개념과 다양한 stage type에 대해 더 자세히 알아보려면 다음 문서를 참조하십시오. :doc:`example page for MoveIt Task Constructor </doc/examples/moveit_task_constructor/moveit_task_constructor_tutorial>`.

.. image:: stages.png
   :width: 700px

기존 open hand stage 뒤에 다음과 같은 단계를 추가를 시작할 것입니다.:

.. code-block:: c++

      auto stage_open_hand =
          std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage_open_hand->setGroup(hand_group_name);
      stage_open_hand->setGoal("open");
      task.add(std::move(stage_open_hand));
      // Add the next lines of codes to define more stages here

Pick Stages
^^^^^^^^^^^

arm을 물건을 집었던 위치로 이동시켜야 합니다. 이것은 ``Connect`` stage로 수행하는데, 이 단계는 이름에서 알 수 있듯이 이전 stage와 다음 stage의 결과를 연결하는 Connector stage입니다. 이 stage는 이름 (``move_to_pick``)과 계획 그룹 및 계획자를 지정하는 ``GroupPlannerVector``로 초기화됩니다. 그 다음, 단계의 시간 제한을 설정하고 stage에 대한 속성을 설정한 다음 작업에 추가합니다.

.. code-block:: c++

      auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
          "move to pick",
          mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
      stage_move_to_pick->setTimeout(5.0);
      stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage_move_to_pick));

다음으로, MoveIt Task Constructor stage 객체에 대한 포인터를 만들고 현재로서는 "nullptr"로 설정합니다. 나중에 이를 사용하여 stage를 저장할 것입니다.

.. code-block:: c++

      mtc::Stage* attach_object_stage =
          nullptr;  // Forward attach_object_stage to place pose generator

다음 코드 블록은 ``SerialContainer``를 만듭니다. 이 컨테이너는 우리 task에 추가될 수 있으며 여러 substages를 포함할 수 있습니다. 이 경우, 컨테이너는 픽킹 작업과 관련된 단계를 포함하는 직렬 컨테이너를 만듭니다. stage를 task에 추가하는 대신 관련 stage를 직렬 컨테이너에 추가합니다. ``exposeTo``를 사용하여 parent task의 task 속성을 새로운 직렬 컨테이너내에 선언하고, configureInitFrom()을 사용하여 초기화합니다. 이렇게 하면 contained stages가 이런 속성들에 액세스할 수 있습니다.

.. code-block:: c++

      {
        auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
        grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                              { "eef", "group", "ik_frame" });


그런 다음, 객체에 접근하는 하나의 stage를 만듭니다. 이 stage는 ``MoveRelative`` stage로서, 현재 위치에서 상대적인 이동을 지정할 수 있습니다. ``MoveRelative``은 propagator stage입니다: 즉, 이웃 stages로부터 솔루션을 받아 다음 단계 또는 이전 stages로 전파합니다. ``cartesian_planner``를 사용하면 end-effector를 직선으로 이동시키는 솔루션을 찾습니다. 속성을 설정하고 이동할 최소 및 최대 거리를 설정합니다. 이제 이동하려는 방향(이 경우 hand frame에서 Z 방향)을 나타내는 ``Vector3Stamped`` 메시지를 만들고 이 stage를 직렬 컨테이너에 추가합니다.

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

이제 grasp pose(물체 잡기)를 생성하는 stage를 만들어 보겠습니다. 이 stage는 generator stage이므로 이전이나 이후의 stages와 상관없이 결과를 계산합니다. 첫 번째 단계인 ``CurrentState``도 마찬가지로 generator stage입니다. 첫 번째 stage와 이 stage를 연결하려면 이미 만들어 놓은 연결 stage를 사용해야 합니다. 이 코드는 stage 속성, 잡기 전 pose, 각도 변화량(angle delta), 모니터링된 stage를 설정합니다. 각도 변화량(angle delta)은 ``GenerateGraspPose`` stage의 속성이며 생성할 poses의 수를 결정하는 데 사용됩니다. MoveIt Task Constructor가 솔루션을 생성할 때, 각도 변화량으로 지정한 차이를 가진 여러 방향에서 물체를 잡으려고 시도합니다. delta 값이 작을수록 그립 방향이 더 가까워집니다. 현재 stage를 정의할 때, 현재 물체 자세 및 모양에 대한 정보를 역 운동학 솔버(inverse kinematic solver)에 전달하는 데 사용되는 ``current_state_ptr``을 설정합니다. 이 stage는 이전처럼 직렬 컨테이너에 직접 추가되지 않으며, 생성된 poses에 대해서 역 운동학(inverse kinematics)을 수행해야 합니다.

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


위에서 생성한 pose에 대한 역 운동학(inverse kinematics)을 계산하기 전에, 먼저 프레임을 정의해야 합니다. 이 작업은 ``geometry_msgs`` 패키지의 ``PoseStamped`` 메시지를 사용하거나, 이 경우와 같이 Eigen 변환 행렬과 관련 link 이름을 사용하여 변환을 정의할 수 있습니다. 여기에서는 변환 행렬을 정의합니다.

.. code-block:: c++

          Eigen::Isometry3d grasp_frame_transform;
          Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
          grasp_frame_transform.linear() = q.matrix();
          grasp_frame_transform.translation().z() = 0.1;

이제 ``generate pose IK``이라는 이름의 ``ComputeIK`` stage를 만들고, 위에서 정의한 ``generate grasp pose`` stage 스테이지를 연결합니다. 일부 로봇은 주어진 pose에 대해서 여러 역 운동학 해를 가질 수 있습니다. 우리는 최대 8개까지 해를 푸는 제한을 설정합니다. 또한 최소 해 거리를 설정합니다. 이는 solution이 얼마나 차이가 나는지 정도를 나타내는 임계값입니다. 만약 solution내에 joint positions가 이전 해와 너무 유사하면 해당 해는 유효하지 않은 것으로 표시됩니다. 다음으로 몇 가지 추가 속성을 구성하고 ``ComputeIK`` stage를 연속 컨테이너에 추가합니다.

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

물체를 집어 들기 위해서는 hand와 물체 사이의 충돌을 허용해야만 합니다. 이 작업은 ``ModifyPlanningScene`` stage를 사용하여 수행할 수 있습니다.``allowCollisions`` 함수를 사용하여 어떤 충돌을 비활성화시킬지 지정할 수 있습니다.``allowCollisions``은 이름 컨테이너와 함께 사용할 수 있으므로,``getLinkModelNamesWithCollisionGeometry`` 함수를 사용하여 hand 그룹내에서 충돌 형상을 가지고 있는 links의 모든 이름을 얻을 수 있습니다.

In order to pick up the object, we must allow collision between the hand and the object. This can be done with a ``ModifyPlanningScene`` stage. The ``allowCollisions`` function lets us specify which colㅍlisions to disable.
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


Place Stages
^^^^^^^^^^^^

Now that the stages that define the pick are complete, we move on to defining the stages for placing the object. We start with a ``Connect`` stage to connect the two, as we will soon be using a generator stage to generate the pose for placing the object.

.. code-block:: c++

      {
        auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
            "move to place",
            mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                      { hand_group_name, sampling_planner } });
        stage_move_to_place->setTimeout(5.0);
        stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage_move_to_place));
      }

We also create a serial container for the place stages. This is done similarly to the pick serial container. The next stages will be added to the serial container rather than the task.

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

We also can re-enable collisions with the object now that we no longer need to hold it. This is done using ``allowCollisions`` almost exactly the same way as disabling collisions, except setting the last argument to ``false`` rather than``true``.

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

The final step is to return home: we use a ``MoveTo`` stage and pass it the goal pose of ``ready``, which is a pose defined in the panda SRDF.

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

Congratulations! You've now defined a pick and place task using MoveIt Task Constructor!

Visualizing with RViz
---------------------

The task with each comprising stage is shown in the Motion Planning Tasks pane. Click on a stage and additional information about the stage will show up to the right. The right pane shows different solutions as well as their associated costs. Depending on the stage type and the robot configuration, there may only be one solution shown.

Click one of the solution costs to see an animation of the robot following the plan for that stage. Click the "Exec" button in the upper-right portion of the pane to execute the motion.

To run the complete MoveIt Task Constructor example included with the MoveIt tutorials: ::

    ros2 launch moveit2_tutorials mtc_demo.launch.py

And in a second terminal: ::

    ros2 launch moveit2_tutorials pick_place_demo.launch.py

Debugging from terminal
^^^^^^^^^^^^^^^^^^^^^^^

When running MTC, it prints a diagram like this to terminal:

.. code-block:: bash

    [demo_node-1]     1  - ←   1 →   -  0 / initial_state
    [demo_node-1]     -  0 →   0 →   -  0 / move_to_home

This example^ shows two stages. The first stage ("initial_state") is a ``CurrentState`` type of stage, which initializes a PlanningScene and captures any collision objects that are present at that moment. A pointer to this stage can be used to retrieve the state of the robot. Since CurrentState inherits from  ``Generator``, it propagates solutions both forward and backward. This is denoted by the arrows in both directions. The first ``1`` indicates that one solution was successfully propagated backwards to the previous stage. The second ``1``, between the arrows, indicates that one solution was generated. The ``0`` indicates that a solution was not propagated forward successfully to the next stage, because the next stage failed.

The second stage ("move_to_home") is a ``MoveTo`` type of stage. It inherits its propagation direction from the previous stage, so both arrows point forward. The ``0``'s indicate that this stage failed completely. From left to right, the ``0``'s mean:

- The stage did not receive a solution from the previous stage
- The stage did not generate a solution
- The stage did not propagate a solution forward to the next stage

In this case, we could tell that "move_to_home" was the root cause of the failure. The problem was a home state that was in collision. Defining a new, collision-free home position fixed the issue.

Various hints
^^^^^^^^^^^^^

Information about individual stages can be retrieved from the task. For example, here we retrieve the unique ID for a stage: ::

    uint32_t const unique_stage_id = task_.stages()->findChild(stage_name)->introspectionId();

A CurrentState type stage does not just retrieve the current state of the robot. It also initializes a PlanningScene object, capturing any collision objects that are present at that moment.

MTC stages can be propagated in forward and backward order. You can easily check which direction a stage propagates by the arrow in the RViz GUI. When propagating backwards, the logic of many operations is reversed. For example, to allow collisions with an object in a ``ModifyPlanningScene`` stage, you would call ``allowCollisions(false)`` rather than ``allowCollisions(true)``. There is a discussion to be read `here. <https://github.com/ros-planning/moveit_task_constructor/issues/349>`_
