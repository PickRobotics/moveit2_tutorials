Visualizing In RViz
===================

이 튜터리얼은 RViz에서 시각화를 렌더링하여 MoveIt 응용 프로그램이 수행하는 작업을 더 쉽게 이해하는 데 도움이 되는 도구를 소개합니다.

Prerequisites
-------------

이미 완료하지 않았다면, :doc:`Your First Project </doc/tutorials/your_first_project/your_first_project>`을 완료하세요.
이 프로젝트는 이전 튜토리얼이 중단된 부분에서 시작하는 ``hello_moveit`` 프로젝트를 사용한다고 가정합니다.

Steps
-----

1 Add the dependency moveit_visual_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``hello_moveit`` 프로젝트의 ``package.xml`` 파일에서 다른 ``<depend>`` 구문 뒤에 다음 줄을 추가하십시오.:

.. code-block:: xml

  <depend>moveit_visual_tools</depend>

다음으로 ``CMakeLists.txt`` 파일에서 find_package 구문의 섹션에 다음 줄을 추가합니다.:

.. code-block:: cmake

  find_package(moveit_visual_tools REQUIRED)

파일 하단 부분에서 ``ament_target_dependencies`` 매크로 호출을 다음과 같이 확장하여 새로운 dependency 항목을 포함하세요:

.. code-block:: cmake

  ament_target_dependencies(
    hello_moveit
    "moveit_ros_planning_interface"
    "moveit_visual_tools"
    "rclcpp"
  )

의존 관계가 제대로 추가되었는지 확인하려면 소스 파일 ``hello_moveit.cpp``에 다음과 같은 헤더 파일 포함을 추가하십시오:

.. code-block:: C++

  #include <moveit_visual_tools/moveit_visual_tools.h>

모든 작업이 정상적으로 이루어졌는지 테스트하기 위해 workspace 디렉토리에서 터미널을 열고 (opt 디렉토리에 설치된 ROS를 source하는 것을 잊지 마세요) 다음 colcon 명령으로 빌드하세요:

.. code-block:: bash

  cd ~/ws_moveit2
  colcon build --mixin debug

2 Create a ROS executor and spin the node on a thread
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

MoveItVisualTools를 초기화하기 전에 ROS 노드에서 실행자(executor)가 작동 중이어야 합니다.
이는 MoveItVisualTools가 ROS 서비스 및 토픽과 상호 작용하는 방식 때문에 필요합니다.

.. code-block:: C++

  #include <thread>  // <---- add this to the set of includes at the top

    ...

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Create the MoveIt MoveGroup Interface
    ...

    // Shutdown ROS
    rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
    spinner.join();  // <--- Join the thread before exiting
    return 0;
  }

이러한 변경 사항을 하나씩 적용한 후, workspace을 다시 빌드하여 문법 오류가 없는지 확인하세요.

3 Create and Initialize MoveItVisualTools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

다음으로 MoveGroupInterface를 생성한 후 MoveItVisualTools를 생성하고 초기화합니다.

.. code-block:: C++

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "panda_arm");

    // Construct and initialize MoveItVisualTools
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        node, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
        move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

ROS node, 로봇의 base link, 사용할 marker topic (나중에 자세히 설명), robot model (move_group_interface에서 얻음)을 constructor(생성자)에 넘겨줍니다.
다음으로 모든 마커를 삭제하는 호출을 합니다. 이렇게 하면 이전 실행에서 남은 RViz의 렌더링된 상태가 모두 지워집니다.마지막으로 원격 제어기를 로드합니다.
원격 제어기는 매우 간단한 plugin으로, RViz에서 버튼을 사용하여 프로그램과 상호 작용할 수 있게 해줍니다.

4 Write closures for visualizations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

우리가 생성 및 초기화를 마치고 나서 이제 현재 scope의 변수에 액세스할 수 있는 closures(함수 객체) 몇 개를 생성하는데, 이것은 나중에 프로그램에서 RViz로 시각화를 렌더링하는 데 도움을 줄 수 있습니다.

.. code-block:: C++

    // Create a closures for visualization
    auto const draw_title = [&moveit_visual_tools](auto text) {
      auto const text_pose = [] {
        auto msg = Eigen::Isometry3d::Identity();
        msg.translation().z() = 1.0;
        return msg;
      }();
      moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                      rviz_visual_tools::XLARGE);
    };
    auto const prompt = [&moveit_visual_tools](auto text) {
      moveit_visual_tools.prompt(text);
    };
    auto const draw_trajectory_tool_path =
        [&moveit_visual_tools,
         jmg = move_group_interface.getRobotModel()->getJointModelGroup(
             "panda_arm")](auto const trajectory) {
          moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
        };
3개의 closures의 각각은 moveit_visual_tools 객체를 참조(reference)로 캡처하고, 마지막 closures는 planning에 사용하고 있는 joint model group 객체에 대한 포인터를 캡처합니다.
각 closure는 ``moveit_visual_tools`` 객체의 함수를 호출하여 RViz에서 무언가를 변경합니다.
첫 번째 closure인 ``draw_title``는 로봇 base에서 1미터 위에 텍스트를 추가합니다.
이는 프로그램의 상태 보여주는 유용한 방법입니다.
두 번째 closure는 ``prompt``라는 함수를 호출합니다. 이 함수는 사용자가 RViz에서 ``next`` 버튼을 누를 때까지 프로그램을 차단합니다.
이는 디버깅할 때 프로그램 단계별로 수행하는 데 도움이 됩니다.
마지막 closure는 계획한 궤적의 도구 경로를 그립니다.
이는 도구의 관점에서 계획한 궤적을 이해하는데 도움이 됩니다.

"왜 이런 lambdas 함수를 만들까?" 하고 의문이 드실 수도 있지만, 코드를 더 읽기 쉽고 이해하기 쉽게 만들기 위한 것입니다.
소프트웨어를 작성할 때 기능을 명명된 함수로 분할하면 쉽게 재사용 및 개별적으로 테스트할 수 있으므로 종종 유용합니다. 다음 섹션에서 생성한 함수를 어떻게 사용하는지 살펴볼 것입니다.

5 Visualize the steps of your program
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

이제 프로그램 중간에 코드를 보강하겠습니다.
계획 및 실행을 위한 코드를 업데이트하여 다음과 같은 새로운 기능을 포함시키세요:

.. code-block:: C++

    // Set a target Pose
    auto const target_pose = [] {
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 1.0;
      msg.position.x = 0.28;
      msg.position.y = -0.2;
      msg.position.z = 0.5;
      return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);

    // Create a plan to that target pose
    prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
    draw_title("Planning");
    moveit_visual_tools.trigger();
    auto const [success, plan] = [&move_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success) {
      draw_trajectory_tool_path(plan.trajectory_);
      moveit_visual_tools.trigger();
      prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
      draw_title("Executing");
      moveit_visual_tools.trigger();
      move_group_interface.execute(plan);
    } else {
      draw_title("Planning Failed!");
      moveit_visual_tools.trigger();
      RCLCPP_ERROR(logger, "Planing failed!");
    }

RViz에서 렌더링된 어떤 것을 변경하기 위한 각 호출 이후에 반드시 ``moveit_visual_tools`` 오브젝트에서 ``trigger``라는 메서드를 호출해야 한다는 것을 금방 알게 될 것입니다.
그 이유는 RViz에 전송된 메시지가 일괄 처리되어 trigger를 호출할 때 marker topics의 대역폭을 줄이기 위해 한 번에 전송되기 때문입니다.

마지막으로, 모든 코드 추가가 올바른지 확인하기 위해 프로젝트를 다시 빌드하십시오.

.. code-block:: bash

  cd ~/ws_moveit2
  source /opt/ros/{DISTRO}/setup.bash
  colcon build --mixin debug

6 Enable visualizations in RViz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

새로운 터미널을 열고 workspace를 source한 다음, 데모 launch 파일을 구동시켜서 RViz를 시작하세요.

.. code-block:: bash

  cd ~/ws_moveit2
  source install/setup.bash
  ros2 launch moveit2_tutorials demo.launch.py

"Displays" 탭에서 "MotionPlanning"의 선택을 해제하여 숨깁니다.
다음 단계에서는 "MotionPlanning" 플러그인을 사용하지 않을 것입니다.

.. image:: uncheck_motion_planning.png

.. image:: unchecked_motion_planning.png

우리가 프로그램에 추가했던 프롬프트와 상호 작용하기 위한 버튼을 추가하려면 "Panels/Add New Panel" 메뉴를 사용하여 대화 상자를 열어야 합니다.:

.. image:: panel_menu.png

그런 다음 ``RvizVisualToolsGui``를 선택하고 OK를 클릭하십시오. 이렇게 하면 나중에 사용할 ``Next`` 버튼이 있는 새로운 패널이 왼쪽 하단에 생성됩니다.

.. image:: add_rviz_tools_gui.png

.. image:: next_button.png

마지막으로 추가한 시각화를 렌더링하기 위해 ``Marker Array``을 추가해야 합니다. "Displays" 패널의 "Add" 버튼을 클릭하십시오.

.. image:: add_button.png

``Marker Array`` 를 선택하고 ``OK``를 클릭하세요.

.. image:: marker_array.png

Displays 패널내에 아이템의 목록 하단으로 스크롤하고 새로 만든 Marker Array가 사용하는 토픽을 ``/rviz_visual_tools``로 수정하세요.

.. image:: marker_array_topic.png

이제 시각화와 함께 프로그램을 실행할 수 있게 준비되었습니다.

7 Run the Program
^^^^^^^^^^^^^^^^^

새 터미널을 열고 workspace로 이동한 후, workspace를 source 및 ``hello_moveit`` 실행:

.. code:: bash

  cd ~/ws_moveit2
  source install/setup.bash
  ros2 run hello_moveit hello_moveit

다음과 같은 로그가 표시되면서 프로그램이 일시 정지되었음을 확인할 수 있습니다.:

.. code::

  [INFO] [1652822889.492940200] [hello_moveit.remote_control]: Waiting to continue: Press 'Next' in the RvizVisualToolsGui window to plan

RViz에서 ``Next`` 버튼을 클릭하면 응용 프로그램이 진행되는 것을 확인할 수 있습니다.

.. image:: planning.png

next 버튼을 클릭한 후에는 응용 프로그램이 계획을 수행하고 로봇 위에 제목을 추가하며 도구 경로를 나타내는 선을 그렸음을 확인할 수 있습니다.
계속하려면 다시 ``Next`` 버튼을 누르면 로봇이 계획을 실행하는 것을 볼 수 있습니다.

.. image:: executing.png


Summary
-------

MoveIt을 사용하여 작성한 프로그램을 RViz의 Gui와 상호 작용하도록 확장하여, 버튼을 사용하여 프로그램 단계를 거치고 로봇 위에 텍스트를 렌더링하고 계획한 도구 경로를 표시할 수 있게 되었습니다.

Further Reading
---------------

- MoveItVisualTools는 로봇 동작 시각화를 위한 더 많은 유용한 기능을 제공합니다. 추가 정보는 `You can read more about it here <https://github.com/ros-planning/moveit_visual_tools/tree/ros2>`_
- MoveItVisualTools 사용과 관련된 더 많은 예시는 :doc:`MoveItCpp Tutorial </doc/examples/moveit_cpp/moveitcpp_tutorial>`
- :codedir:`전체 코드 hello_moveit.cpp source<tutorials/visualizing_in_rviz/hello_moveit.cpp>`.

Next Step
---------

다음 튜터리얼인 :doc:`Planning Around Objects </doc/tutorials/planning_around_objects/planning_around_objects>`에서는 여기에서 우리가 작성한 프로그램을 확장하여 충돌 환경에 추가하고 이러한 변경 사항을 반영한 로봇 계획을 수행하는 방법을 다룰 것입니다.
