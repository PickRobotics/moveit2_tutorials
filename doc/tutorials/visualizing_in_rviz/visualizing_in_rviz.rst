RViz내에서 시각화하기
======================

이 튜터리얼은 RViz에서 시각화를 렌더링하여 MoveIt 어플리케이션이 수행하는 작업을 더 쉽게 이해하는 것을 돕는 도구를 소개합니다.

사전 요구사항
---------------

:doc:`Your First Project </doc/tutorials/your_first_project/your_first_project>` 을 모두 완료하고 진행하세요.
이번 프로젝트는 이전 튜토리얼의 ``hello_moveit`` 프로젝트의 중단된 부분에서 시작한다고 가정합니다.

Steps
-----

1 moveit_visual_tools에 의존성 추가하기
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``hello_moveit`` 프로젝트의 ``package.xml`` 파일에서 다른 ``<depend>`` 구문 뒤에 다음 줄을 추가하십시오.:

.. code-block:: xml

  <depend>moveit_visual_tools</depend>

다음으로 ``CMakeLists.txt`` 파일에서 ``find_package`` 구문의 섹션에 다음 줄을 추가합니다.:

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

의존 관계가 제대로 추가되었는지 확인하려면 소스 파일 ``hello_moveit.cpp`` 에 다음과 같은 헤더 파일 포함을 추가하십시오:

.. code-block:: C++

  #include <moveit_visual_tools/moveit_visual_tools.h>

모든 작업이 정상적으로 이루어졌는지 테스트하기 위해서 workspace 디렉토리에서 터미널을 열고 (opt 디렉토리에 설치된 ROS를 source하는 것을 잊지 마세요) colcon 명령으로 빌드하세요:

.. code-block:: bash

  cd ~/ws_moveit
  colcon build --mixin debug

2 ROS executor 생성 및 thread로 해당 node를 spin하기
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

MoveItVisualTools를 초기화하기 전에 ROS 노드에서 작동 중인 실행자(executor)가 있어야 합니다.
이는 MoveItVisualTools가 ROS service 및 topic과 상호 작용하는 방식 때문에 필요합니다. 먼저 threading 라이브러리를 includes에 추가하세요.

.. code-block:: C++

  #include <thread>  // <---- add this to the set of includes at the top

loggers를 생성하고 이름을 지정함으로써 프로그램 로그를 구성할 수 있습니다.

  .. code-block:: C++

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

다음으로 MoveIt MoveGroup Interface를 생성하기 전에 executor를 추가하세요.

.. code-block:: C++

    // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Create the MoveIt MoveGroup Interface

  ...

마지막으로 종료하기 전에 thread를 join해야 합니다.

.. code-block:: C++

    // Shutdown ROS
    rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
    spinner.join();  // <--- Join the thread before exiting
    return 0;

이러한 변경 사항을 하나씩 적용한 후, workspace을 다시 빌드하여 문법 오류가 없는지 확인하세요.

3 MoveItVisualTools 생성 및 초기화
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

다음으로 MoveGroupInterface를 생성한 후 MoveItVisualTools를 생성하고 초기화합니다.

.. code-block:: C++

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "manipulator");

    // Construct and initialize MoveItVisualTools
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
        move_group_interface.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

생성자에게 다음 내용을 전달합니다. : ROS node, 로봇의 base link, 사용할 marker topic (나중에 자세히 설명), robot model (move_group_interface에서 얻음)
다음으로 모든 마커를 삭제하는 호출을 합니다. 이렇게 하면 이전 실행에서 남은 RViz의 렌더링된 상태가 모두 지워집니다.
마지막으로 원격 제어기를 로드합니다.
원격 제어기는 매우 간단한 plugin으로, RViz에서 버튼을 사용하여 프로그램과 상호 작용할 수 있게 해줍니다.

4 시각화를 위해 closures 작성하기
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

우리가 생성 및 초기화를 마치고 나서 이제 현재 scope의 변수에 액세스할 수 있는 closures(함수 객체) 몇 개를 생성하는데, 이것은 나중에 프로그램에서 RViz로 시각화를 렌더링하는 데 도움을 줄 수 있습니다.

.. code-block:: C++

    // Create closures for visualization
    auto const draw_title = [&moveit_visual_tools](auto text) {
      auto const text_pose = [] {
        auto msg = Eigen::Isometry3d::Identity();
        msg.translation().z() = 1.0;  // Place text 1m above the base link
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
             "manipulator")](auto const trajectory) {
          moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
        };

각각 3개의 람다는 참조에 의해 ``moveit_visual_tools`` 를 캡처하고, 마지막 하나는 우리가 계획하고 있는 조인트 모델 그룹 객체에 대한 포인터를 캡처합니다.
이러한 각각은 RViz에서 무언가를 변경하는 ``moveit_visual_tools`` 의 함수를 호출합니다.

* 첫 번째 함수인 ``draw_title`` 은 로봇의 base에서 1미터 위에 텍스트를 추가합니다. 이것은 프로그램의 상태를 하이 레벨에서 보여주는 유용한 방법입니다.
* 두 번째 함수는 ``prompt`` 라는 함수를 호출합니다. 이 함수는 사용자가 RViz에서 ``next`` 버튼을 누를 때까지 프로그램을 차단합니다. 이는 디버깅할 때 프로그램 단계를 거치는 데 도움이 됩니다.
* 마지막 함수는 계획된 궤적의 도구 경로를 그립니다. 이는 종종 도구의 관점에서 계획된 궤적을 이해하는 데 도움이 됩니다.

왜 이런 lambdas를 만들까 궁금할 수 있습니다. 그 이유는 단순히 코드를 나중에 더 쉽게 읽고 이해할 수 있도록 만들기 위해서입니다.
소프트웨어를 작성할 때, 기능을 쉽게 재사용 및 개별적으로 테스트할 수 있는 이름이 지정된 함수로 분할하는 것이 종종 도움이 됩니다.
다음 섹션에서는 우리가 만든 이 함수들을 어떻게 사용하는지 볼 수 있습니다.

5 프로그램의 단계들을 시각화하기
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

이제 프로그램 중간에 있는 코드를 수정합니다.
계획 및 실행 코드를 업데이트하여 이러한 새로운 기능을 포함시킵니다. :

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
      draw_trajectory_tool_path(plan.trajectory);
      moveit_visual_tools.trigger();
      prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
      draw_title("Executing");
      moveit_visual_tools.trigger();
      move_group_interface.execute(plan);
    } else {
      draw_title("Planning Failed!");
      moveit_visual_tools.trigger();
      RCLCPP_ERROR(logger, "Planning failed!");
    }

RViz에서 렌더링된 항목을 변경하기 위해 각 호출 후 ``moveit_visual_tools`` 에서 ``trigger`` 라는 메서드를 호출해야 한다는 점을 빠르게 알 수 있습니다.
이유는 RViz에 전송되는 메시지가 일괄 처리되고 ``trigger`` 를 호출할 때 marker topic의 대역폭을 줄이기 위해 전송되기 때문입니다.

마지막으로 모든 코드 추가가 올바른지 확인하기 위해 프로젝트를 다시 빌드하십시오.

.. code-block:: bash

  cd ~/ws_moveit
  source /opt/ros/rolling/setup.bash
  colcon build --mixin debug

6 RViz에서 시각화 활성화
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

새로운 터미널을 열고 워크스페이스를 source한 다음, RViz를 여는 데모 launch 파일을 시작하십시오.

.. code-block:: bash

  cd ~/ws_moveit
  source install/setup.bash
  ros2 launch moveit2_tutorials demo.launch.py

"Displays" 탭에서 "MotionPlanning" 체크를 해제하여 감춰지게 합니다.
이번 단계에서는 "MotionPlanning" 플러그인을 사용하지 않을 것입니다.

.. image:: uncheck_motion_planning.png

.. image:: unchecked_motion_planning.png

프로그램에 추가한 프롬프트와 상호 작용하기 위한 버튼을 추가하려면 "Panels/Add New Panel" 메뉴를 사용하여 대화 상자를 열어야 합니다.:

.. image:: panel_menu.png

``RvizVisualToolsGui`` 선택 후 OK를 클릭합니다.
이 작업은 왼쪽 하단에 나중에 사용할 "Next" 버튼이 있는 새로운 패널을 생성합니다.

.. image:: add_rviz_tools_gui.png

.. image:: next_button.png

마지막으로, 추가한 시각화를 렌더링하기 위해 ``Marker Array`` 을 추가해야 합니다. "Displays" 패널의 "Add" 버튼을 클릭하십시오.

.. image:: add_button.png

``Marker Array`` 선택 후 ``OK`` 를 클릭합니다.

.. image:: marker_array.png

Displays 패널의 항목 하단으로 스크롤하여 새 Marker Array가 사용하는 topic을 ``/rviz_visual_tools`` 로 편집합니다.

.. image:: marker_array_topic.png

새로운 프로그램을 시각화 기능과 함께 실행할 준비가 되었습니다.

7 프로그램 실행하기
^^^^^^^^^^^^^^^^^^^^^

새로운 터미널에서 작업 공간으로 이동하고, 작업 공간을 소싱한 후 ``hello_moveit`` 실행합니다.:

.. code:: bash

  cd ~/ws_moveit
  source install/setup.bash
  ros2 run hello_moveit hello_moveit

프로그램이 다음과 같은 로그와 함께 중지되었음을 확인할 수 있습니다.:

.. code::

  [INFO] [1652822889.492940200] [hello_moveit.remote_control]: Waiting to continue: Press 'Next' in the RvizVisualToolsGui window to plan

RViz에서 ``Next`` 버튼을 클릭하여 응용 프로그램 진행을 확인합니다.

.. image:: planning.png

다음 버튼을 클릭한 후, 응용 프로그램이 계획을 수행하고 로봇 위에 제목을 추가하며 도구 경로를 나타내는 선을 그렸음을 확인할 수 있습니다.
계속하려면 다시 ``Next`` 를 눌러 로봇이 계획을 실행하도록 지시합니다.

.. image:: executing.png


요약
-------

MoveIt으로 작성한 프로그램을 RViz의 Gui와 연동되도록 확장했습니다. 버튼을 사용하여 프로그램 단계별로 진행하고 로봇 위에 텍스트를 렌더링하며 계획한 도구 경로를 표시할 수 있습니다.

추가 읽을꺼리
---------------

- MoveItVisualTools는 로봇 모션 시각화를 위한 더 많은 유용한 기능을 제공합니다. `여기에서 자세한 내용을 읽을 수 있습니다. <https://github.com/ros-planning/moveit_visual_tools/tree/ros2>`_.
- 또한 :doc:`MoveItCpp Tutorial </doc/examples/moveit_cpp/moveitcpp_tutorial>` 에서도 ``MoveItVisualTools`` 사용에 대한 더 많은 예제를 찾을 수 있습니다.
- :codedir:`여기는 전체 hello_moveit.cpp 소스 복사본 <tutorials/visualizing_in_rviz/hello_moveit.cpp>` 입니다.

다음 단계
---------

다음 튜터리얼인 :doc:`Planning Around Objects </doc/tutorials/planning_around_objects/planning_around_objects>` 에서는 여기서 구축한 프로그램을 확장하여 충돌 환경에 추가하고 이러한 변경 사항을 반영한 로봇 계획을 살펴보도록 합시다.
