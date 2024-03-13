Visualizing In RViz
===================

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

Each of the three closures capture ``moveit_visual_tools`` by reference and the last one captures a pointer to the joint model group object we are planning with.
Each of these call a function on ``moveit_visual_tools`` that changes something in RViz.

* The first one, ``draw_title`` adds text one meter above the base of the robot. This is a useful way to show the state of your program from a high level.
* The second one calls a function called ``prompt``. This function blocks your program until the user presses the ``next`` button in RViz. This is helpful for stepping through a program when debugging.
* The last one draws the tool path of a trajectory that we have planned. This is often helpful for understanding a planned trajectory from the perspective of the tool.

You might be asking yourself why we would create lambdas like this, and the reason is simply to make the code that comes later easier to read and understand.
As your write software, it is often helpful to break up your functionality into named functions which can be easily reused and tested on their own.
You will see in the next section how we use these functions we created.

5 Visualize the steps of your program
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Now we'll augment the code in the middle of your program.
Update your code for planning and executing to include these new features:

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

One thing you'll quickly notice is that we have to call a method called ``trigger`` on ``moveit_visual_tools`` after each call to change something rendered in RViz.
The reason for this is that messages sent to RViz are batched up and sent when you call ``trigger`` to reduce bandwidth of the marker topics.

Lastly, build your project again to make sure all the code additions are correct.

.. code-block:: bash

  cd ~/ws_moveit
  source /opt/ros/rolling/setup.bash
  colcon build --mixin debug

6 Enable visualizations in RViz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open a new terminal, source the workspace, and then start the demo launch file that opens RViz.

.. code-block:: bash

  cd ~/ws_moveit
  source install/setup.bash
  ros2 launch moveit2_tutorials demo.launch.py

Uncheck "MotionPlanning" in the "Displays" tab to hide it.
We aren't going to be using the "MotionPlanning" plugin for this next part.

.. image:: uncheck_motion_planning.png

.. image:: unchecked_motion_planning.png

To add the buttons to interact with the prompts we added to our program open the dialog with the "Panels/Add New Panel" menu:

.. image:: panel_menu.png

Then select ``RvizVisualToolsGui`` and click OK.
This will create a new panel on the bottom left with a ``Next`` button we'll use later.

.. image:: add_rviz_tools_gui.png

.. image:: next_button.png

Finally, we need to add a ``Marker Array`` to render the visualizations we've added.
Click on the "Add" Button in the "Displays" panel.

.. image:: add_button.png

Select ``Marker Array`` and click ``OK``.

.. image:: marker_array.png

Scroll to the bottom of the items in the Displays panel and edit the topic that the new Marker Array is using to ``/rviz_visual_tools``.

.. image:: marker_array_topic.png

You are now ready to run your new program with visualizations.

7 Run the Program
^^^^^^^^^^^^^^^^^

In a new terminal, go to the workspace, source the workspace, and run ``hello_moveit``:

.. code:: bash

  cd ~/ws_moveit
  source install/setup.bash
  ros2 run hello_moveit hello_moveit

You'll notice that your program has stopped with a log that looks like this:

.. code::

  [INFO] [1652822889.492940200] [hello_moveit.remote_control]: Waiting to continue: Press 'Next' in the RvizVisualToolsGui window to plan

Click the ``Next`` button in RViz and see your application advance.

.. image:: planning.png

You'll see after you clicked the next button, your application planned, added a title above the robot, and drew a line representing the tool path.
To continue, press ``Next`` again to see your robot execute the plan.

.. image:: executing.png


Summary
-------

You extended the program you wrote with MoveIt to interact with the Gui in RViz, allowing you to step through your program with a button, render some text above the robot, and display the tool path that you planned.

Further Reading
---------------

- MoveItVisualTools has many more useful features for visualizing robot motions. `You can read more about it here <https://github.com/ros-planning/moveit_visual_tools/tree/ros2>`_.
- There are also more examples of using ``MoveItVisualTools`` in :doc:`MoveItCpp Tutorial </doc/examples/moveit_cpp/moveitcpp_tutorial>`.
- :codedir:`Here is a copy of the full hello_moveit.cpp source<tutorials/visualizing_in_rviz/hello_moveit.cpp>`.

Next Step
---------

In the next tutorial :doc:`Planning Around Objects </doc/tutorials/planning_around_objects/planning_around_objects>`, you will expand on the program you built here to add to the collision environment and see the robot plan with these changes.
