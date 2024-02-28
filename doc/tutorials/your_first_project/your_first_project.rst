첫번째 C++ MoveIt 프로젝트
=============================

이 튜터리얼은 MoveIt를 사용하여 첫번째 C++ 애플리케이션을 작성하는 방법을 안내합니다.

사전 요구 사항
-------------

:doc:`시작하기 튜토리얼 </doc/tutorials/getting_started/getting_started>`에서 설명한 단계를 완료했는지 확인하십시오.

이 튜터리얼은 여러분이 ROS 2의 기본을 이해하고 있다고 가정합니다.
이를 위해 `공식 ROS 2 튜토리얼 <https://docs.ros.org/en/{DISTRO}/Tutorials.html>`_을 "Writing a simple publisher and Subscriber (C++)"까지 완료하십시오.

Steps
-----

1 패키지 생성하기
^^^^^^^^^^^^^^^^^^

터미널을 열고 `ROS 2 설치를 source <https://docs.ros.org/en/{DISTRO}/Tutorials/Configuring-ROS2-Environment.html>`_하여 ``ros2`` 명령이 작동하도록 합니다.

:doc:`Getting Started Tutorial </doc/tutorials/getting_started/getting_started>`에서 생성한 ``ws_moveit2`` 디렉토리로 이동합니다.

``src`` 디렉토리로 이동하십시오. 이 디렉토리에 소스 코드를 넣을 것입니다.

ROS 2 명령줄 도구를 사용하여 새 패키지를 만듭니다.:

.. code-block:: bash

  ros2 pkg create \
   --build-type ament_cmake \
   --dependencies moveit_ros_planning_interface rclcpp \
   --node-name hello_moveit hello_moveit

이 명령의 출력으로 새 디렉토리에 몇 개의 파일을 만들었다는 것을 보여줍니다.

``moveit_ros_planning_interface``와 ``rclcpp``를 의존성으로 추가했다는 것에 주목하십시오.
이렇게 하면 이 두 패키지에 의존할 수 있도록 ``package.xml`` 및 ``CMakeLists.txt`` 파일에 필요한 변경 사항이 생성됩니다.

``ws_moveit2/src/hello_moveit/src/hello_moveit.cpp``에 생성된 새 소스 파일을 좋아하는 편집기로 엽니다.

2 ROS Node와 Executor를 생성하기
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

코드의 첫번째 블록은 약간의 boilerplate이지만 ROS 2 튜토리얼에서 이것을 보는 것에 익숙해져 있어야 합니다.

.. code-block:: C++

  #include <memory>

  #include <rclcpp/rclcpp.hpp>
  #include <moveit/move_group_interface/move_group_interface.h>

  int main(int argc, char * argv[])
  {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    // Next step goes here

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
  }

2.1 Build와 Run
~~~~~~~~~~~~~~~~~

우리가 작성한 코드를 빌드하고 실행하여 모든 것이 올바른지 확인해야 합니다.

먼저, ``ws_moveit2`` 디렉토리로 돌아가서 다음 명령을 실행하십시오.:

.. code-block:: bash

  colcon build --mixin debug

이 명령이 성공하면 **새 터미널을 열고** 작업 공간 환경 스크립트를 그 새 터미널에서 source하여 프로그램을 실행할 수 있습니다.

.. code-block:: bash

  cd ~/ws_moveit2
  source install/setup.bash

이제 프로그램을 실행하고 출력을 확인하십시오.

.. code-block:: bash

  ros2 run hello_moveit hello_moveit

프로그램은 오류 없이 실행되고 종료해야 합니다.

2.2 코드 살펴보기
~~~~~~~~~~~~~~~~~~~~

The headers included at the top are just some standard C++ headers and the header for ROS and MoveIt that we will use later.

After that, we have the normal call to initialize rclcpp, and then we create our Node.

.. code-block:: C++

  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

The first argument is a string that ROS will use to make a unique node.
The second is needed for MoveIt because of how we use ROS Parameters.

Lastly, we have the code to shutdown ROS.

3 Plan and Execute using MoveGroupInterface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In place of the comment that says "Next step goes here," add this code:

.. code-block:: C++

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

3.1 Build and Run
~~~~~~~~~~~~~~~~~

Just like before, we need to build the code before we can run it.

In the workspace directory, ``ws_moveit2``, run this command:

.. code-block:: bash

  colcon build --mixin debug

After this succeeds, we need to re-use the demo launch file from the previous tutorial to start RViz and the MoveGroup node.
In a separate terminal, source the workspace and then execute this:

.. code-block:: bash

  ros2 launch moveit2_tutorials demo.launch.py

Then in the ``Displays`` window under ``MotionPlanning/Planning Request``, uncheck the box ``Query Goal State``.

.. image:: rviz_1.png
   :width: 300px

In a third terminal, source the workspace and run your program.

.. code-block:: bash

  ros2 run hello_moveit hello_moveit

This should cause the robot in RViz to move and end up in this pose:

.. image:: rviz_2.png
   :width: 300px

Note that if you ran the node ``hello_moveit`` without launching the demo launch file first, it will wait for 10 seconds and then print this error and exit.

.. code-block:: bash

  [ERROR] [1644181704.350825487] [hello_moveit]: Could not find parameter robot_description and did not receive robot_description via std_msgs::msg::String subscription within 10.000000 seconds.

This is because the ``demo.launch.py`` launch is starting the ``MoveGroup`` node that provides the robot description.
When ``MoveGroupInterface`` is constructed, it looks for a node publishing a topic with the robot description.
If it fails to find that within 10 seconds, it prints this error and terminates the program.

3.2 Examine the code
~~~~~~~~~~~~~~~~~~~~

The first thing we do is create the MoveGroupInterface. This object will be used to interact with move_group, which allows us to plan and execute trajectories.
Note that this is the only mutable object that we create in this program.
Another thing to take note of is the second interface to the ``MoveGroupInterface`` object we are creating here: ``"panda_arm"``.
That is the group of joints as defined in the robot description that we are going to operate on with this ``MoveGroupInterface``.

.. code-block:: C++

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

Then, we set our target pose and plan. Note that only the target pose is set (via ``setPoseTarget``. The starting pose is implicitly the position published by joint state publisher, which could be changed using the ``MoveGroupInterface::setStartState*`` family of functions (but is not in this tutorial).

One more thing to note about this next section is the use of lambdas for constructing the message type ``target_pose`` and planning.
This is a pattern you'll find in modern C++ codebases that enables writing in a more declarative style.
For more information about this pattern there is a couple of links at the end of this tutorial.

.. code-block:: C++

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

Finally, we execute our plan if planning was successful, otherwise we log an error:

.. code-block:: C++

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

Summary
-------

* You created a ROS 2 package and wrote your first program using MoveIt.
* You learned about using the MoveGroupInterface to plan and execute moves.
* :codedir:`Here is a copy of the full hello_moveit.cpp source at the end of this tutorial<tutorials/your_first_project/hello_moveit.cpp>`.

Further Reading
---------------

- We used lambdas to be able to initialize objects as constants. This is known as a technique called IIFE.  `Read more about this pattern from C++ Stories <https://www.cppstories.com/2016/11/iife-for-complex-initialization/>`_.
- We also declared everything we could as const.  `Read more about the usefulness of const here <https://www.cppstories.com/2016/12/please-declare-your-variables-as-const/>`_.

Next Step
---------

In the next tutorial :doc:`Visualizing in RViz </doc/tutorials/visualizing_in_rviz/visualizing_in_rviz>`, you will expand on the program you built here to create visual markers that make it easier to understand what MoveIt is doing.
