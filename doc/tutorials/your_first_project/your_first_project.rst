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

코드의 첫 부분에 포함된 헤더는 표준 C++ header와 나중에 사용할 ROS 및 MoveIt 헤더입니다.

이후에 rclcpp를 초기화하고 Node를 생성합니다.

.. code-block:: C++

  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

첫번째 인자는 문자열로서 ROS가 고유한 노드를 만들기 위해 사용된다.
두번째 인자는 MoveIt에서 ROS 파라미터를 사용하는 방식 때문에 필요하다.

마지막으로 ROS를 종료하는 코드이다.

3 MoveGroupInterface를 사용하는 Plan and Execute
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``Next step goes here``라고 적힌 주석 대신에 다음 코드를 추가하십시오.:

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

3.1 Build 와 Run
~~~~~~~~~~~~~~~~~

전과 같이 코드를 실행하기 전에 코드를 빌드해야 합니다.

워크스페이스 디렉토리 ``ws_moveit2`` 에서 다음 명령을 실행하십시오.:

.. code-block:: bash

  colcon build --mixin debug

이 명령이 성공하면, 이전 튜토리얼에서 사용한 데모 런치 파일을 재사용하여 RViz와 MoveGroup 노드를 시작해야 합니다.
별도의 터미널에서 워크스페이스를 source하고 다음을 실행하십시오.:

.. code-block:: bash

  ros2 launch moveit2_tutorials demo.launch.py

``Displays`` 창에서 ``MotionPlanning/Planning Request`` 아래의 ``Query Goal State`` 상자를 선택 해제하십시오.

.. image:: rviz_1.png
   :width: 300px

세 번째 터미널에서 워크스페이스를 source하고 프로그램을 실행하십시오.

.. code-block:: bash

  ros2 run hello_moveit hello_moveit

이것은 RViz내에 로봇이 이 pose에 있도록 이동하고 종료해야 합니다.:

.. image:: rviz_2.png
   :width: 300px

만약 먼저 데모 런치 파일을 실행하지 않고 ``hello_moveit`` 노드를 실행했다면, 10초를 기다린 후 다음과 같은 오류가 출력되고 종료됩니다.

.. code-block:: bash

  [ERROR] [1644181704.350825487] [hello_moveit]: Could not find parameter robot_description and did not receive robot_description via std_msgs::msg::String subscription within 10.000000 seconds.

이것은 ``demo.launch.py`` 런치가 로봇 서술을 제공하는 ``MoveGroup`` 노드를 시작시키기 때문에 발생합니다.
``MoveGroupInterface``이 생성될 때 로봇 서술과 함께 topic을 publish하는 노드를 찾습니다.
10초 내에 찾지 못하면 이 오류를 출력하고 프로그램을 종료합니다.

3.2 코드 살펴보기
~~~~~~~~~~~~~~~~~~~~

첫번째로  ``MoveGroupInterface``를 생성합니다. 이 객체는 move_group과 상호작용할 수 있도록 해줍니다. 이것은 우리가 Trajectories를 plan and execute할 수 있게 해줍니다.
이 프로그램에서 생성하는 유일한 mutable 객체임을 주목하십시오.
주목해야할 또 다른 것은 여기서 우리가 생성한 ``MoveGroupInterface`` 객체에 대한 두번째 인터페이스입니다: ``"panda_arm"``.
이것은 로봇 서술에서 정의된 관절 그룹으로, 이 ``MoveGroupInterface``를 사용하여 작동할 것입니다.

.. code-block:: C++

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

그런 다음, 목표 포즈와 계획을 설정합니다. 목표 pose는 오직 ``setPoseTarget``을 통해서만 설정된다는 점에 유의하십시오. 시작 포즈는 현재 로봇의 joint state publisher에 의해 publish된 위치이며, ``MoveGroupInterface::setStartState*`` 함수군을 사용하여 변경할 수 있지만 하지만 여기서는 다루지 않습니다.

다음 섹션에서 유의해야 할 또 다른 사항은 ``target_pose`` 및 planning의 메시지 타입을 구성하기 위해 lambdas를 사용하는 것입니다.
이것은 현대 C++ 코드베이스에서 더 선언적인 스타일로 작성할 수 있도록 하는 패턴입니다.
이 패턴에 대한 자세한 내용은 이 튜터리얼의 끝에 몇개의 링크를 제공합니다.

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


마지막으로, 계획이 성공적으로 수립되었으면 계획을 실행하고, 그렇지 않으면 다음과 같은 오류를 기록합니다.

.. code-block:: C++

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

Summary
-------

* ROS 2 패키지를 만들고 MoveIt을 사용하는 첫 번째 프로그램을 작성했습니다.
* MoveGroupInterface를 사용하여 이동 계획 및 실행 방법에 대해 배웠습니다.
* :codedir:`이 튜터리얼<tutorials/your_first_project/hello_moveit.cpp> 끝에는 전체 hello_moveit.cpp 소스 코드 사본이 있습니다.`.

Further Reading
---------------

- 람다 함수를 사용하여 객체를 상수로 초기화할 수 있었습니다. 이 기술은 IIFE라고 불립니다. `이 패턴과 관련된 참고는 C++ Stories <https://www.cppstories.com/2016/11/iife-for-complex-initialization/>`_
- 또한 가능한 모든 것을 const로 선언했습니다. const의 유용성에 대해서는 `C++ Stories:  <https://www.cppstories.com/2016/12/please-declare-your-variables-as-const/>`_

Next Step
---------

다음 튜토리얼 :doc:`Visualizing in RViz </doc/tutorials/visualizing_in_rviz/visualizing_in_rviz>`에서 여기서 작성한 프로그램을 확장하여 MoveIt이 무엇을 하는지 이해하기 쉽게 하는 visual marker를 만들 것입니다.
