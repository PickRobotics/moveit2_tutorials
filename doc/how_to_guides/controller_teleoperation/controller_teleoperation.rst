게임패드로 로봇 팔 원격제어 방법
===============================================

이 가이드는 게임패드를 사용하여 Panda 팔을 움직이는 방법을 소개합니다.

사전준비
-------------
workspace에 필요한 패키지가 설치되어 있고 ROS2 joy를 지원하는 게임패드가 필요합니다.
``joy`` node를 launch한 다음 ``ros2 topic echo /joy`` 를 실행하여 게임패드가 ``joy`` 에서 감지되었는지 확인할 수 있습니다.

요구사항
------------
- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- MoveIt 2 Tutorials
- `ROS2 joy <https://index.ros.org/p/joy/>`_

단계
------

1. MoveIt 2 워크스페이스를 빌드합니다.

  먼저 moveit2 워크스페이스의 루트 디렉토리로 이동하는 ``cd`` 명령을 사용하십시오. (:doc:`Getting Started </doc/tutorials/getting_started/getting_started>` 튜토리얼을 따랐다면 이 디렉토리는 ``~/ws_moveit/`` 입니다.)

  다음으로 ``colcon build`` 실행합니다. Then, run ``colcon build``.

2. Plug in your gamepad.
3. Source the install script and run the ``moveit_servo`` example file.

  Run ``source install/setup.bash``, then ``ros2 launch moveit_servo servo_example.launch.py``

4. Move the arm around, using the below image as a guide.

  .. image:: xboxcontroller.png
    :width: 600px

The drawio document can be seen `here <https://drive.google.com/file/d/1Hr3ZLvkYo0y0fA3Qb1Nk_y7wag4UO8Al/view?usp=sharing>`__.

Explanation
-----------

This section explains the launch file and the node that translates gamepad inputs to motion commands.

Launch File
^^^^^^^^^^^

The file that launches this example is
``ws_moveit2/src/moveit2/moveit_ros/moveit_servo/launch/servo_example.launch.py``

This launch file launches everything needed for the panda arm planning, and also launches the ``joy`` node and the ``JoyToServoPub`` node (which is explained below).

Of primary interest is the section of code that launches the joy and ``JoyToServoPub`` nodes.
They are both created as ``ComposableNode``\s. More information about ``ComposableNode``\s can be found `here <https://roscon.ros.org/2019/talks/roscon2019_composablenodes.pdf>`__ and `here <https://medium.com/@waleedmansoor/understanding-ros-nodelets-c43a11c8169e>`__.

.. code-block:: python

  ComposableNode(
      package="moveit_servo",
      plugin="moveit_servo::JoyToServoPub",
      name="controller_to_servo_node",
  ),
  ComposableNode(
      package="joy",
      plugin="joy::Joy",
      name="joy_node",
  )

JoyToServoPub
^^^^^^^^^^^^^

The node that translates gamepad inputs to motion commands is
``ws_moveit2/src/moveit2/moveit_ros/moveit_servo/src/teleop_demo/joystick_servo_example.cpp``

This node subscribes to the joy node (which publishes messages giving the state of the gamepad). It publishes ``TwistStamped`` messages, ``JointJog`` messages, and ``PlanningScene`` messages.

The ``PlanningScene`` message is only published once, when the JoyToServoPub is first constructed. It simply adds some obstacles into the planning scene.

The difference between the ``JointJog`` and ``TwistStamped`` messages is
that the inverse kinematic solver moves the joints to achieve the end
effector motions defined by the ``TwistStamped`` messages, while the
``JointJog`` messages directly move individual joints.

The ``joyCB`` function is called when a message is published to the ``joy``
topic, and translates the button presses from the gamepad into commands
for the arm. If both ``JointJog`` and ``TwistStamped`` messages would be
published by the inputs, only ``JointJog`` messages are published.
