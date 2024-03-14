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
- MoveIt 2 튜터리얼
- `ROS2 joy <https://index.ros.org/p/joy/>`_

단계
------

1. MoveIt 2 워크스페이스를 빌드합니다.

  먼저 moveit2 워크스페이스의 루트 디렉토리로 이동하는 ``cd`` 명령을 사용하십시오. (:doc:`Getting Started </doc/tutorials/getting_started/getting_started>` 튜토리얼을 따랐다면 이 디렉토리는 ``~/ws_moveit/`` 입니다.)

  다음으로 ``colcon build`` 실행합니다. Then, run ``colcon build``.

2. 게임패드를 연결합니다.
3. install 스크립트를 source하고 ``moveit_servo`` 예제 파일을 실행합니다.

  Run ``source install/setup.bash``, then ``ros2 launch moveit_servo servo_example.launch.py``

4. 팔을 주위로 움직여봅니다. 아래 이미지를 참고하세요.

  .. image:: xboxcontroller.png
    :width: 600px

drawio 문서는 `here <https://drive.google.com/file/d/1Hr3ZLvkYo0y0fA3Qb1Nk_y7wag4UO8Al/view?usp=sharing>`__ 를 참고하세요.

Explanation
-----------

이 섹션은 launch 파일과 게임패드 입력을 모션 명령어로 변환하는 node에 대해 설명합니다.

Launch 파일
^^^^^^^^^^^

이 예제를 launch하는 파일은 ``ws_moveit2/src/moveit2/moveit_ros/moveit_servo/launch/servo_example.launch.py`` 입니다.

이 launch 파일은 panda arm planning에 필요한 모든 것을 실행하며, 또한 ``joy`` node와 아래 설명하는 ``JoyToServoPub`` node도 launch 시킵니다.

가장 중요한 부분은  joy node와 ``JoyToServoPub`` node를 launch시키는 코드 부분입니다.
이 둘은 모두 ``ComposableNode``\s 로 생성됩니다. ``ComposableNode``\s 에 대한 자세한 내용은 `here <https://roscon.ros.org/2019/talks/roscon2019_composablenodes.pdf>`__ and `here <https://medium.com/@waleedmansoor/understanding-ros-nodelets-c43a11c8169e>`__ 링크에서 찾을 수 있습니다.

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

게임패드 입력을 모션 명령어로 변환하는 node는 다음과 같습니다. ``ws_moveit2/src/moveit2/moveit_ros/moveit_servo/src/teleop_demo/joystick_servo_example.cpp``

이 node는 joy node (게임패드 상태 메시지를 publish)를 subscribe합니다. 이 node는 ``TwistStamped`` messages, ``JointJog`` messages, ``PlanningScene`` messages를 게시합니다.

``PlanningScene`` 메시지는 JoyToServoPub이 처음 생성될 때 한 번만 publish됩니다. 간단히 말해 몇 가지 장애물을 계획 scene에 추가합니다.

``JointJog`` 와 ``TwistStamped`` 메시지의 차이점은 역 운동학 솔버(inverse kinematic solver)가 ``TwistStamped`` 메시지에 의해 정의된 end-effector 동작을 달성하기 위해 조인트를 이동시키는 반면
``JointJog`` 메시지는 개별 조인트들을 직접 이동시킨다는 점입니다.

``joyCB`` 함수는 ``joy`` topic으로 메시지가 publish될 때 호출되고, 게임 패드의 버튼 누름을 팔에 대한 명령으로 변환합니다.
입력에 의해 ``JointJog`` 와 ``TwistStamped`` 메시지가 모두 publish되는 경우 ``JointJog`` 메시지만 게시됩니다.
