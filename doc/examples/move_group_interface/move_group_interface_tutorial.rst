Move Group C++ Interface
==================================
.. image:: move_group_interface_tutorial_start_screen.png
   :width: 700px

MoveIt에서 가장 간단한 사용자 인터페이스는 :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>` 클래스를 통해 제공됩니다. 이 클래스는 사용자가 수행하고자 하는 대부분의 작업, 즉 joint나 pose goals 설정, motion plans 생성, 로봇 이동, 환경에 객체 추가 및 로봇에서 객체 연결/분리 기능을 쉽게 사용할 수 있는 기능을 제공합니다. 이 인터페이스는 ROS topics, services 및 actions을 통해 :moveit_codedir:`MoveGroup<moveit_ros/move_group/src/move_group.cpp>` node와 통신합니다.

move group interface의 강력함을 확인하려면 `YouTube video demo <https://youtu.be/_5siHkFQPBQ>`_ 를 시청하세요!

Getting Started
---------------
만약 아직 완료하지 않았다면,:doc:`Getting Started </doc/tutorials/getting_started/getting_started>` 를 완료하세요.

Running the Code
----------------
2개의 터미널을 여세요. 첫 번째 터미널에서 RViz를 시작하고 모든 로딩이 완료될 때까지 기다리세요: ::

  ros2 launch moveit2_tutorials move_group.launch.py

2번째 터미널에서 launch 파일을 실행하세요: ::

  ros2 launch moveit2_tutorials move_group_interface_tutorial.launch.py

잠시 후, RViz 창이 나타나야 하며 이 페이지 상단의 이미지와 비슷하게 생겼습니다. 각 데모 단계를 진행하려면 화면 하단의 **RvizVisualToolsGui** panel에서 **Next** 버튼을 누르거나 상단의  **Tools** panel에서 **Key Tool**를 선택한 다음 RViz가 선택된 상태에서 키보드의 **0** 키를 누르세요.

Expected Output
---------------
이 튜터리얼 상단의 `YouTube video <https://youtu.be/_5siHkFQPBQ>`_ 참고하여 예상되는 출력을 확인하세요. RViz에서 우리는 다음과 같은 내용을 확인할 수 있어야 합니다:
 1. 로봇 팔이 앞쪽의 pose goal로 이동합니다.
 2. 로봇 팔이 옆쪽의 joint goal로 이동합니다.
 3. 로봇 팔은 end-effector 높이를 유지하면서 새로운 pose goal로 다시 이동합니다.
 4. 로봇 팔이 원하는 카테시안 경로(아래, 오른쪽, 위+왼쪽 삼각형)를 따라 이동합니다.
 5. 로봇 팔이 방해물 없이 간단한 goal 지점으로 이동합니다.
 6. 상자 모양의 객체가 팔 오른쪽 환경에 추가됩니다.
    |B|

 7. 로봇 팔이 상자와 충돌을 피하면서 pose goal로 이동합니다.
 8. 객체가 손목에 부착되고 (색상이 보라색/주황색/초록색으로 변합니다).
 9. 로봇은 부착된 물체와 함께 팔을 pose goal로 이동하며 상자와의 충돌을 피합니다.
 10. 물체가 손목에서 분리됩니다(색상이 다시 초록색으로 변경됩니다).
 11. 객체가 환경에서 제거됩니다.

.. |B| image:: ./move_group_interface_tutorial_robot_with_box.png

The Entire Code
---------------
전체 코드는 MoveIt GitHub 프로젝트의 :codedir:`here in the MoveIt GitHub project<examples/move_group_interface/src/move_group_interface_tutorial.cpp>`에 있습니다. 다음으로 코드를 단계별로 살펴봄으로써 기능을 설명합니다.

.. tutorial-formatter:: ./src/move_group_interface_tutorial.cpp

The Launch File
---------------
전체 launch 파일은 GitHub의 :codedir:`here<examples/move_group_interface/launch/move_group_interface_tutorial.launch.py>`에 있습니다. 이 튜터리얼의 모든 코드는 MoveIt setup의 일부로서 **moveit2_tutorials** package에서 실행할 수 있습니다.


A Note on Setting Tolerances
----------------------------
`MoveGroupInterface's <https://github.com/ros-planning/moveit2/blob/ed844d4b46f70ed6e97d0c1f971ab2b9a45f156d/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h#L293>`_의 *setGoalTolerance()* 및 관련 메서드는 실행 허용 오차가 아니라 **planning** 허용 오차를 설정합니다.

실행 허용 오차를 설정하려면 FollowJointTrajectory controller를 사용하는 경우 *controller.yaml* 파일을 편집해야하고, planner에서 생성된 trajectory 메시지에 직접 추가해야 합니다.
