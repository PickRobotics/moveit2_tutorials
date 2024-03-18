.. _Move Group Interface:

Move Group C++ Interface
==================================
.. image:: move_group_interface_tutorial_start_screen.png
   :width: 700px

MoveIt에서 가장 간단한 사용자 인터페이스는 :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>` 클래스를 통해 이루어집니다.
이 클래스는 사용자가 수행하고자 하는 대부분의 작업, 특히 관절 또는 포즈 목표 설정, 동작 계획 생성, 로봇 이동, 환경에 물체 추가, 로봇에서 물체 부착/분리 등 대부분의 작업을 위한 사용하기 쉬운 기능을 제공합니다.
이 인터페이스는 ROS 주제, 서비스 및 동작을 통해 :moveit_codedir:`MoveGroup<moveit_ros/move_group/src/move_group.cpp>` 노드에 통신합니다.


이 짧은 `유튜브 비디오 데모 <https://youtu.be/_5siHkFQPBQ>`_를 시청하여 이동 그룹 인터페이스의 강력한 기능을 확인하세요!


시작하기
---------------
아직 완료하지 않았다면 :doc:`자막 시작하기 </doc/tutorials/getting_started/getting_started>`의 단계를 완료했는지 확인하세요.

코드 실행하기
----------------
두 개의 셸을 엽니다. 첫 번째 셸에서 RViz를 시작하고 모든 로딩이 완료될 때까지 기다립니다: ::

  ros2 launch moveit2_tutorials move_group.launch.py

두 번째 셸에서 실행 파일을 실행합니다: ::

  ros2 launch moveit2_tutorials move_group_interface_tutorial.launch.py

잠시 후 이 페이지 상단에 있는 것과 비슷한 모양의 RViz 창이 나타납니다. 각 데모 단계를 진행하려면 화면 하단의 **RvizVisualToolsGui** 패널에서 **다음** 버튼을 누르거나 화면 상단의 **도구** 패널에서 **키 도구**를 선택한 다음 RViz에 초점을 맞춘 상태에서 키보드에서 **0**을 누르세요.

예상 출력
---------------
예상 출력은 이 자습서 상단의 `유튜브 동영상 <https://youtu.be/_5siHkFQPBQ>`_을 참조하십시오. RViz에서 다음을 볼 수 있어야 합니다:
 1. 로봇이 팔을 포즈 목표 지점까지 앞쪽으로 이동합니다.
 2. 로봇이 팔을 측면의 조인트 골로 이동합니다.
 3. 로봇이 엔드 이펙터 레벨을 유지하면서 팔을 새 포즈 골로 다시 이동합니다.
 4. 로봇이 원하는 데카르트 경로(아래, 오른쪽, 위+왼쪽 삼각형)를 따라 팔을 움직입니다.
 5. 로봇이 장애물이 없는 간단한 목표까지 팔을 이동합니다.
 6. 팔 오른쪽에 상자 개체가 환경에 추가됩니다.
    |B|

 7. 로봇이 박스와의 충돌을 피하면서 팔을 포즈 목표 지점으로 이동합니다.
 8. 물체가 손목에 부착됩니다(색상이 보라색/주황색/녹색으로 변경됨).
 9. 로봇이 상자와의 충돌을 피하면서 물체를 부착한 팔을 포즈 골대로 이동합니다.
 10. 물체가 손목에서 분리됩니다(색상이 다시 녹색으로 바뀝니다).
 11. 물체가 환경에서 제거됩니다.

.. |B| image:: ./move_group_interface_tutorial_robot_with_box.png

전체 코드
---------------
전체 코드는 MoveIt GitHub 프로젝트의 <examples/move_group_interface/src/move_group_interface_tutorial.cpp>`에서 볼 수 있습니다. 이제 코드를 하나씩 살펴보며 기능을 설명하겠습니다.

.. tutorial-formatter:: ./src/move_group_interface_tutorial.cpp

시작 파일
---------------
전체 실행 파일은 GitHub의 :codedir:`here<examples/move_group_interface/launch/move_group_interface_tutorial.launch.py>`에 있습니다. 이 튜토리얼의 모든 코드는 MoveIt 설정의 일부로 제공되는 **moveit2_tutorials** 패키지에서 실행할 수 있습니다.


허용 오차 설정에 대한 참고 사항
----------------------------
`MoveGroupInterface의 <https://github.com/ros-planning/moveit2/blob/ed844d4b46f70ed6e97d0c1f971ab2b9a45f156d/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h#L293>`_ *setGoalTolerance()* 및 관련 메서드는 실행이 아닌 **계획**에 대한 허용 오차를 설정합니다.

실행 허용 오차를 구성하려면 FollowJointTrajectory 컨트롤러를 사용하는 경우 *controller.yaml* 파일을 편집하거나 플래너에서 생성된 궤적 메시지에 수동으로 추가해야 합니다.
