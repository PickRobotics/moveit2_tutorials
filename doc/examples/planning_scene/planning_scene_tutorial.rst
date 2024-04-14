Planning Scene
==================================

:moveit_codedir:`PlanningScene<moveit_core/planning_scene/include/moveit/planning_scene/planning_scene.h>` 클래스는 충돌 검사 및 제약 조건 검사를 위해 사용하는 주요 인터페이스를 제공합니다. 이번 튜토리얼에서는 이 클래스의 C++ 인터페이스에 대해 살펴보겠습니다.

시작하기
---------------
먼저 :doc:`Getting Started </doc/tutorials/getting_started/getting_started>` 를 완료하세요.

전체 코드
---------------
전체 코드는 다음에서 볼 수 있습니다 :codedir:`here in the MoveIt GitHub project<examples/planning_scene>`.

.. tutorial-formatter:: ./src/planning_scene_tutorial.cpp

Launch 파일
---------------
전체 launch 파일은 :codedir:`here <examples/planning_scene/launch/planning_scene_tutorial.launch.py>` Github에 있습니다. 이 튜터리얼의 모든 코드는 moveit_tutorials package에서 컴파일 및 실행할 수 있습니다.

코드 실행하기
----------------
moveit2_tutorials 패키지에서 직접 코드를 실행하기 위해서 launch 파일을 실행합니다.::

 ros2 launch moveit2_tutorials planning_scene_tutorial.launch.py

예상 출력
---------------

예상 출력은 다음과 같은 형식입니다. 랜덤 조인트 값을 사용하기 때문에 숫자는 일치하지 않을 것입니다.: ::

 moveit2_tutorials: Test 1: Current state is in self collision
 moveit2_tutorials: Test 2: Current state is not in self collision
 moveit2_tutorials: Test 3: Current state is not in self collision
 moveit2_tutorials: Test 4: Current state is valid
 moveit2_tutorials: Test 5: Current state is in self collision
 moveit2_tutorials: Contact between: panda_leftfinger and panda_link1
 moveit2_tutorials: Contact between: panda_link1 and panda_rightfinger
 moveit2_tutorials: Test 6: Current state is not in self collision
 moveit2_tutorials: Test 7: Current state is not in self collision
 moveit2_tutorials: Test 8: Random state is not constrained
 moveit2_tutorials: Test 9: Random state is not constrained
 moveit2_tutorials: Test 10: Random state is not constrained
 moveit2_tutorials: Test 11: Random state is feasible
 moveit2_tutorials: Test 12: Random state is not valid

**Note:** 출력형태가 다른 ROS 콘솔 포맷이더라도 신경쓰지마세요. ROS 콘솔 logger는 다음 문서를 보고 커스텀 가능합니다. :ros_documentation:`this tutorial <Tutorials/Logging-and-logger-configuration.html#console-output-formatting>`
