Planning Scene
==================================

The :moveit_codedir:`PlanningScene<moveit_core/planning_scene/include/moveit/planning_scene/planning_scene.h>` class provides the main interface that you will use
for collision checking and constraint checking. In this tutorial, we
will explore the C++ interface to this class.

Getting Started
---------------
아직 완료하지 않았다면, :doc:`Getting Started </doc/tutorials/getting_started/getting_started>` 단계를 완료하세요.

The entire code
---------------
전체 코드는 :codedir:`here in the MoveIt GitHub project<examples/planning_scene>`.

.. tutorial-formatter:: ./src/planning_scene_tutorial.cpp

The launch file
---------------
launch 파일은 :codedir:`here <examples/planning_scene/launch/planning_scene_tutorial.launch.py>` on GitHub. All the code in this tutorial can be compiled and run from the moveit_tutorials package.

Running the code
----------------
moveit_tutorials 패키지에서 직접 코드를 실행하도록 launch 파일을 roslaunch로 실행하세요.: ::

 ros2 launch moveit2_tutorials planning_scene_tutorial.launch.py

Expected Output
---------------

출력은 다음과 같은 형태입니다. 임의의 joint 값을 사용하고 있으므로 실제 결과와 다소 차이가 있을 수 있습니다. ::

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

**Note:** 여러분의 콘솔에서 ROS 콘솔 출력 형식이 다르게 나타나더라도 걱정하지 마십시오. 다음 문서를 참고하여 ROS console logger를 사용자 정의할 수 있습니다. :ros_documentation:`this tutorial <Tutorials/Logging-and-logger-configuration.html#console-output-formatting>`
