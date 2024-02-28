MoveIt RViz에서 바로 시작하기
==========================
.. image:: rviz_plugin_head.png
   :width: 700px

이 튜터리얼은 RViz와 MoveIt Display 플러그인을 사용하여 MoveIt에서 motion plan을 생성하는 방법을 가르칠 것입니다. RViz는 ROS의 주요 시각화 도구이며 로봇공학에서 디버깅에 매우 유용한 도구입니다. MoveIt Display 플러그인을 사용하면 가상 환경 (planning scenes)을 설정하고 로봇을 대화식으로 시작 및 목표 상태를 생성하고 다양한 모션 플래너를 테스트하고 결과를 시각화할 수 있습니다. 시작해 봅시다!

시작하기
---------------
아직 수행하지 않은 경우 :doc:`시작하기 </doc/tutorials/getting_started/getting_started>` 또는 :doc:`Docker 가이드 </doc/how_to_guides/how_to_setup_docker_containers_in_ubuntu>`의 단계를 완료하십시오. 2022년 9월 26일 기준으로 Cyclone DDS를 활성화했는지 확인하십시오.

Step 1: 데모 런칭 및 Plugin 설정
------------------------------------------------

* 데모 런칭: ::

   ros2 launch moveit2_tutorials demo.launch.py rviz_config:=panda_moveit_config_demo_empty.rviz

* 이것을 처음으로 수행하는 경우, RViz에서 empty 환경을 볼 수 있으며 Motion Planning 플러그인을 추가해야합니다.:

  * RViz내에서 emtpy world를 볼 수 있다.:

  |A|

  * RViz Displays Tab에서 *Add* 누르기:

  |B|

  * moveit_ros_visualization 폴더에서 "MotionPlanning"을 DisplayType으로 선택하십시오. "Ok"를 누릅니다.

  |C|

  * 이제 RViz에 Panda robot이 나타난다.:

  |D|

.. |A| image:: rviz_empty.png
               :width: 700px

.. |B| image:: rviz_click_add.png
               :width: 405px

.. |C| image:: rviz_plugin_motion_planning_add.png
               :width: 400px

.. |D| image:: rviz_start.png
               :width: 700px

* 일단 Motion Planning 플러그인을 로드하면 설정할 수 있습니다. "Displays" 하위 창의 "Global Options" 탭에서 **Fixed Frame** 필드를 ``/panda_link0``로 설정하십시오.

* 이제 로봇 (이 경우 Panda)을 위한 플러그인을 구성하기 시작할 수 있습니다. "Displays"에서 "MotionPlanning"을 클릭하십시오.

  * **Robot Description** 필드가 ``robot_description``로 설정되어 있는지 확인하십시오.

  * **Planning Scene Topic** 필드가 ``/monitored_planning_scene``로 설정되어 있는지 확인하십시오. 토픽 이름을 클릭하여 토픽 이름 드롭다운을 노출시킵니다.

  * **Planned Path** 아래 **Trajectory Topic** 필드가 ``/display_planned_path``로 설정되어 있는지 확인하십시오.

  * **Planning Request**에서 **Planning Group**를 ``panda_arm``으로 변경하십시오. 이것은 MotionPlanning 패널의 하단 왼쪽에도 볼 수 있습니다.


.. image:: rviz_plugin_start.png
   :width: 700px


Step 2: 시각화된 로봇과 Play
---------------------------------------
4가지 서로 다른 겹치는 시각화가 있습니다:

#. The robot's configuration in the ``/monitored_planning_scene`` planning environment (active by default).

#. The planned path for the robot (active by default).

#. Green: The start state for motion planning (disabled by default).

#. Orange: The goal state for motion planning (active by default).

The display states for each of these visualizations can be toggled on and off using checkboxes:

#. The planning scene robot using the **Show Robot Visual** checkbox in the **Scene Robot** tree menu.

#. The planned path using the **Show Robot Visual** checkbox in the **Planned Path** tree menu.

#. The start state using the **Query Start State** checkbox in the **Planning Request** tree menu.

#. The goal state using the **Query Goal State** checkbox in the **Planning Request** tree menu.

* Play with all these checkboxes to switch on and off different visualizations.

.. image:: rviz_plugin_visualize_robots.png
   :width: 700px

Step 3: Interact with the Panda
-------------------------------

For the next steps we will want only the scene robot, start state and goal state:

#. Check the **Show Robot Visual** checkbox in the **Planned Path** tree menu

#. Un-check the **Show Robot Visual** checkbox in the **Scene Robot** tree menu

#. Check the **Query Goal State** checkbox in the **Planning Request** tree menu.

#. Check the **Query Start State** checkbox in the **Planning Request** tree menu.

There should now be two interactive markers. One marker corresponding to the orange colored arm will be used to set the "Goal State" for motion planning and the other marker corresponding to a green colored arm are used to set the "Start State" for motion planning. If you don't see the interactive markers press **Interact** in the top menu of RViz (Note: some tools may be hidden, press **"+"** in the top menu to add the **Interact** tool as shown below).

.. image:: rviz_plugin_interact.png
   :width: 700px

You should now be able to use these markers to drag the arm around and change its orientation. Try it!

Moving into collision
+++++++++++++++++++++

For this section, hide the planned path and the goal state:

#. Un-check the **Show Robot Visual** checkbox in the **Planned Path** tree menu

#. Un-check the **Query Goal State** checkbox in the **Planning Request** tree menu.

Now, only the Start State (the green colored arm) should be visible.  Try to move the arm into a configuration where two of its links are in collision with each other.  (If you find this difficult, make sure the "Use Collision-Aware IK" checkbox under the Planning tab of the MotionPlanning plugin is un-checked.)  Once you do this, the links that are in collision will turn red.

.. image:: rviz_plugin_collision.png
   :width: 700px

Now, check the "Use Collision-Aware IK" checkbox, and try again to move two of the links into collision with each other.  When the checkbox is ticked, the IK solver will keep attempting to find a collision-free solution for the desired end-effector pose. When it is not checked, the solver will allow collisions to happen in the solution. The links in collision will always still be visualized in red, regardless of the state of the checkbox.

.. image:: rviz_plugin_collision_aware_ik_checkbox.png
   :width: 700px

Moving out of Reachable Workspace
+++++++++++++++++++++++++++++++++
Note what happens when you try to move an end-effector out of its reachable workspace.

.. image:: rviz_plugin_invalid.png
   :width: 700px

Before moving onto the next section, re-enable the planned path and the goal state:

#. Check the **Show Robot Visual** checkbox in the **Planned Path** tree menu

#. Check the **Query Goal State** checkbox in the **Planning Request** tree menu.

Moving Joints or in Null Space
++++++++++++++++++++++++++++++
You can use the **Joints** tab to move single joints and the redundant joints of 7-DOF robots. Try moving the "null space exploration" slider as shown in the animation below.

.. raw:: html

    <video width="700px" controls="true" autoplay="true" loop="true">
        <source src="../../../_static/videos/rviz_joints_nullspace.webm" type="video/webm">
        The joints moving while the end effector stays still
    </video>

Step 4: Use Motion Planning with the Panda
-------------------------------------------

* Now, you can start motion planning with the Panda in the MoveIt RViz Plugin.

  * Move the Start State to a desired location.

  * Move the Goal State to another desired location.

  * Make sure both states are not in collision with the robot itself.

  * Make sure the Planned Path is being visualized. Also check the
    **Show Trail** checkbox in the **Planned Path** tree menu.

* In the **MotionPlanning** window under the **Planning** tab, press the **Plan** button. You
  should be able to see a visualization of the arm moving and a trail.

.. image:: rviz_plugin_planned_path.png
   :width: 700px

Introspecting Trajectory Waypoints
++++++++++++++++++++++++++++++++++

You can visually introspect trajectories point by point in RViz.

* From "*Panels*" menu, select "*Trajectory - Trajectory Slider*". You'll see a new Slider panel on RViz.

* Set your goal pose, then run *Plan*.

* Play with the "*Slider*" panel, e.g. move the slider, push "*Play*" button.

NOTE: Once you placed your EEF to a new goal, be sure to run *Plan* before running *Play* -- otherwise you'll see the waypoints for the previous goal if available.

.. image:: rviz_plugin_slider.png
   :width: 700px

Plan Cartesian motions
++++++++++++++++++++++

If the "Use Cartesian Path" checkbox is activated, the robot will attempt to move the end effector linearly in cartesian space.

.. image:: rviz_plan_free.png
   :width: 700px

.. image:: rviz_plan_cartesian.png
   :width: 700px


Executing Trajectories, Adjusting Speed
+++++++++++++++++++++++++++++++++++++++

Clicking "Plan & Execute" or "Execute" after a successful plan will send the trajectory to the robot - in this tutorial, since you used ``demo.launch``, the robot is only simulated.

Initially, the default velocity and acceleration are scaled to 10% (``0.1``) of the robot's maximum. You can change these scaling factors in the Planning tab shown below, or change these default values in the ``moveit_config`` of your robot (in ``joint_limits.yaml``).

.. image:: rviz_plugin_collision_aware_ik_checkbox.png
   :width: 700px


Next Steps
----------

RViz Visual Tools
+++++++++++++++++
Many of the tutorials use ``moveit_visual_tools`` to step through a demo. Before continuing on to the next tutorials it is a good idea to enable the **RvizVisualToolsGui**.

From "*Panels*" menu, select "*Add New Panels*". From the menu, select "*RvizVisualToolsGui*" and click OK. You'll see the new panel added to RViz.

.. image:: rviz_add_rviz_visual_tools.png
   :width: 400px

.. image:: rviz_panels.png
   :width: 700px

Saving Your Configuration
+++++++++++++++++++++++++
RViz enables you to save your configuration under ``File->Save Config``. You should do this before continuing on to the next tutorials. If you choose to save your configuration under a new name, you can use ``File->Save Config As`` and refer to your configuration file using: ::

   ros2 launch moveit2_tutorials demo.launch.py rviz_config:=your_rviz_config.rviz

Replace ``your_rviz_config.rviz`` with the name of the file you saved to ``moveit2_tutorials/doc/tutorials/quickstart_in_rviz/launch/`` and build the workspace so it can be found.


Next Tutorial
+++++++++++++

In :doc:`Your First MoveIt Project </doc/tutorials/your_first_project/your_first_project>`, you will create a C++ program using MoveIt to plan and execute moves.
