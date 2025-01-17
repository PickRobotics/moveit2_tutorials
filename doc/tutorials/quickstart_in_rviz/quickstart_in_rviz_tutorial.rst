.. _Quickstart in RViz:

MoveIt RViz에서 바로 시작하기
==============================
.. image:: rviz_plugin_head.png
   :width: 700px

이 튜터리얼은 RViz와 MoveIt Display 플러그인을 사용하여 MoveIt에서 motion plan을 생성하는 방법을 가르칠 것입니다. RViz는 ROS의 주요 시각화 도구이며 로봇공학에서 디버깅에 매우 유용한 도구입니다. MoveIt Display 플러그인을 사용하면 가상 환경 (planning scenes)을 설정하고 로봇을 대화식으로 시작 및 목표 상태를 생성하고 다양한 모션 플래너를 테스트하고 결과를 시각화할 수 있습니다. 시작해 봅시다!

시작하기
---------------
먼저 :doc:`Getting Started </doc/tutorials/getting_started/getting_started>` 또는 :doc:`Docker Guide </doc/how_to_guides/how_to_setup_docker_containers_in_ubuntu>` 을 완료하고 진행하세요.
Docker 가이드를 따라한 경우, 튜토리얼을 설정하기 위해 :ref:`create_colcon_workspace` 가이드를 따라하세요.

Step 1: 데모 런칭 및 Plugin 설정
------------------------------------------------

* 데모 런칭: ::

   ros2 launch moveit2_tutorials demo.launch.py

* 이것을 처음으로 수행하는 경우, RViz에서 empty world를 보게되며 Motion Planning 플러그인을 추가해야합니다.:

  * RViz내에서 empty world를 볼 수 있습니다.:

  |A|

  * RViz Displays Tab에서 *Add* 누르기:

  |B|

  * moveit_ros_visualization 폴더에서 DisplayType으로 "MotionPlanning"을 선택하세요. "Ok"를 누릅니다.

  |C|

  * 이제 RViz에 Kinova 로봇이 나타납니다.:

  |D|

.. |A| image:: rviz_empty.png
               :width: 700px

.. |B| image:: rviz_click_add.png
               :width: 405px

.. |C| image:: rviz_plugin_motion_planning_add.png
               :width: 400px

.. |D| image:: rviz_start.png
               :width: 700px

* 일단 Motion Planning 플러그인을 로드하면 설정할 수 있습니다. "Displays" 하위 창의 "Global Options" 탭에서 **Fixed Frame** 필드를 ``/base_link`` 로 설정하십시오.

* 이제 로봇 (이 경우 Kinova Gen 3)을 위한 플러그인 설정을 시작할 수 있습니다. "Displays"에서 "MotionPlanning"을 클릭하십시오.

  * **Robot Description** 필드가 ``robot_description`` 로 설정되어 있는지 확인하십시오.

  * **Planning Scene Topic** 필드가 ``/monitored_planning_scene`` 로 설정되어 있는지 확인하십시오.
    topic 이름을 클릭하여 topic-name drop-down을 노출시킵니다.

  * **Planned Path** 아래 **Trajectory Topic** 필드가 ``/display_planned_path`` 로 설정되어 있는지 확인하십시오.

  * **Planning Request** 에서 **Planning Group** 를 ``manipulator`` 으로 변경하십시오. 이것은 하단 왼쪽의 MotionPlanning 패널에서도 볼 수 있습니다.


.. image:: rviz_plugin_start.png
   :width: 700px


Step 2: 시각화된 로봇 조작하기
---------------------------------------
4가지 서로 다른 겹치는 시각화가 있습니다:

#. ``/monitored_planning_scene`` 계획 환경에서의 로봇 구성(기본적으로 활성화)

#. 로봇의 계획 경로(기본적으로 활성화)

#. 녹색: 모션 계획의 시작 상태(기본적으로 비활성화)

#. 오렌지색: 모션 계획의 목표도달 상태(기본적으로 활성화)

각 시각화의 표시 상태는 다음과 같은 체크박스를 사용하여 켜거나 끌 수 있습니다. :

#. **Scene Robot** 트리 메뉴의 **Show Robot Visual** 체크박스를 사용하여 scene 로봇 계획하기

#. **Planned Path** 트리 메뉴의 **Show Robot Visual** 체크박스를 사용하여 계획된 경로

#. **Planning Request** 트리 메뉴의 **Query Start State** 체크박스를 사용하여 시작 상태

#. **Planning Request** 트리 메뉴의 **Query Goal State** 체크박스를 사용하여 목표 상태

* 다른 시각화를 켜거나 끄기 위해 이 모든 체크박스를 사용해 보세요.

.. image:: rviz_plugin_visualize_robots.png
   :width: 700px

Step 3: Kinova Gen 3와 상호작용
--------------------------------------

다음 단계에서는 scene robot, 시작 상태, 목표 상태만 필요합니다:

#. **Planned Path** 트리 메뉴에서 **Show Robot Visual** 체크박스를 선택합니다.

#. **Scene Robot** 트리 메뉴에서 **Show Robot Visual** 체크박스를 선택 해제합니다.

#. **Planning Request** 트리 메뉴에서 **Query Goal State** 체크박스를 선택합니다.

#. **Planning Request** 트리 메뉴에서 **Query Start State** 체크박스를 선택합니다.

이제 두 개의 상호 작용 마커가 있어야 합니다. 오렌지색 팔에 해당하는 한 마커는 모션 플래닝의 "목표 상태(Goal State)"를 설정하는 데 사용되고, 녹색 팔에 해당하는 다른 마커는 모션 플래닝의 "시작 상태(Start State)"를 설정하는 데 사용됩니다. 상호 작용 마커가 보이지 않으면 RViz 상단 메뉴에서 **Interact**을 누르십시오(참고: 일부 도구가 숨겨져 있을 수 있습니다. 아래 표시된 것처럼 상단 메뉴에서 **"+"**를 눌러 **Interact** 도구를 추가하십시오).

.. image:: rviz_plugin_interact.png
   :width: 700px

이제 마커를 사용하여 팔을 드래그하고 방향을 변경할 수 있습니다. 직접 시도해 보세요!

충돌하도록 이동시키기
+++++++++++++++++++++++

이 섹션 진행을 위해서 계획 경로(planned path)와 목표 상태(goal state)를 숨기기를 합니다.:

#. **Planned Path** 트리 메뉴에서 **Show Robot Visual** 체크 상자를 해제합니다.

#. **Planning Request** 트리 메뉴에서 **Query Goal State** 체크 상자를 해제합니다.

이제 시작 상태(녹색 팔)만 표시되어야 합니다. 팔의 두 link가 서로 충돌하는 설정으로 이동시켜보세요. (찾기 어려운 경우 MotionPlanning 플러그인의 Planning 탭 아래 있는 "Use Collision-Aware IK" 체크 상자가 해제되어 있는지 확인하십시오.) 이 작업을 수행하면 충돌상태에 있는 links가 빨강색으로 변합니다.

.. image:: rviz_plugin_collision.png
   :width: 700px

이제 "Use Collision-Aware IK" 체크 상자를 선택하고 두 links를 다시 충돌하도록 움직여보세요. 이 체크 상자가 선택되어 있으면 IK solver는 원하는 end-effector 포즈에 대한 충돌 없는 해결책을 찾기 위해 계속 시도합니다. 체크 상자가 선택되어 있지 않으면, solver는 해당 솔루션에서 충돌 발생을 허용합니다. 충돌하는 링크는 항상 체크 상자 상태에 관계없이 빨강색으로 표시됩니다.

.. image:: rviz_plugin_collision_aware_ik_checkbox.png
   :width: 700px

도달 가능 작업공간 밖으로 이동시키기
++++++++++++++++++++++++++++++++++++++
end-effector를 도달 가능한 작업 공간 밖으로 이동시키려고할때 어떤 일이 발생하는지 살펴봅시다.

.. image:: rviz_plugin_invalid.png
   :width: 700px

다음 섹션으로 이동하기 전에 계획된 경로(planned path) 및 목표 상태(goal state)를 다시 활성화하십시오.:

#. **Planned Path** 트리 메뉴에서 **Show Robot Visual** 체크박스를 선택합니다.

#. **Planning Request** 트리 메뉴에서 **Query Goal State** 체크박스를 선택합니다.

Moving Joints or in Null Space
++++++++++++++++++++++++++++++
**Joints** 탭을 사용하여 단일 관절과 7-DOF 로봇의 여분 관절을 이동시킬 수 있습니다. 아래 애니메이션과 같이 "null space exploration" 슬라이더를 이동시켜 보세요.

.. raw:: html

    <video width="700px" controls="true" autoplay="true" loop="true">
        <source src="../../../_static/videos/rviz_joints_nullspace.webm" type="video/webm">
        The joints moving while the end effector stays still
    </video>

Step 4: Kinova Gen 3로 Motion Planning 사용하기
---------------------------------------------------

* 이제 MoveIt RViz 플러그인에서 Kinova Gen 3으로 모션 플래닝 시작할 수 있습니다.

  * 시작 상태(Start State)를 원하는 위치로 이동시킵니다.

  * 목표 상태(Goal State)를 다른 원하는 위치로 이동시킵니다.

  * 두 상태 모두 로봇 자체와 충돌상태에 있지 않도록 해야합니다.

  * **Planned Path** 트리 메뉴에서 **Show Trail** 체크박스의 체크를 해제합니다.

* **Planning** 탭 아래 **MotionPlanning** 창에서 **Plan** 버튼을 누릅니다.

* **Planned Path** 트리 메뉴에서 **Show Trail** 체크박스를 선택합니다. 이제 팔의 경로가 연속적인 manipulator 포즈들로 표시됩니다.

.. image:: rviz_plugin_planned_path.png
   :width: 700px

Introspecting Trajectory Waypoints
++++++++++++++++++++++++++++++++++

RViz에서 포인트별로 궤적을 시각적으로 검토할 수 있습니다.

* "*Panels*" 메뉴에서 "*Trajectory - Trajectory Slider*"를 선택하십시오. RViz에 새 슬라이더 패널이 표시됩니다.

* 목표 포즈를 설정한 다음 *Plan* 을 실행하십시오.

* "*Slider*" 패널을 조작하십시오(예: 슬라이더 이동시키기, "*Play*" 버튼 누르기).

참고: end-effector를 새 목표 위치로 이동시킨 후 *Play* 을 실행하기 전에 반드시 *Plan* 을 실행해야 합니다. 그렇지 않으면 이전 목표 지점에 대한 waypoints가 표시됩니다.


.. image:: rviz_plugin_plan_slider.png
   :width: 700px

Plan Cartesian motions
++++++++++++++++++++++

"Use Cartesian Path" 체크박스가 활성화되면 로봇은 end-effector를 데카르트 공간에서 직선으로 이동하려고 시도합니다.

.. image:: rviz_plugin_plan_free.png
   :width: 700px

.. image:: rviz_plugin_plan_cartesian.png
   :width: 700px


궤적 실행, 속도 조정
+++++++++++++++++++++++++++++++++++++++

성공적인 계획 후에 "Plan & Execute" 또는 "Execute" 을 클릭하면 궤적을 로봇에게 전송합니다. 이 튜토리얼에서는 ``kinova_demo.launch`` 를 사용했기 때문에 로봇은 단지 시뮬레이션만 됩니다.

초기에 기본 속도와 가속도는 로봇의 최대 속도의 10% (``0.1``)로 조정됩니다. 이러한 스케일링 요소는 아래 표시된 Planning 탭에서 변경하거나 로봇의 ``moveit_config`` (``joint_limits.yaml`` 에 있음)에서 기본값을 변경할 수 있습니다.

.. image:: rviz_plugin_collision_aware_ik_checkbox.png
   :width: 700px


다음 단계
-----------

RViz Visual Tools
+++++++++++++++++
많은 튜토리얼에서 데모를 단계별로 진행하기 위해서 ``moveit_visual_tools`` 사용합니다. 다음 튜토리얼로 넘어가기 전에 **RvizVisualToolsGui** 를 활성화하는 것이 좋습니다.

"*Panels*" 메뉴에서 "*Add New Panels*" 를 선택하세요. 메뉴에서 "*RvizVisualToolsGui*" 를 선택하고 OK를 클릭합니다. RViz에 새로운 패널이 추가되는 것을 확인할 수 있습니다.

.. image:: rviz_add_rviz_visual_tools.png
   :width: 400px

.. image:: rviz_panels.png
   :width: 700px

설정 저장하기
+++++++++++++++++++++++++
RViz에서는 ``File->Save Config`` 을 통해 설정을 저장할 수 있습니다. 다음 튜터리얼로 넘어가기 전에 반드시 이 작업을 수행해야 합니다. 만약 설정을 새 이름으로 저장하고 싶다면 ``File->Save Config As`` 을 사용하고 다음과 같이 설정 파일을 참조할 수 있습니다. : ::

   ros2 launch moveit2_tutorials demo.launch.py rviz_config:=your_rviz_config.rviz

``your_rviz_config.rviz``를 ``moveit2_tutorials/doc/tutorials/quickstart_in_rviz/launch/`` 폴더에 저장한 파일 이름으로 변경합니다. workspace를 빌드하고나면 검색이 가능합니다.


다음 튜터리얼
+++++++++++++++

:doc:`Your First MoveIt Project </doc/tutorials/your_first_project/your_first_project>` 에서 MoveIt을 사용하여 로봇 이동 계획과 실행을 위한 C++ 프로그램을 만들게 됩니다.
