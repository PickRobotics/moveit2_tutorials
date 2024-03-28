MoveIt Setup Assistant
========================

.. image:: setup_assistant_launch.png
   :width: 700px
   :align: center

개요
--------
MoveIt Setup Assistant는 로봇을 MoveIt과 함께 사용하도록 설정하는 그래픽 사용자 인터페이스입니다.
주요 기능은 로봇에 대한 의미적 로봇 설명 형식(Semantic Robot Description Format(SRDF)) 파일을 생성하는 것으로, 이 파일은 planning group, end-effectors, 다양한 운동학 파라미터와 같이 MoveIt에 필요한 추가 정보를 지정합니다.
또한 MoveIt pipeline과 함께 사용하는 데 필요한 다른 설정 파일도 생성합니다.
MoveIt Setup Assistant를 사용하려면 로봇에 대한 URDF 파일이 필요합니다.

URDF 파일이 있으면 MoveIt Setup Assistant를 열고 URDF를 가져올 수 있습니다.
이 튜터리얼은 운동학 구조 정의, 계획 그룹 및 엔드 이펙터 지정, 충돌 검사 관련 설정과 같은 로봇의 여러 요소를 설정하는 일련의 단계를 안내합니다.
URDF 및 SRDF에 대한 자세한 내용은 :doc:`URDF와 SRDF 개요 </doc/examples/urdf_srdf/urdf_srdf_tutorial>` 페이지를 참조하십시오.

시작하기
---------------

MoveIt 과 ROS 2

* :doc:`instructions for installing MoveIt </doc/tutorials/getting_started/getting_started>` 을 먼저 완료하세요.

* 우리는 `moveit_resources_panda_description <https://github.com/ros-planning/moveit_resources/tree/humble/panda_description>`_ 패키지를 사용합니다. 이 패키지는 MoveIt 설치에 따라 완료했다면 이미 workspace에 포함되어 있습니다.

단계 1: 시작
-------------

* MoveIt Setup Assistant를 시작하기 위해서: ::

   ros2 launch moveit_setup_assistant setup_assistant.launch.py

* 이 패키지를 사용하면 두 가지 선택지를 제공하는 시작 화면이 나타납니다:
   **새로운 MoveIt Configuration Package 생성하기(Create New MoveIt Configuration Package)** 또는 **기존 MoveIt 설정 패키지 편집(Edit Existing MoveIt Configuration Package)** 

* **Create New MoveIt Configuration Package** 를 클릭하면
   다음과 같은 화면이 나타납니다.:

.. image:: setup_assistant_create_package.png
   :width: 700px
   :align: center

* **Browse** 버튼을 클릭하고 다음 경로에 있는 ``moveit_resources_panda_description 패키지`` 의 ``panda.urdf`` 파일로 이동하십시오: ::

   ~/ws_moveit2/src/moveit_resources/panda_description/urdf/panda.urdf

  해당 파일을 선택한 다음 **Load Files** 를 클릭하십시오. Setup Assistant는 파일을 로드할 것이며 (몇 초 정도 걸릴 수 있음),
  이 화면이 나타납니다.:

.. image:: setup_assistant_load_panda_urdf.png
   :width: 700px
   :align: center

단계 2: Self-Collision 행렬 생성
--------------------------------------

기본 제공되는 self-collision 행렬 생성기는 로봇의 안전한 links 쌍에 대해 충돌 검사를 비활성화하여 모션 계획 시간을 단축하는 데 도움이 됩니다.
이는 항상 충돌하는 links 쌍, 절대 충돌하지 않는 links 쌍, 로봇의 기본 위치에서 충돌하는 links 쌍, 또는 운동학 체인(Kinematic chain)에서 서로 인접한 links 쌍을 식별하여 수행됩니다.

샘플링 밀도를 설정할 수 있습니다. 샘플링 밀도는 self-collision 여부를 확인하기 위해 검사하는 로봇의 랜덤 위치 수를 결정합니다.
기본적으로 생성기는 10,000개의 랜덤 위치를 검사하지만, 더 정확한 결과를 위해 샘플링 밀도의 최대값을 사용하는 것이 좋습니다.
충돌 검사는 병렬 처리되어 충돌 행렬 생성을 위한 전체 처리 시간을 줄입니다.

충돌 행렬을 생성하려면 MoveIt Setup Assistant의 왼쪽 패널에서 **Self-Collisions** 창을 선택하고 자기 충돌 샘플링 밀도를 조정하십시오.
그런 다음 충돌 **Generate Collision Matrix** 버튼을 클릭하여 계산을 시작하십시오.
Setup Assistant은 몇 초 안에 self-collision 행렬을 계산합니다.
self-collision 행렬은 충돌 검사를 안전하게 비활성화할 수 있는 링크 쌍을 확인하는 과정을 포함합니다.

.. image:: collision_matrix/setup_assistant_panda_collision_matrix.png
   :width: 700px
   :align: center

계산이 완료되면 결과가 메인 테이블에 표시됩니다.
테이블은 충돌 검사로부터 비활성화 시키기 위해 안전하거나 안전하지 않은 것으로 확인된 links 쌍을 보여줍니다.
비활성화하더라도 안전한 links는 체크 마크로 표시됩니다. 특정 links 쌍에 대한 자체 충돌 검사를 활성화 또는 비활성화하려면 필요에 따라 체크 마크를 수동으로 조정할 수 있습니다.

.. image:: collision_matrix/setup_assistant_panda_collision_matrix_done.png
   :width: 700px
   :align: center

단계 3: 가상 조인트 추가
--------------------------
가상 조인트는 주로 로봇을 world에 연결하는데 사용됩니다.
베이스가 고정된 manipulator인 Panda arm의 경우, 고정 가상 조인트를 정의하는 것은 선택 사항입니다. 그러나 ``panda_link0`` 을 ``world`` 프레임에 연결하는 ``fixed`` 가상 조인트를 정의할 것입니다. 이 가상 조인트는 팔의 base 부분이 world 프레임에서 정지 상태를 유지한다는 것을 의미합니다.

* **Virtual Joints** 창 선택기를 클릭하십시오. **Add Virtual Joint** 를 클릭하십시오.

* 조인트 이름을 ``virtual_joint`` 로 설정하십시오.

* 자식 link를 ``panda_link0`` 으로, 부모 프레임 이름을 ``world`` 로 설정하십시오.

* 조인트 타입을 ``fixed`` 로 설정하십시오.

* **Save** 을 클릭하면 다음 화면이 나타납니다.:

.. image:: setup_assistant_panda_virtual_joints.png
   :width: 700px
   :align: center

.. note:: 가상 관절은 모바일 베이스를 탑재한 로봇, 예를 들어 mobile manipulator에 특히 유용합니다. 이런 로봇들은 로봇의 베이스의 모션을 모델링할 수 있게 해주며, 이는 모션 계획 및 제어에 필수적입니다. 예제로, 가상 평면 관절(virtual planar joint)을 사용하여 로봇 베이스 프레임을 odometry 프레임에 연결하여 로봇의 환경 내 이동을 효과적으로 표현할 수 있습니다.

Step 4: Planning Groups 추가
---------------------------------

MoveIt의 Planning groups은 로봇의 팔이나 엔드 이펙터와 같은 다른 부분들을 의미론적으로 기술하여 모션 계획을 용이하게 합니다.

move group은 로봇의 특정 운동학 체인(kinematic chain)에 해당하도록 설정할 수 있습니다. kinematic 체인은 로봇의 베이스부터 엔드 이펙터까지의 일련의 변환을 정의하는 링크와 조인트의 집합입니다. 예를 들어, move group은 로봇의 팔을 나타내도록 정의될 수 있으며, 이는 팔을 움직이는 데 필요한 모든 링크와 조인트로 구성됩니다.

Move groups은 또한 로봇에 관련된 링크 또는 조인트 세트로 표현될 수도 있습니다.
예를 들어, Move groups은 로봇의 그리퍼(gripper)를 표현하도록 정의될 수 있으며, 이는 그리퍼의 열거나 닫는 모션을 실현하는 데 필요한 모든 링크 또는 조인트로 구성됩니다.

* **Planning Groups** 창 선택기를 클릭하세요.

* **Add Group** 을 클릭하면 다음과 같은 화면이 나타납니다.:

.. image:: planning_groups/setup_assistant_panda_planning_groups.png
   :width: 700px
   :align: center

arm group 추가

* 먼저 Panda arm을 planning group에 추가할 예정입니다.

  * **Group Name** 에 ``panda_arm`` 을 입력합니다.

  * 운동학 솔버(kinematics solver)에 MoveIt 기본값인 **kdl_kinematics_plugin/KDLKinematicsPlugin** 을 선택하세요.
    대안으로 :doc:`IKFast </doc/examples/ikfast/ikfast_tutorial>` 혹은 `pick_ik <https://github.com/PickNikRobotics/pick_ik>`_ 와 같은 다른 플러그인도 사용할 수 있습니다.

  * **Kin. Search Resolution** 와 **Kin. Search Timeout** 은 기본값 그대로 유지하세요.

.. image:: planning_groups/setup_assistant_panda_arm_group.png
   :width: 700px
   :align: center

* 이제 **Add Joints** 버튼을 클립합니다. 왼쪽 창에는 모든 조인트 목록이 표시됩니다. 팔에 속하는 모든 조인트를 선택하여 오른쪽 창에 추가해야 합니다. 조인트는 내부 트리 구조에 저장된 순서대로 배열되어 있으므로, 직렬 연결(serial chain)의 조인트를 쉽게 선택할 수 있습니다.

  * ``virtual_joint`` 을 클릭하고, 키보드의 **Shift** 키를 누른 상태에서  ``panda_joint8`` 을 클릭하세요.  그 다음 **>** 버튼을 클릭하여 **Selected Joints** 의 목록(오른쪽)에 이 관절들을 추가하세요.

.. image:: planning_groups/setup_assistant_panda_arm_group_joints.png
   :width: 700px
   :align: center

* **Save** 를 클릭하여 선택한 그룹을 저장합니다.

.. image:: planning_groups/setup_assistant_panda_arm_group_saved.png
   :width: 700px
   :align: center

end-effector group 추가

.. image:: planning_groups/setup_assistant_panda_hand_group.png
   :width: 700px
   :align: center

.. note:: end effector는 직렬 체인으로 연결된 links로 구성되지 않습니다.
   따라서 그룹의 **Kinematic Solver** 는 **None** 으로 설정되어 있어야만 한다.

* 다음 단계를 수행합니다.

  * **Add Group** 버튼을 클릭합니다.

  * **Group Name** 을 ``hand`` 로 입력합니다.

  * **Kinematic Solver** 는 기본값인 **None** 으로 유지하십시오.

  * **Kin. Search Resolution** 와 **Kin. Search Timeout** 는 기본값으로 유지하십시오.

  * **Add Links** 버튼을 클립하세요.

  * ``panda_hand``, ``panda_leftfinger``, ``panda_rightfinger`` 를 선택하여 오른쪽 **Selected Links** 목록에 추가하십시오.

  * **Save** 를 클릭하세요.

.. image:: planning_groups/setup_assistant_panda_hand_group_links.png
   :width: 700px
   :align: center

두 팔과 손 그룹을 모두 추가한 후 커스텀 그룹 목록은 다음과 같이 표시됩니다.

.. image:: planning_groups/setup_assistant_panda_planning_groups_done.png
   :width: 700px
   :align: center


.. note:: **Add Subgroup** 옵션을 사용하여 다른 move groups으로 구성된 move groups을 만들 수 있습니다.
   이는 다중 팔 시스템의 동시 모션을 계획할 때와 같이 여러 move groups을 함께 제어해야 하는 경우에 유용합니다.

Step 5: Robot Poses 추가
---------------------------

Setup Assistant를 사용하면 로봇 설정에 사전 정의된 포즈를 추가할 수 있으며,
이는 특정 초기 포즈 또는 준비 포즈를 정의하는 데 유용합니다.
나중에 MoveIt API를 사용하여 로봇에게 이러한 포즈로 이동하도록 명령할 수 있습니다.

팔에 대한 ready pose 추가

* **Robot Poses** 창을 클릭합니다..

* **Add Pose** 를 클릭하세요. 포즈 이름을 선택하십시오. 로봇은 기본 포즈에 있을 것이며 모든 조인트는 0 값으로 설정됩니다. 원하는 위치까지 개별 조인트를 움직인 다음 **Save** 버튼을 클릭하여 포즈를 저장하십시오. 포즈가 특정 그룹과 연결되는 방식에 유의하십시오. 각 그룹에 대해 개별 포즈를 저장할 수 있습니다.

* ``panda_arm`` 을 선택하고 다음 조인트 값 ``{0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}`` 을 사용하여 ``ready`` 포즈를 정의하십시오.

* **IMPORTANT TIP**: 모든 조인트를 움직여 보십시오. URDF에서 조인트 제한에 문제가 있으면 여기에서 즉시 확인할 수 있습니다.

.. image:: predefined_poses/setup_assistant_panda_predefined_arm_pose.png
   :width: 700px

그리퍼에 대해서 열기/닫기 포즈를 추가

* arm에 대한 포즈를 정의하는 동일한 단계를 따르되 ``hand`` 그룹을 선택하십시오.

* ``open`` 포즈를 ``hand`` 그룹에 대해 조인트 값 ``0.035`` 로 추가하십시오.

.. image:: predefined_poses/setup_assistant_panda_predefined_hand_open_pose.png
   :width: 700px

* ``close`` 포즈를 ``hand`` 그룹에 대해 조인트 값 ``0.0`` 으로 추가하십시오.

.. image:: predefined_poses/setup_assistant_panda_predefined_hand_close_pose.png
   :width: 700px

.. note:: ``panda_finger_joint1`` 만 목록에 나타나는 이유는 ``panda_finger_joint2`` 은 ``panda_finger_joint1`` 를 따라하기 때문입니다.

위의 단계를 완료하면, ``panda_arm`` 및 ``hand`` 그룹에 대해 다음과 같은 로봇 포즈 세트가 정의되어야 합니다.

.. image:: predefined_poses/setup_assistant_panda_predefined_poses_done.png
   :width: 700px

Step 6: Label End Effectors
---------------------------

Panda 팔의 손을 move group으로 추가했으므로, 이제 End Effector로 지정할 수 있습니다.
그룹을 end effector로 지정하면, MoveIt는 이 그룹에 대해 특정한 작업을 수행할 수 있습니다.
예를 들어, end effector는 집어 올리기 및 놓기 태스크를 수행하는 동안 팔에 물체를 부착하는 데 사용할 수 있습니다.

* **End Effectors** 창을 클릭합니다.

* **Add End Effector** 를 클릭합니다.

* 그리퍼의 **End Effector Name** 으로 ``hand`` 를 선택합니다.

* **End Effector Group** 으로 ``hand`` 를 선택합니다.

* end-effector의 **Parent Link** 로 ``panda_link8`` 를 선택합니다.

* **Parent Group** 를 비워두세요.

.. image:: setup_assistant_panda_add_end_effector.png
   :width: 700px

Step 7: 패시브 조인트 추가
--------------------------

**Passive Joints** 창은 로봇에 존재할 수 있는 패시브 조인트를 지정할 수 있도록 설계되었습니다.
패시브 조인트는 구동 장치가 없는, 즉 직접 제어할 수 없는 조인트를 말합니다.
모션 플래너가 패시브 조인트의 존재를 인지하고 해당 조인트를 사용하는 경로 계획을 피하기 위해서는 반드시 이를 지정하는 것이 중요합니다.
플래너가 패시브 조인트를 인식하지 못하면, 패시브 조인트를 움직이는 경로를 계획하려고 시도할 수 있으며, 이는 유효하지 않은 계획을 초래합니다.
Panda 로봇 팔은 패인트 조인트가 없으므로 이 단계는 건너뛸 수 있습니다.

Step 8: ros2_control URDF 수정
--------------------------------------

**ros2_control URDF Modification** 창은 로봇의 URDF 파일을 `ros2_control <https://control.ros.org/master/index.html>`_ 과 함께 작동하도록 수정하는데 도움이 됩니다.

.. note:: 만약 로봇의 URDF/xacro 파일에 이미 ``ros2_control.xacro`` 태그가 포함되어 있다면, 이 단계는 건너뛸 수 있습니다.

이 수정은 정의된 move group의 각 조인트에 대한 명령 및 상태 인터페이스 태그를 추가합니다.
``command_interface`` 태그는 조인트를 제어하는데 사용할 수 있는 명령의 종류를 정의합니다.
``state_interface`` 태그는 조인트에서 읽을 수 있는 상태 정보의 종류를 정의합니다.

기본적으로 MoveIt Setup Assistant는 **position** 명령 인터페이스와 **position** 와 **velocity** 상태 인터페이스를 사용하며, 이 설정으로 진행합니다.

.. image:: ros2_control/setup_assistant_ros2_control_tags.png
   :width: 700px

필요한 경우 로봇 조인트에 대한 원하는 명령 또는 상태 인터페이스를 선택한 다음 **Add Interfaces** 버튼을 클릭하십시오.

Step 9: ROS 2 Controllers
-------------------------

ROS 2 컨트롤은 로봇의 실시간 제어를 위한 프레임워크이며, 새로운 로봇 하드웨어의 통합을 관리하고 단순화하도록 설계되었습니다.
보다 자세한 내용은 `ros2_control <https://control.ros.org/master/index.html>`_ 문서를 참조하십시오.

**ROS 2 Controllers** 창을 사용하여 로봇 조인트을 구동하는 시뮬레이션 컨트롤러를 자동 생성할 수 있습니다.

.. image:: ros2_controllers/setup_assistant_ros2_controllers.png
   :width: 700px

arm 컨트롤러 추가

* **ROS 2 Controllers** 창 선택기 클릭합니다.

* **Add Controller** 를 클릭하면 다음과 같은 화면이 나타납니다.:

* 먼저 Panda arm 조인트 궤적 컨트롤러를 추가합니다.

* **Controller Name** 에 ``panda_arm_controller`` 를 입력합니다.

* 컨트롤러 타입으로 **joint_trajectory_controller/JointTrajectoryController** 를 선택합니다.

.. image:: ros2_controllers/setup_assistant_panda_arm_ros2_controller_type.png
   :width: 700px

* 다음으로 컨트롤러 조인트를 선택해야 합니다. 조인트는 개별적으로 또는 move group별로 추가할 수 있습니다.

* 이제 **Add Planning Group Joints** 를 클릭합니다.

* **Available Groups** 탭에서 ``panda_arm`` 그룹을 선택하여 **Selected Groups** 에 추가합니다.

* 선택한 컨트롤러를 저장하려면 **Save** 을 클릭합니다.

.. image:: ros2_controllers/setup_assistant_panda_arm_ros2_controller_group.png
   :width: 700px

hand 컨트롤러 추가

* arm에 대한 것과 같은 단계를 따르지만, **position_controllers/GripperActionController** 을 선택합니다.

.. image:: ros2_controllers/setup_assistant_hand_ros2_controller_type.png
   :width: 700px

* **Available Groups** 탭에서 ``hand`` 그룹을 선택하여 **Selected Groups** 에 추가합니다.

* 선택한 컨트롤러를 저장하려면 **Save** 을 클릭합니다.

.. image:: ros2_controllers/setup_assistant_hand_ros2_controller_group.png
   :width: 700px

arm과 hand 컨트롤러를 선택한 후에는, 컨트롤러 목록이 다음과 같이 나타납니다.

.. image:: ros2_controllers/setup_assistant_ros2_controllers_done.png
   :width: 700px

Step 10: MoveIt 컨트롤러들
---------------------------

MoveIt은 계획된 궤적을 실행하기 위해 ``FollowJointTrajectoryAction`` 인터페이스를 가진 궤적 컨트롤러를 요구합니다.
이 인터페이스는 생성된 궤적을 로봇 ROS 2 컨트롤러에게 보냅니다.

**MoveIt Controllers** 창을 사용하여 MoveIt controller manager가 사용할 컨트롤러를 자동 생성할 수 있습니다.
이전 ROS 2 컨트롤러 설정 단계에서 설정한 컨트롤러 이름과 일치하는지 확인하십시오.
이 단계의 사용자 인터페이스는 이전 단계와 유사합니다.

.. image:: moveit_controllers/setup_assistant_moveit_controllers.png
   :width: 700px

arm MoveIt 컨트롤러 추가

* **MoveIt Controllers** 창 선택기를 클릭하십시오.

* **Add Controller** 를 클릭하여 새로운 arm 컨트롤러를 생성하십시오.

* **Controller Name** 에 ``panda_arm_controller`` 를 입력하십시오.

* 컨트롤러 유형에서 **FollowJointTrajectory** 를 선택하십시오.

* 컨트롤러 조인트에 ``panda_arm`` 플래닝 그룹의 선택하십시오.

* 컨트롤러를 저장합니다.

.. image:: moveit_controllers/setup_assistant_panda_arm_moveit_controller_type.png
   :width: 700px


hand MoveIt 컨트롤러 추가

* **Add Controller** 를 클릭하여 새로운 컨트롤러를 생성하십시오.

* **Controller Name** 을 ``hand_controller`` 로 입력하십시오.

* 컨트롤러 유형에서 **Gripper Command** 를 선택하십시오.

* 컨트롤러 조인트를 ``hand`` 플래닝 그룹으로 선택하십시오.

* 컨트롤러를 저장합니다.

.. image:: moveit_controllers/setup_assistant_hand_moveit_controller_type_gripper.png
   :width: 700px

이전 단계를 완료하면, arm과 hand에 대한 MoveIt 컨트롤러 목록이 다음과 같이 나타납니다.

.. image:: moveit_controllers/setup_assistant_moveit_controllers_done_gripper.png
   :width: 700px

Step 11: Perception
-------------------

Setup Assistant의 "Perception" 탭은 로봇에서 사용하는 3D 센서 구성을 설정하는데 사용됩니다. 이 설정은 **sensors_3d.yaml** 이라는 YAML 설정 파일에 저장됩니다.

**sensors_3d.yaml** 이 필요하지 않은 경우 **None** 을 선택하고 다음 단계로 진행하십시오.

.. image:: perception/setup_assistant_panda_3d_perception.png
   :width: 700px

**point_cloud** 구성 파라미터를 생성하려면 다음 예제를 참조하십시오.:

.. note:: This configuration is not valid for the Panda robot arm because it does not have a ``head_mounted_kinect`` camera.

.. image:: perception/setup_assistant_panda_3d_perception_point_cloud.png
   :width: 700px

해당 매개변수에 대한 자세한 내용은 :doc:`Perception Pipeline tutorial </doc/examples/perception_pipeline/perception_pipeline_tutorial>` 을 참조하십시오.

Step 12: Launch 파일
---------------------

**Launch Files** 창에서, 생성될 launch 파일 목록을 볼 수 있습니다.
기본 옵션은 일반적으로 충분하지만, 응용 프로그램에 대한 특정 요구 사항이 있는 경우 필요에 따라 변경할 수 있습니다.
각 파일을 클릭하여 해당 기능 요약을 확인하십시오.

.. image:: setup_assistant_launch_files.png
   :width: 700px

Step 13: 작성자 정보 추가
--------------------------------

colcon은 공개 목적으로 작성자 정보를 요구합니다.

* **Author Information** 창을 클릭하세요.
* 이름과 이메일 주소를 입력합니다.


Step 14: 설정 파일 생성
--------------------------------------

거의 완성되었습니다. 마지막 단계 - MoveIt 사용을 시작하는 데 필요한 모든 설정 파일을 생성하는 것입니다.

* **Configuration Files** 창을 클릭하십시오. 새롭게 설정 파일들을 포함해서 생성될 ROS 2 패키지의 위치와 이름을 선택하십시오. **Browse** 를 클릭하고 적절한 위치(예: ROS 2 워크스페이스의 src 디렉토리)를 선택한 다음 **Create Folder** 를 클릭하고 ``panda_moveit_config`` 라고 이름 짓고 **Open** 를 클릭하십시오. 생성된 모든 파일은 선택한 디렉토리로 직접 이동합니다.

* **Generate Package** 버튼을 클릭하십시오. 이제 Setup Assistant는 선택한 디렉토리에 일련의 launch 및 config 파일을 생성합니다. 생성된 모든 파일은 생성될 파일 탭에 나타나며 각 파일을 클릭하여 해당 기능에 대한 설명을 볼 수 있습니다. 생성된 파일에 대한 자세한 내용은 :doc:`the Configuration section in the documentation </doc/examples/examples>` 을 참조하십시오.

.. image:: setup_assistant_done.png
   :width: 700px

축하합니다! 이제 MoveIt에 필요한 설정 파일 생성을 완료했습니다.

panda_moveit_config 패키지를 빌드하고 데모 실행하기
----------------------------------------------------------
생성된 ``panda_moveit_config`` 패키지만 빌드하고 데모를 실행하려면 다음 단계를 따르세요. ::

   cd ~/ws_moveit2
   colcon build --packages-select panda_moveit_config
   source install/setup.bash

MoveIt 데모를 시작하여 RViz에서 로봇의 모션을 대화형으로 계획하고 실행합니다. ::

   ros2 launch moveit_resources_panda_moveit_config demo.launch.py

사전 정의한 ``ready`` 포즈로 이동하도록 로봇에게 명령하고 손을 ``open`` 과 ``close`` 모션을 하도록 하는 방법의 예제를 보려면 이 `간단한 YouTube 비디오 <https://youtu.be/f__udZlnTdM>`_ 확인하세요.

What's Next
-----------

RViz를 사용하여 MoveIt 모션 계획 시작하기

* 생성된 설정 파일을 사용하여 RViz에서 MoveIt으로 모션을 계획 및 시각화하는 방법을 배워보자.
  :doc:`MoveIt Quickstart in Rviz tutorial </doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial>` 을 참고하세요.

첫 번째 C++ MoveIt 응용 프로그램 작성

* 이 튜토리얼(:doc:`this tutorial </doc/tutorials/your_first_project/your_first_project>` )을 사용하여 MoveIt을 사용하는 첫 번째 C++ 응용 프로그램을 작성하고, :doc:`this example </doc/examples/move_group_interface/move_group_interface_tutorial>` 에서 제공하는 ``MoveGroupInterface`` 에 익숙해지며 이를 사용하여 로봇의 모션 계획, 실행 및 시각화를 수행하세요.

URDF vs SRDF: 차이점 이해

* 이 튜토리얼에서 언급된 URDF 및 SRDF 컴포넌트에 대한 자세한 내용은 :doc:`URDF and SRDF </doc/examples/urdf_srdf/urdf_srdf_tutorial>` 페이지를 참조하십시오.

사용 가능한 역 운동학 솔버 탐색

* 기본 KDL 솔버 외에도 대체 IK 솔버를 사용할 수 있습니다. 자세한 내용은 :doc:`IKFast </doc/examples/ikfast/ikfast_tutorial>` 와 `pick_ik <https://github.com/PickNikRobotics/pick_ik>`_ 를 참조하십시오.
