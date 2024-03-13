.. _MoveIt Configuration:

MoveIt 설정
==================================

여러분의 로봇에 대한 MoveIt 설정 권장 방법은 MoveIt 설정을 포함하는 colcon 패키지를 만드는 것입니다.

``my_robot`` 이라는 이름의 로봇에 대한 설정 패키지를 만들고 싶다고 가정해봅시다.
이를 위해서는  ``my_robot_moveit_config`` 라는 이름의 colcon 패키지를 생성할 수 있습니다. 이 패키지의 구조는 다음과 같습니다.:

.. code-block::

    my_robot_moveit_config
        config/
            kinematics.yaml
            joint_limits.yaml
            *_planning.yaml
            moveit_controllers.yaml
            moveit_cpp.yaml
            sensors_3d.yaml
            ...
        launch/
        .setup_assistant
        CMakeLists.txt
        package.xml

이러한 패키지는 직접 만들 수도 있고,  :doc:`MoveIt Setup Assistant </doc/examples/setup_assistant/setup_assistant_tutorial>` 를 사용하여 URDF 또는 xacro 로봇 서술 파일을 기반으로 자동 생성할 수도 있습니다.

실제 MoveIt 설정 패키지 예제는 `moveit_resources <https://github.com/ros-planning/moveit_resources/tree/ros2>`_ 리포지토리에서 참조할 수 있습니다.


설정 파일 개요
----------------------------

MoveIt 설정 패키지의 ``config/`` 폴더에는 MoveIt의 다양한 기능에 대한 파라미터를 설명하는 여러 파일을 포함하고 있습니다.

여러 파일은 런타임시에 필요한 기능에 따라 선택적으로 의존하고 있다는 점에 유의하세요.

Robot Description
^^^^^^^^^^^^^^^^^

이것이 MoveIt 설정 패키지에서 가장 중요한 정보입니다.
이 폴더에는 로봇의 운동학, 계획 그룹, 충돌 규칙 등을 설명하는 URDF 및 SRDF 파일이 반드시 있어야 합니다.
이러한 파일정보는 :doc:`URDF/SRDF Overview </doc/examples/urdf_srdf/urdf_srdf_tutorial>` 를 참조하십시오.

Joint Limits
^^^^^^^^^^^^

URDF 파일 스펙을 사용하면 joint 위치 및 속도 제한을 설정할 수 있습니다.
하지만 기본 로봇 서술 파일을 수정하지 않고 MoveIt으로 모션 계획에 대해서 다른 joint limits(조인트 제약)을 정의할 수도 있습니다.
더욱이 MoveIt의 일부 기능에서는 URDF에서 지정할 수 없는 가속도 및 저크 제한(jerk limits)과 같은 추가적인 관절 제한 유형(types of joint limits)을 사용합니다.

이 파일의 기본 위치는 ``config/joint_limits.yaml`` 입니다.

2개 joints를 가지는 단순한 로봇의 joint limits 예제 일부는 아래와 같습니다.:

.. code-block:: yaml

    joint_limits:
        joint1:
            has_velocity_limits: true
            max_velocity: 2.0
            has_acceleration_limits: true
            max_acceleration: 4.0
            has_jerk_limits: false
        panda_joint2:
            has_velocity_limits: true
            max_velocity: 1.5
            has_acceleration_limits: true
            max_acceleration: 3.0
            has_jerk_limits: false

Inverse Kinematics (IK) Solver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

MoveIt의 많은 모션 계획 응용 프로그램은 inverse kinematics을 풀어야 합니다.풀 필요가 있습니다.

사용하는 IK solver 플러그인과 해당 매개 변수는 ``config/kinematics.yaml`` 파일을 통해 설정합니다.

자세한 내용은 :doc:`Kinematics Configuration </doc/examples/kinematics_configuration/kinematics_configuration_tutorial>` 을 참조하십시오.

Motion Planning 설정
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

MoveIt에서 사용할 수 있는 motion planner 플러그인 종류마다 관련 설정은 ``config/*_planning.yaml`` 파일에 존재합니다.
예를 들어, :doc:`OMPL </doc/examples/ompl_interface/ompl_interface_tutorial>` 와 :doc:`Pilz Industrial Motion Planner </doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner>` 모두 사용 가능한 로봇의 경우 아래와 같은 폴더 구조를 가집니다.:

.. code-block::

    my_robot_moveit_config
        config/
            ompl_planning.yaml
            pilz_industrial_motion_planner_planning.yaml
            ...
        ...

기본적으로 이러한 ``config/*_planning.yaml`` 패턴에 일치하는 모든 파라미터 파일이 로드됩니다.
별도의 설정이 없다면 OMPL이 기본 planning pipeline으로 설정됩니다. 그렇지 않은 경우에는 목록의 첫 번째 파이프라인이 기본값으로 설정됩니다.

개별 planning 설정 파일의 내용에 대해 자세히 알아보려면 해당 planner의 설정 문서를 참조하십시오.

궤적(Trajectory) 실행 설정
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

MoveIt은 일반적으로 `JointTrajectoryController <https://github.com/ros-controls/ros2_controllers/tree/master/joint_trajectory_controller>`_ 에게 manipulator motion command를 publish합니다.
자세한 내용은 :doc:`Low Level Controllers </doc/examples/controller_configuration/controller_configuration_tutorial>` 섹션을 참조하십시오.

궤적 실행 정보에 대한 기본 위치는 ``config/moveit_controllers.yaml`` 내에 있습니다.

MoveItCpp 설정
^^^^^^^^^^^^^^^^^^^^^^^

만약 :doc:`MoveItCpp </doc/examples/moveit_cpp/moveitcpp_tutorial>` 을 사용하고 있다면, 필요한 모든 매개변수를 포함하는 파일을 정의할 수 있습니다.

이 파일의 기본 위치는 ``config/moveit_cpp.yaml`` 입니다.

3D Perception 설정
^^^^^^^^^^^^^^^^^^^^^^^^^^^

만약 모션 계획을 위해 3D point cloud를 생성할 수 있는 지각 센서(perception sensor)를 사용하고 있다면, MoveIt에 대한 설정값들을 설정할 수 있습니다.
더 자세한 정보는 :doc:`Perception Pipeline Tutorial </doc/examples/perception_pipeline/perception_pipeline_tutorial>` 을 참조하세요.

이 파일의 기본 위치는 ``config/sensors_3d.yaml`` 입니다.

Loading Configuration Parameters into Launch Files
--------------------------------------------------

ROS 2 launch 파일내에서 사용 목적으로 MoveIt 설정 패키지의 매개변수를 쉽게 로드하기 위해 MoveIt은 ``MoveItConfigsBuilder`` 유틸리티를 제공합니다.
``my_robot_moveit_config`` 패키지에서 설정 매개변수를 로드하려면 다음과 같이 하십시오.:

.. code-block:: python

    from moveit_configs_utils import MoveItConfigsBuilder

    moveit_config = (
        MoveItConfigsBuilder("my_robot")
        .to_moveit_configs()
    )

다음으로 node를 launch할때, 전체 설정 매개 변수 세트를 사용할 수도 있습니다.:

.. code-block:: python

    from launch_ros.actions import Node

    my_node = Node(
        package="my_package",
        executable="my_executable",
        parameters=[moveit_config.to_dict()],
    )

혹은 선택한 서브-컴포넌트를 아래와 같이 선택할 수 있습니다.:

.. code-block:: python

    from launch_ros.actions import Node

    my_node = Node(
        package="my_package",
        executable="my_executable",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

위 문법으로 이 문서에서 설명하는 기본 파일 이름 패턴과 일치하는지를 설정 파일을 자동으로 검색합니다.
여러분이 다른 명명 규칙을 가지고 있는 경우라면, ``MoveItConfigsBuilder`` 에 함수를 사용하여 파일 이름을 직접 설정할 수 있습니다.
예를 들어, 기본값이 아닌 robot description와 IK solver 파일 경로를 사용하고, planning pipelines을 설정하려면 다음과 같이 하십시오.:

.. code-block:: python

    from moveit_configs_utils import MoveItConfigsBuilder

    moveit_config = (
        MoveItConfigsBuilder("my_robot")
        .robot_description(file_path="config/my_robot.urdf.xacro")
        .robot_description_kinematics(file_path="config/my_kinematics_solver.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="pilz_industrial_motion_planner",
        )
        .to_moveit_configs()
    )

이 페이지를 읽었기 때문에 MoveIt 2 튜터리얼 전체에서 사용하는 launch 파일을 더 잘 이해할 수 있게 되었고 다른 MoveIt 설정 패키지를 보더라도 이해할 수 있게 되었습니다.
