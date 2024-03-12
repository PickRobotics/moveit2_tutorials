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

Motion Planning Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For each type of motion planner plugin available in MoveIt, there is a corresponding ``config/*_planning.yaml`` file that describes its configuration.
For example, a robot that can use both :doc:`OMPL </doc/examples/ompl_interface/ompl_interface_tutorial>` and :doc:`Pilz Industrial Motion Planner </doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner>` will have the following folder structure:

.. code-block::

    my_robot_moveit_config
        config/
            ompl_planning.yaml
            pilz_industrial_motion_planner_planning.yaml
            ...
        ...

By default, all parameter files that match this ``config/*_planning.yaml`` pattern will be loaded.
If OMPL is configured as a planning pipeline, that will be the default; otherwise, it will be the first pipeline in the list.

To learn more about the contents of the individual planning configuration files, refer to the configuration documentation for those planners.

Trajectory Execution Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

MoveIt typically publishes manipulator motion commands to a `JointTrajectoryController <https://github.com/ros-controls/ros2_controllers/tree/master/joint_trajectory_controller>`_.
To learn more, refer to the :doc:`Low Level Controllers </doc/examples/controller_configuration/controller_configuration_tutorial>` section.

The default location for trajectory execution information is in ``config/moveit_controllers.yaml``.

MoveItCpp Configuration
^^^^^^^^^^^^^^^^^^^^^^^

If you are using :doc:`MoveItCpp </doc/examples/moveit_cpp/moveitcpp_tutorial>`, you can define a file with all the necessary parameters.

The default location of this file is in ``config/moveit_cpp.yaml``.

3D Perception Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you are using a perception sensor capable of generating 3D point clouds for motion planning, you can configure those settings for MoveIt.
For more information, refer to the :doc:`Perception Pipeline Tutorial </doc/examples/perception_pipeline/perception_pipeline_tutorial>`.

The default location of this file is in ``config/sensors_3d.yaml``.

Loading Configuration Parameters into Launch Files
--------------------------------------------------

To easily load parameters from MoveIt configuration packages for use in your ROS 2 launch files, MoveIt provides a ``MoveItConfigsBuilder`` utility.
To load the configuration parameters from your ``my_robot_moveit_config`` package:

.. code-block:: python

    from moveit_configs_utils import MoveItConfigsBuilder

    moveit_config = (
        MoveItConfigsBuilder("my_robot")
        .to_moveit_configs()
    )

Then, you can either use the complete set of configuration parameters when launching a node:

.. code-block:: python

    from launch_ros.actions import Node

    my_node = Node(
        package="my_package",
        executable="my_executable",
        parameters=[moveit_config.to_dict()],
    )

or you can include selected sub-components as follows:

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

Note that the above syntax will automatically look for configuration files that match the default file naming patterns described in this document.
If you have a different naming convention, you can use the functions available in ``MoveItConfigsBuilder`` to directly set file names.
For example, to use a non-default robot description and IK solver file path, and configure planning pipelines:

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

Now that you have read this page, you should be able to better understand the launch files available throughout the MoveIt 2 tutorials, and when encountering other MoveIt configuration packages in the wild.
