MoveIt에서 Launch 파일
==========================

많은 MoveIt 튜토리얼과 실제 사용되는 MoveIt 패키지는 ROS 2 launch 파일을 사용합니다.

이 튜토리얼은 동작하는 MoveIt 예제를 설정하는 전형적인 Python launch 파일에 대해서 다룹니다. 안내합니다.
:codedir:`Getting Started tutorial launch file <tutorials/quickstart_in_rviz/launch/demo.launch.py>` 를 상세히 살펴보면서 이 과정을 진행합니다.

launch 파일에 익숙하지 않은 경우, 먼저 `the ROS 2 documentation <https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Creating-Launch-Files.html>`_ 를 참조하십시오.

MoveIt 설정을 로드하기
--------------------------------

MoveIt는 로봇 서술 및 의미 서술 파일 (:ref:`URDF and SRDF`), 모션 계획 및 kinematics 플러그인, 궤적 실행 등을 사용하기 위해서 여러 설정 파라미터가 필요합니다.
이 파라미터는 :ref:`MoveIt Configuration` 패키지 내에 포함되어 있습니다.

파이썬 launch 파일에서 ``MoveItConfigsBuilder`` 유틸리티를 다음과 같이 사용하여 MoveIt 설정 패키지를 참조하는 간편한 방법이 있습니다.:

.. code-block:: python

    from moveit_configs_utils import MoveItConfigsBuilder

    # Define xacro mappings for the robot description file
    launch_arguments = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "true",
        "gripper": "robotiq_2f_85",
        "dof": "7",
    }

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config"
        )
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

Move Group을 Launch하기
-------------------------------

MoveIt 설정 파라미터를 모두 로드한 후 로드된 전체 MoveIt 파라미터 세트를 사용하여 :ref:`Move Group Interface` 를 launch할 수 있습니다.

.. code-block:: python

    from launch_ros.actions import Node

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

RViz로 시각화하기
---------------------

:ref:`Quickstart in RViz` 튜토리얼에서 설명했듯이, RViz를 사용하여 로봇 model을 시각화하고 모션 계획 태스크를 수행할 수 있습니다.

다음 코드는 launch 인자를 사용하여 RViz 설정 파일 이름을 받고, 이것을 알고 있는 패키지 디렉토리에 대한 상대 경로로 패키징한 다음 RViz 실행 프로그램을 launch할 때 인자로 지정합니다.

.. code-block:: python

    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
    from launch_ros.substitutions import FindPackageShare

    # Get the path to the RViz configuration file
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="kinova_moveit_config_demo.rviz",
        description="RViz configuration file",
    )
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("moveit2_tutorials"), "launch", rviz_base]
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

Transforms을 ``tf2`` 로 publish하기
-----------------------------------------

ROS 에코시스템의 다양한 도구들은 MoveIt와 모션 플래닝의 중요한 부분인 좌표 변환(coordinate transforms)을 표현하기 위해 `tf2 <https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Tf2.html>`_ 라이브러리를 사용합니다.

따라서, launch 파일에는 node들이 포함되어 있습니다. 이 node들은 ``tf2`` 에게 고정(정적) 변환과 동적 이름 모두를 publish합니다. 노드가 포함됩니다.
이 경우 다음과 같은 것들이 필요합니다:

* robot description의 기본 프레임인 ``base_link`` 와 ``world`` 프레임 간의 정적 변환
* `robot state publisher <https://github.com/ros/robot_state_publisher>`_ node는 로봇의 joint states를 수신하고 로봇의 URDF 모델을 사용하여 프레임 변환(frame transforms)을 계산하고 이것들을 ``tf2`` 로 publish함

.. code-block:: python

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

궤적 실행을 위해 ``ros2_control`` 설정하기
----------------------------------------------------

MoveIt는 일반적으로 관절 궤적을 생성해서 이 궤적을 실행할 수 있는 로봇 제어기에게 전송해서 실행시킬 수 있습니다.
가장 일반적으로 `ros2_control <https://control.ros.org/master/index.html>`_ 라이브러리에 연결하여 이를 수행합니다.

``ros2_control`` 은 실제 로봇 하드웨어 또는 Gazebo나 NVIDIA Isaac Sim과 같은 물리 기반 시뮬레이터의 로봇에 연결할 수 있지만, 간단하고 이상적인 시뮬레이션을 위한 가짜 구성 요소 `mock components <https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html>`_ 기능도 제공합니다.
우리 예제에서는 이전에 정의된 ``use_fake_hardware`` xacro 파라미터를 사용하여 URDF 레벨에서 이를 설정합니다.
핵심적인 아이디어는 실행되는 하드웨어(시뮬레이션 또는 실제)에 관계없이 ``ros2_control`` launch가 동일하다는 점입니다.

``ros2_control`` 를 구동시키는 것은 controller manager node를 실행한 다음, 궤적 실행에 필요한 컨트롤러들의 목록을 생성하는 것을 포함합니다.
우리 예제에서는 다음과 같은 컨트롤러가 있습니다.:

* joint state broadcaster는 robot state publisher가 프레임을 ``tf2`` 에게 전송하는데 필요한 joint states를 publish함
* arm 액츄에이터용 joint 궤적 제어기
* 평행-jaw 그리퍼용 그리퍼 액션 제어기(gripper action controller)

.. code-block:: python

    ros2_controllers_path = os.path.join(
        get_package_share_directory("kinova_gen3_7dof_robotiq_2f_85_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
    )

모든 node들 launch하기
-----------------------

마지막으로, 이전 섹션에서 설명한 모든 것을 실제로 lauch하도록 launch 파일에게 지시할 수 있습니다.

.. code-block:: python

    # ... all our imports go here

    def generate_launch_description():

        # ... all our other code goes here

        return LaunchDescription(
            [
                rviz_config_arg,
                rviz_node,
                static_tf,
                robot_state_publisher,
                run_move_group_node,
                ros2_control_node,
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
                hand_controller_spawner,
            ]
        )
