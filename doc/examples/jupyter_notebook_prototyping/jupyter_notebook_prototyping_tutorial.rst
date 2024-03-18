Jupyter Notebook 프로토타이핑
==================================
.. raw:: html

        <iframe width="560" height="315" src="https://www.youtube.com/embed/7KvF7Dj7bz0" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

In this tutorial you will learn how to use Jupyter notebooks with the MoveIt 2 Python API. This tutorial is broken down into the following sections:

* **시작하기:** 튜터리얼 설정 요구사항 개요
* **launch 파일 이해:** launch 파일 스펙 개요
* **Notebook 설정:** Notebook imports and configuration.
* **Motion Planning 예제:** 모션 계획에 ``moveit_py`` API 사용 예제
* **Teleoperation 예제:** 조이스틱으로 로봇 원격제어에 ``moveit_py`` API 사용 예제

이 튜터리얼의 코드는 `여기 <https://github.com/peterdavidfagan/moveit2_tutorials/tree/moveit_py_notebook_tutorial/doc/examples/jupyter_notebook_prototyping>`_ 를 참고하세요.

시작하기
---------------
이 튜터리얼을 완료하려면, MoveIt 2 및 관련 튜터리얼을 포함하는 colcon 워크스페이스를 설정해야 합니다. 이러한 워크스페이스를 설정하는 방법에 대한 훌륭한 개요는 :doc:`Getting Started Guide </doc/tutorials/getting_started/getting_started>` 에서 제공됩니다.

일단 워크스페이스를 설정한 후, 아래 명령을 실행하여 이 튜터리얼의 코드를 실행할 수 있습니다 (이 튜터리얼의 서보 섹션에는 PS4 Dualshock가 필요합니다. PS4 Dualshock가 없는 경우 이 파라미터를 false로 설정하세요.): ::

        ros2 launch moveit2_tutorials jupyter_notebook_prototyping.launch.py start_servo:=true

+ 이 튜터리얼을 완료하는데 필요한 노드들을 launch 시킵니다.

+ 중요하게는, jupyter notebook 서버를 실행시켜 튜터리얼 과정 코드를 실행할 수 있게 해줍니다.

+ 만약 브라우저가 자동으로 jupyter 노트북 인터페이스를 열지 않는다면, 브라우저에서 http://localhost:8888 주소로 이동하여 연결할 수 있습니다.

+ 이때 launch 파일을 launch시킬 때 터미널에서 토큰을 입력하라는 메시지가 나타날 것이며, 이 토큰을 입력하여 notebook 서버에 연결할 수 있습니다.

+ 또한 터미널 출력에 토큰이 포함된 URL도 함께 출력되므로, 이 URL을 직접 사용하면 토큰을 수동 입력하지 않고 notebook 서버에 연결할 수도 있습니다.

이러한 단계를 완료하면 튜터리얼을 과정을 진행할 수 있습니다. notebook 코드를 실행하기 전에 launch 파일 사양에 대한 간략한 설명을 제공하여 notebook 인스턴스를 실행하는 방법을 이해할 수 있도록 합니다.


launch 파일 이해
--------------------------------
이 튜터리얼에서 사용되는 `launch file <https://github.com/peterdavidfagan/moveit2_tutorials/blob/moveit_py_notebook_tutorial/doc/examples/jupyter_notebook_prototyping/launch/jupyter_notebook_prototyping.launch.py>`_ 은 다른 튜터리얼과 비교했을 때 jupyter notebook 서버를 구동시키는 것이 차이점입니다.
여기서는 일반적인 launch 파일 코드를 간략하게 살펴보고 주로 notebook 서버를 구동시키는 것에  초점을 맞추도록 하겠습니다.

필요한 패키지를 import하기: ::

        import os
        import yaml
        from launch import LaunchDescription
        from launch.actions import ExecuteProcess, DeclareLaunchArgument
        from launch.substitutions import LaunchConfiguration
        from launch_ros.actions import Node, SetParameter
        from ament_index_python.packages import get_package_share_directory
        from moveit_configs_utils import MoveItConfigsBuilder

yaml 파일을 로딩하는 유틸리티 정의: ::

        def load_yaml(package_name, file_path):
                package_path = get_package_share_directory(package_name)
                absolute_file_path = os.path.join(package_path, file_path)

                try:
                        with open(absolute_file_path, 'r') as file:
                        return yaml.safe_load(file)
                except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
                        return None


서보 노드 시작을 위해 launch 인자 정의: ::

        start_servo = LaunchConfiguration('start_servo')

        start_servo_arg = DeclareLaunchArgument(
                'start_servo',
                default_value='false',
                description='Start the servo node.')

MoveIt 설정 정의 (이 단계는 나중에 notebook을 구성할 때도 중요합니다.): ::

        moveit_config = (
                MoveItConfigsBuilder(
                        robot_name="panda", package_name="moveit_resources_panda_moveit_config"
                )
                .robot_description(file_path="config/panda.urdf.xacro")
                .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
                .moveit_cpp(
                        file_path=os.path.join(
                                get_package_share_directory("moveit2_tutorials"),
                                "config",
                                "jupyter_notebook_prototyping.yaml"
                        )
                )
                .to_moveit_configs()
        )

MoveIt 설정이 정의되면 다음 노드들 세트를 시작합니다:

* **rviz_node:** 시각화를 위해 rviz2를 시작합니다.
* **static_tf:** world 프레임과 ;anda base 프레임 간의 정적 변환을 publish합니다.
* **robot_state_publisher:** 업데이트된 로봇 상태 정보(변환)를 publish됩니다.
* **ros2_control_node:** 조인트 그룹을 제어하는 데 사용됩니다.

::

        rviz_config_file = os.path.join(
                get_package_share_directory("moveit2_tutorials"),
                "config", "jupyter_notebook_prototyping.rviz",
        )
        rviz_node = Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_config_file],
                parameters=[
                        moveit_config.robot_description,
                        moveit_config.robot_description_semantic,
                ],
        )

        static_tf = Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                output="log",
                arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
        )

        robot_state_publisher = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="both",
                parameters=[moveit_config.robot_description],
        )

        ros2_controllers_path = os.path.join(
                get_package_share_directory("moveit_resources_panda_moveit_config"),
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

        load_controllers = []
        for controller in [
                "panda_arm_controller",
                "panda_hand_controller",
                "joint_state_broadcaster",
        ]:
                load_controllers += [
                        ExecuteProcess(
                        cmd=["ros2 run controller_manager spawner {}".format(controller)],
                        shell=True,
                        output="screen",)
                        ]

각 노드에 대한 설정을 정의한 후, jupyter 노트북 서버를 시작하는 프로세스도 정의합니다.: ::

        notebook_dir = os.path.join(get_package_share_directory("moveit2_tutorials"), "src")
        start_notebook = ExecuteProcess(
                cmd=["cd {} && python3 -m notebook".format(notebook_dir)],
                shell=True,
                output="screen",
        )


서보를 시작하려는 경우 조이스틱 및 서보 노드도 정의합니다. 마지막으로  ``LaunchDescription`` 을 반환합니다.: ::

        if start_servo:
                servo_yaml = load_yaml("moveit_servo", "config/panda_simulated_config.yaml")
                servo_params = {"moveit_servo": servo_yaml}

                joy_node = Node(
                        package="joy",
                        executable="joy_node",
                        name="joy_node",
                        output="screen",
                )

                servo_node = Node(
                        package="moveit_servo",
                        executable="servo_node_main",
                        parameters=[
                                servo_params,
                                moveit_config.robot_description,
                                moveit_config.robot_description_semantic,
                                moveit_config.robot_description_kinematics,
                        ],
                        output="screen",
                )

                return LaunchDescription(
                        [
                        start_servo_arg,
                        start_notebook,
                        static_tf,
                        robot_state_publisher,
                        rviz_node,
                        ros2_control_node,
                        joy_node,
                        servo_node,
                ]
                + load_controllers
                )

서보를 시작하지 않는 겨우, 우리가 정의한 모든 nodes와 프로세스들을 포함하는 launch description을 반환합니다. : ::

        return LaunchDescription(
                [
                start_servo_arg,
                static_tf,
                robot_state_publisher,
                rviz_node,
                ros2_control_node,
                start_notebook,
                ]
                + load_controllers
                )

Notebook 설정
--------------
우리가 jupyter 노트북 서버를 실행했으니, 이제 notebook에서 코드 실행을 시작할 수 있습니다. 첫 번째 단계는 필요한 패키지를 가져오는 것입니다.: ::

        import os
        import sys
        import yaml
        import rclpy
        import numpy as np

        # message libraries
        from geometry_msgs.msg import PoseStamped, Pose

        # moveit_py
        from moveit.planning import MoveItPy
        from moveit.core.robot_state import RobotState

        # config file libraries
        from moveit_configs_utils import MoveItConfigsBuilder
        from ament_index_python.packages import get_package_share_directory

필요한 패키지를 가져왔으면, 이제 ``moveit_py`` node 구성을 정의해야 합니다. 이를 위해 다음과 같이 ``MoveItConfigsBuilder`` 를 사용합니다.: ::

        moveit_config = (
                MoveItConfigsBuilder(robot_name="panda", package_name="moveit_resources_panda_moveit_config")
                .robot_description(file_path="config/panda.urdf.xacro")
                .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
                .moveit_cpp(
                        file_path=os.path.join(
                                get_package_share_directory("moveit2_tutorials"),
                                "config",
                                "jupyter_notebook_prototyping.yaml",
                        )
                )
                .to_moveit_configs()
        ).to_dict()

여기서 생성된 설정 인스턴스를 사전으로 변환하여 ``moveit_py`` node 노드를 초기화하는 데 사용합니다. 마지막으로 ``moveit_py`` node 를 초기화합니다.: ::

        # initialise rclpy (only for logging purposes)
        rclpy.init()

        # instantiate moveit_py instance and a planning component for the panda_arm
        panda = MoveItPy(node_name="moveit_py", config_dict=moveit_config)
        panda_arm = panda.get_planning_component("panda_arm")

Motion Planning 예제
-----------------------
먼저, 나중에 계획된 궤적을 계획하고 실행할 때 사용할 helper 함수를 만듭니다.: ::

        def plan_and_execute(
                robot,
                planning_component,
                single_plan_parameters=None,
                multi_plan_parameters=None,
        ):
                """A helper function to plan and execute a motion."""
                # plan to goal
                if multi_plan_parameters is not None:
                        plan_result = planning_component.plan(
                                multi_plan_parameters=multi_plan_parameters
                        )
                elif single_plan_parameters is not None:
                        plan_result = planning_component.plan(
                        single_plan_parameters=single_plan_parameters
                )
                else:
                        plan_result = planning_component.plan()

                # execute the plan
                if plan_result:
                        robot_trajectory = plan_result.trajectory
                        robot.execute(robot_trajectory, controllers=[])
                else:
                        print("Planning failed")

notebook 내에서 간단한 모션의 계획 및 실행을 시연하는 것부터 시작할 수 있습니다.: ::

        # set plan start state using predefined state
        panda_arm.set_start_state("ready")

        # set pose goal using predefined state
        panda_arm.set_goal_state(configuration_name = "extended")

        # plan to goal
        plan_and_execute(panda, panda_arm)

대화식으로 모션 플래닝을 수행할 수 있습니다.(보다 상세한 모션 플래닝 API 정보는 모션 플래닝 튜토리얼을 참조하십시오.) 코드를 개발하다가 다음과 같은 실수를 했을 경우를 생각해봅시다.: ::

        # set plan start state using predefined state
        panda_arm.set_start_state("ready") # This conflicts with the current robot configuration and will cause an error

        # set goal using a pose message this time
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "panda_link0"
        pose_goal.pose.orientation.w = 1.0
        pose_goal.pose.position.x = 0.28
        pose_goal.pose.position.y = -0.2
        pose_goal.pose.position.z = 0.5
        panda_arm.set_goal_state(pose_stamped_msg = pose_goal, pose_link = "panda_link8")

        # plan to goal
        plan_and_execute(panda, panda_arm)

notebook을 사용하고 있기 때문에 파일을 다시 컴파일하지 않고도 이러한 실수를 쉽게 수정할 수 있습니다. 위의 notebook을 아래 내용과 일치하도록 간단히 편집하고 셀을 다시 실행하십시오.: ::

        # set plan start state using predefined state
        panda_arm.set_start_state_to_current_state()

        # set goal using a pose message this time
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "panda_link0"
        pose_goal.pose.orientation.w = 1.0
        pose_goal.pose.position.x = 0.28
        pose_goal.pose.position.y = -0.2
        pose_goal.pose.position.z = 0.5
        panda_arm.set_goal_state(pose_stamped_msg = pose_goal, pose_link = "panda_link8")

        # plan to goal
        plan_and_execute(panda, panda_arm)

Teleoperation 예제
---------------------

또한 로봇을 실시간으로 원격 조종하기를 원할 수도 있습니다. 파이썬 API를 사용하면 모든 프로세스를 종료하고 다시 시작하지 않고도 대화 형식으로 원격 조종을 시작/중지할 수 있습니다. 이 예제에서는 notebook을 통해 로봇을 원격 조종하고 모션 플래닝을 수행하며 다시 로봇을 원격 조종하는 방법을 보여드리겠습니다.

이 섹션에서는 ``moveit_py``를 사용하여 원격 조종을 지원하는 장치가 필요합니다. 이 경우에는 PS4 듀얼쇼크 컨트롤러를 사용합니다.

로봇 원격 조종을 시작하려면 PS4 듀얼쇼크 컨트롤러를 원격 조종 장치로 인스턴스화합니다.: ::

        from moveit.servo_client.devices.ps4_dualshock import PS4DualShockTeleop

        # instantiate the teleoperating device
        ps4 = PS4DualShockTeleop(ee_frame_name="panda_link8")

        # start teleloperating the robot
        ps4.start_teleop()

로봇을 기본 구성으로 되돌리기 위한 모션 플래닝을 수행하려면, 로봇 원격 조종을 중지하고 아래와 같이 기존 모션 플래닝 API를 활용하기만 하면 됩니다: ::

        # stop teleoperating the robot
        ps4.stop_teleop()

        # plan and execute
        # set plan start state using predefined state
        panda_arm.set_start_state_to_current_state()

        # set pose goal using predefined state
        panda_arm.set_goal_state(configuration_name = "ready")

        # plan to goal
        plan_and_execute(panda, panda_arm)

이렇게 하면 로봇이 기본 구성으로 되돌아갑니다. 이 구성에서 다시 로봇 원격 조종을 시작할 수 있습니다.: ::

        ps4.start_teleop()
