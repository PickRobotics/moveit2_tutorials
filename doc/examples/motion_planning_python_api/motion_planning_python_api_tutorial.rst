Motion Planning Python API
==================================

.. raw:: html

        <iframe width="560" height="315" src="https://www.youtube.com/embed/7KvF7Dj7bz0" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

이 튜터리얼에서 모션 플래닝 API ``moveit_py`` 의 기본 사항을 다루고 있습니다.
이 튜토리얼은 다음과 같은 섹션으로 구성되어 있습니다:

* **시작하기:** 튜터리얼 설정 요구사항 개요
* **Planning 파라미터 이해:** 지원되는 모션 플래너의 파라미터 설정 개요
* **단일 파이프라인 Planning (기본 설정):** 미리 정의된 로봇 구성을 사용하여 플래닝
* **단일 파이프라인 Planning (Robot State):** 로봇 상태 인스턴스를 사용하여 플래닝
* **단일 파이프라인 Planning (Pose Goal):** 포즈 목표를 사용하여 플래닝
* **단일 파이프라인 Planning (커스텀 Constraints):** 커스텀 제약 조건을 사용하는 플래닝
* **다중 파이프라인 Planning:** 여러 플래닝 파이프라인을 병렬로 실행
* **Planning Scene 사용하기:** 충돌 물체 추가/제거와 충돌 검사

:doc:`/doc/examples/moveit_cpp/moveitcpp_tutorial` and
:doc:`/doc/examples/move_group_interface/move_group_interface_tutorial`

이 튜터리얼의 코드는 :codedir:`moveit2_tutorials GitHub project<examples/motion_planning_python_api>` 에서 찾을 수 있습니다.

시작하기
-----------------------------------------------
이 튜토리얼을 완료하려면 MoveIt 2 및 해당 튜토리얼을 포함하는 워크스페이스를 설정해야만 합니다.
워크스페이스를 설정하는 방법은 :doc:`Getting Started Guide </doc/tutorials/getting_started/getting_started>` 에 설명되어 있으며, 자세한 내용은 이 가이드를 참조하십시오.

워크스페이스를 설정한 후, 다음 명령을 실행하여 이 튜토리얼의 코드를 실행할 수 있습니다: ::

        ros2 launch moveit2_tutorials motion_planning_python_api_tutorial.launch.py

Planning 파라미터 이해
----------------------------------------------------
MoveIt은 기본적으로 여러 모션 플래닝 라이브러리를 지원하며, 사용하려는 플래너에 대한 설정/파라미터를 제공하는 것이 중요합니다.

이를 위해 우리는 ``moveit_py`` node와 관련된 파라미터를 정의하는 yaml 설정 파일을 지정합니다.

설정 파일 예제는 다음과 같습니다.: ::

        planning_scene_monitor_options:
                name: "planning_scene_monitor"
                robot_description: "robot_description"
                joint_state_topic: "/joint_states"
                attached_collision_object_topic: "/moveit_cpp/planning_scene_monitor"
                publish_planning_scene_topic: "/moveit_cpp/publish_planning_scene"
                monitored_planning_scene_topic: "/moveit_cpp/monitored_planning_scene"
                wait_for_initial_state_timeout: 10.0

        planning_pipelines:
                pipeline_names: ["ompl", "pilz_industrial_motion_planner", "chomp", "ompl_rrt_star"]

        plan_request_params:
                planning_attempts: 1
                planning_pipeline: ompl
                max_velocity_scaling_factor: 1.0
                max_acceleration_scaling_factor: 1.0

        ompl_rrtc:
                plan_request_params:
                        planning_attempts: 1
                        planning_pipeline: ompl
                        planner_id: "RRTConnectkConfigDefault"
                        max_velocity_scaling_factor: 1.0
                        max_acceleration_scaling_factor: 1.0
                        planning_time: 1.0

        ompl_rrt_star:
                plan_request_params:
                        planning_attempts: 1
                        planning_pipeline: ompl_rrt_star # Different OMPL pipeline name!
                        planner_id: "RRTstarkConfigDefault"
                        max_velocity_scaling_factor: 1.0
                        max_acceleration_scaling_factor: 1.0
                        planning_time: 1.5

        pilz_lin:
                plan_request_params:
                        planning_attempts: 1
                        planning_pipeline: pilz_industrial_motion_planner
                        planner_id: "PTP"
                        max_velocity_scaling_factor: 1.0
                        max_acceleration_scaling_factor: 1.0
                        planning_time: 0.8

        chomp:
                plan_request_params:
                        planning_attempts: 1
                        planning_pipeline: chomp
                        max_velocity_scaling_factor: 1.0
                        max_acceleration_scaling_factor: 1.0
                        planning_time: 1.5


설정 파일의 첫 번째 블록은 subscribe하는 topic과 같은 플래닝 씬 모니터 옵션(planning scene monitor option)을 설정합니다 (참고: 플래닝 씬 모니터에 익숙하지 않은 경우 :doc:`this tutorial </doc/examples/planning_scene_monitor/planning_scene_monitor_tutorial>` 을 살펴보세요): ::

        planning_scene_monitor_options:
                name: "planning_scene_monitor"
                robot_description: "robot_description"
                joint_state_topic: "/joint_states"
                attached_collision_object_topic: "/moveit_cpp/planning_scene_monitor"
                publish_planning_scene_topic: "/moveit_cpp/publish_planning_scene"
                monitored_planning_scene_topic: "/moveit_cpp/monitored_planning_scene"
                wait_for_initial_state_timeout: 10.0

설정 파일의 두 번째 블록은 사용하고자 하는 계획 파이프라인을 설정합니다.
MoveIt은 OMPL, Pilz 산업용 모션 플래너, 확률론적 궤적 최적화 모션 플래너 (STOMP), 검색 기반 계획 라이브러리 (SBPL), 공분산 하밀토니안 최적화 모션 플래너 (CHOMP) 등 여러 모션 계획 라이브러리를 지원합니다.
``moveit_py`` node를 구성할 때, 사용하려는 계획 파이프라인의 설정을 지정해야 합니다.: ::

        planning_pipelines:
                pipeline_names: ["ompl", "pilz_industrial_motion_planner", "chomp", "ompl_rrt_star"]

이러한 이름이 지정된 파이프라인 각각에 대해서는 planner_id 및 계획 시도 횟수와 같은 다른 설정값을 통해 사용할 플래너를 식별하는 설정을 제공해야 합니다.: ::

        ompl_rrtc:
                plan_request_params:
                        planning_attempts: 1
                        planning_pipeline: ompl
                        planner_id: "RRTConnectkConfigDefault"
                        max_velocity_scaling_factor: 1.0
                        max_acceleration_scaling_factor: 1.0
                        planning_time: 0.5

        ompl_rrt_star:
                plan_request_params:
                        planning_attempts: 1
                        planning_pipeline: ompl_rrt_star
                        planner_id: "RRTstarkConfigDefault"
                        max_velocity_scaling_factor: 1.0
                        max_acceleration_scaling_factor: 1.0
                        planning_time: 1.5

        pilz_lin:
                plan_request_params:
                        planning_attempts: 1
                        planning_pipeline: pilz_industrial_motion_planner
                        planner_id: "PTP"
                        max_velocity_scaling_factor: 1.0
                        max_acceleration_scaling_factor: 1.0
                        planning_time: 0.8

        chomp:
                plan_request_params:
                        planning_attempts: 1
                        planning_pipeline: chomp
                        max_velocity_scaling_factor: 1.0
                        max_acceleration_scaling_factor: 1.0
                        planning_time: 1.5

지정된 이러한 파라미터는 ``moveit_py`` node 파라미터로 사용할 수 있으며, 런타임에서 계획을 수행할 때 활용됩니다.
바로 이 부분을 다음으로 살펴보겠습니다.

moveit_py 와 planning 컴포넌트를 인스턴스화하기
----------------------------------------------------
모션을 계획하기 전에, ``moveit_py`` node와 그 파생된 계획 컴포넌트를 인스턴스화해야 합니다.
또한 ``rclpy`` 로거 객체도 인스턴스화할 것입니다.: ::

        rclpy.init()
        logger = rclpy.logging.get_logger("moveit_py.pose_goal")

        # instantiate MoveItPy instance and get planning component
        panda = MoveItPy(node_name="moveit_py")
        panda_arm = panda.get_planning_component("panda_arm")
        logger.info("MoveItPy instance created")

``panda_arm`` 변수로 표현되는 계획 컴포넌트를 사용하여, 모션 계획을 시작할 수 있습니다.
또한 모션을 계획 및 실행을 위해서 helper 함수도 정의합니다.: ::

        def plan_and_execute(
                robot,
                planning_component,
                logger,
                single_plan_parameters=None,
                multi_plan_parameters=None,
                ):
                """A helper function to plan and execute a motion."""
                # plan to goal
                logger.info("Planning trajectory")
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
                        logger.info("Executing plan")
                        robot_trajectory = plan_result.trajectory
                        robot.execute(robot_trajectory, controllers=[])
                else:
                        logger.error("Planning failed")

단일 파이프라인 Planning - 기본 설정
----------------------------------------------------
우리는 미리 정의된 로봇 구성 (srdf 파일에서 정의)으로 단일 계획 파이프라인을 실행하여 ``moveit_py`` 모션 계획 API를 탐색하기 시작합니다.: ::

        # set plan start state using predefined state
        panda_arm.set_start_state(configuration_name="ready")

        # set pose goal using predefined state
        panda_arm.set_goal_state(configuration_name="extended")

        # plan to goal
        plan_and_execute(panda, panda_arm, logger)

단일 파이프라인 Planning - Robot State
----------------------------------------------------
다음으로 로봇 상태를 목표로 계획합니다.
이러한 방법은 우리가 원하는대로 로봇 상태 설정을 변경할 수 있기 때문에 매우 유연합니다 (예: 조인트 값 설정을 통해).
여기서는 ``set_start_state_to_current_state`` 메서드를 사용하여 로봇의 시작 상태를 현재 상태로 설정하고 ``set_goal_state`` 메서드를 사용하여 목표 상태를 임의의 설정값으로 설정합니다.
그런 다음 목표 상태로 계획하고 계획을 실행합니다.: ::

        # instantiate a RobotState instance using the current robot model
        robot_model = panda.get_robot_model()
        robot_state = RobotState(robot_model)

        # randomize the robot state
        robot_state.set_to_random_positions()

        # set plan start state to current state
        panda_arm.set_start_state_to_current_state()

        # set goal state to the initialized robot state
        logger.info("Set goal state to the initialized robot state")
        panda_arm.set_goal_state(robot_state=robot_state)

        # plan to goal
        plan_and_execute(panda, panda_arm, logger)

단일 파이프라인 Planning - Pose Goal
----------------------------------------------------
목표 상태를 지정하는 또 다른 일반적인 방법은 포즈 목표를 표현하는 ROS 메시지를 사용하는 것입니다.
여기에서는 로봇의 엔드 이펙터에 대한 포즈 목표를 설정하는 방법을 보여줍니다.: ::

        # set plan start state to current state
        panda_arm.set_start_state_to_current_state()

        # set pose goal with PoseStamped message
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "panda_link0"
        pose_goal.pose.orientation.w = 1.0
        pose_goal.pose.position.x = 0.28
        pose_goal.pose.position.y = -0.2
        pose_goal.pose.position.z = 0.5
        panda_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="panda_link8")

        # plan to goal
        plan_and_execute(panda, panda_arm, logger)

단일 파이프라인 Planning - 커스텀 Constraints
----------------------------------------------------
커스텀 제약 조건을 통해 모션 계획의 출력을 제어할 수도 있습니다.
여기에서는 일련의 조인트 제약 조건을 만족하는 설정으로 계획하는 것을 보여줍니다.: ::

        # set plan start state to current state
        panda_arm.set_start_state_to_current_state()

        # set constraints message
        joint_values = {
                "panda_joint1": -1.0,
                "panda_joint2": 0.7,
                "panda_joint3": 0.7,
                "panda_joint4": -1.5,
                "panda_joint5": -0.7,
                "panda_joint6": 2.0,
                "panda_joint7": 0.0,
        }
        robot_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
                robot_state=robot_state,
                joint_model_group=panda.get_robot_model().get_joint_model_group("panda_arm"),
        )
        panda_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        # plan to goal
        plan_and_execute(panda, panda_arm, logger)

다중 파이프라인 Planning
----------------------------------------------------
``moveit_cpp`` 와 ``moveit_py`` 의 최근 추가 기능은 여러 계획 파이프라인을 병렬로 실행하고 생성된 모든 모션 계획 중 작업 요구 사항을 가장 잘 충족하는 모션 계획을 선택하는 기능입니다.
이전 섹션에서 우리는 일련의 계획 파이프라인을 정의했습니다.
여기서는 이러한 파이프라인 중 몇 개와 함께 병렬로 계획하는 방법을 살펴볼 것입니다.: ::

        # set plan start state to current state
        panda_arm.set_start_state_to_current_state()

        # set pose goal with PoseStamped message
        panda_arm.set_goal_state(configuration_name="ready")

        # initialise multi-pipeline plan request parameters
        multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
                panda, ["ompl_rrtc", "pilz_lin", "chomp", "ompl_rrt_star"]
        )

        # plan to goal
        plan_and_execute(
                panda,
                panda_arm,
                logger,
                multi_plan_parameters=multi_pipeline_plan_request_params,
        )

        # execute the plan
        if plan_result:
                logger.info("Executing plan")
                panda_arm.execute()

Planning Scene 사용하기
----------------------------------------------------
이 섹션의 코드는 다음과 같이 다른 파이썬 파일을 실행하도록 요구합니다. 사용자는 다음과 같이 파일을 지정할 수 있습니다.: ::

        ros2 launch moveit2_tutorials motion_planning_python_api_tutorial.launch.py example_file:=motion_planning_python_api_planning_scene.py

planning scene과 상호 작용하려면 planning scene monitor를 만들어야 합니다.: ::

        panda = MoveItPy(node_name="moveit_py_planning_scene")
        panda_arm = panda.get_planning_component("panda_arm")
        planning_scene_monitor = panda.get_planning_scene_monitor()

그런 다음 planning scene monitor의 ``read_write`` 컨텍스트를 사용하여 planning scene에 충돌 물체를 추가할 수 있습니다.: ::

        with planning_scene_monitor.read_write() as scene:
                collision_object = CollisionObject()
                collision_object.header.frame_id = "panda_link0"
                collision_object.id = "boxes"

                box_pose = Pose()
                box_pose.position.x = 0.15
                box_pose.position.y = 0.1
                box_pose.position.z = 0.6

                box = SolidPrimitive()
                box.type = SolidPrimitive.BOX
                box.dimensions = dimensions

                collision_object.primitives.append(box)
                collision_object.primitive_poses.append(box_pose)
                collision_object.operation = CollisionObject.ADD

                scene.apply_collision_object(collision_object)
                scene.current_state.update()  # Important to ensure the scene is updated

객체 제거는 ``CollisionObject.REMOVE`` 연산을 사용하거나, scene에서 모든 객체를 제거하는 방법으로도 수행할 수 있습니다.: ::

        with planning_scene_monitor.read_write() as scene:
                scene.remove_all_collision_objects()
                scene.current_state.update()

scene을 수정하지 않아도 되는 태스크(예: 충돌 검사)에 대해서 planning scene monitor의 ``read_only`` 컨텍스트를 사용할 수 있다.
예제: ::

        with planning_scene_monitor.read_only() as scene:
                robot_state = scene.current_state
                original_joint_positions = robot_state.get_joint_group_positions("panda_arm")

                # Set the pose goal
                pose_goal = Pose()
                pose_goal.position.x = 0.25
                pose_goal.position.y = 0.25
                pose_goal.position.z = 0.5
                pose_goal.orientation.w = 1.0

                # Set the robot state and check collisions
                robot_state.set_from_ik("panda_arm", pose_goal, "panda_hand")
                robot_state.update()  # required to update transforms
                robot_collision_status = scene.is_state_colliding(
                        robot_state=robot_state, joint_model_group_name="panda_arm", verbose=True
                )
                logger.info(f"\nRobot is in collision: {robot_collision_status}\n")

                # Restore the original state
                robot_state.set_joint_group_positions(
                        "panda_arm",
                        original_joint_positions,
                )
                robot_state.update()  # required to update transforms
