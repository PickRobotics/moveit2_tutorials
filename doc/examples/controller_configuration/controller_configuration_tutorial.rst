Low Level Controllers
=====================
MoveIt은 일반적으로 manipulator의 동작 명령을 `JointTrajectoryController <https://github.com/ros-controls/ros2_controllers/tree/master/joint_trajectory_controller>`_ 에게 publish합니다. 이 튜토리얼은 MoveGroup을 사용하여 로봇을 제어하는 것을 가정하며 MoveItCpp 또는 MoveIt Servo는 사용하지 않습니다. 최소 설정은 다음과 같습니다.:

#. YAML 설정 파일. 모범 사례로, 이 파일의 이름을 :code:`moveit_controllers.yaml` 로 지정하는 것을 권장합니다. 이 파일은 MoveIt에게 사용 가능한 컨트롤러, 각 컨트롤러와 연결된 조인트, 그리고 MoveIt 컨트롤러 인터페이스 타입(:code:`FollowJointTrajectory` 또는 :code:`GripperCommand`)을 알려줍니다. (`예제 컨트롤러 설정 파일 <https://github.com/ros-planning/moveit_resources/blob/ros2/panda_moveit_config/config/moveit_controllers.yaml>`_).

#. launch 파일. 이 launch 파일은 :code:`moveit_controllers.yaml` 파일을 로딩하고 :code:`moveit_simple_controller_manager/MoveItSimpleControllerManager` 를 지정해야 합니다. 이러한 yaml 파일이 로딩된 후에는 Move Group node에 파라미터로 전달됩니다. (`예제 Move Group launch 파일 <https://github.com/ros-planning/moveit_resources/blob/ros2/panda_moveit_config/launch/demo.launch.py>`_).

#. 해당 :code:`ros2_control` JointTrajectoryControllers를 launch합니다. 이는 MoveIt 2 에코시스템과 별개입니다. (`예제 ros2_control launching <https://github.com/ros-controls/ros2_control_demos>`_) 각 JointTrajectoryController는 action 인터페이스를 제공합니다. 위의 yaml 파일을 기반으로 MoveIt은 자동으로 이 action 인터페이스에 연결됩니다.

#. 참고: 여러분의 로봇에 :code:`ros2_control` 을 사용할 필요는 없습니다. 여러분이 독점적인 action 인터페이스를 작성할 수도 있습니다. 실제로는 99%의 사용자가 ros2_control을 선택합니다.

MoveIt Controller Managers
--------------------------
controller manager의의 base 클래스는 MoveItControllerManager (MICM)라고 불립니다. MICM의 자식 클래스 중 하나는 Ros2ControlManager (R2CM)이며 ros2_control과의 인터페이스에 가장 적합합니다. R2CM은 MoveIt으로 들어오는 궤적 명령내에 있는 조인트 이름을 파싱하고, 적합한 컨트롤러를 활성화할 수 있습니다. 예를 들어, 단일 조인트 그룹내에 있는 두 개의 manipulators 제어를 바로 단일 manipulator로 전환할 수 있습니다. R2CM을 사용하려면 launch 파일에서 :code:`moveit_manage_controllers = true` 를 설정하면 됩니다. `예제 R2CM launch 파일 <https://github.com/ros-planning/moveit_resources/blob/ros2/panda_moveit_config/launch/demo.launch.py>`_.

MoveIt Controller Interfaces
----------------------------

위 내용은 조인트 궤적 컨트롤러 액션(joint trajectory controller action) 인터페이스의 실행에 대해  설명합니다. 또한 MoveIt은 action 인터페이스를 통한 parallel-jaw gripper control를 지원합니다. 이 섹션에서는 이 두 옵션의 파라미터에 대해 설명합니다.

#. FollowJointTrajectory Controller Interface

The parameters are:
 * *name*: 컨트롤러의 이름  (중요 참고사항은 아래 디버깅 정보 참조)
 * *action_ns*: 컨트롤러의 action 네임스페이스 (중요 참고사항은 아래 디버깅 정보 참조)
 * *type*: 사용하는 action의 타입 (여기서는 FollowJointTrajectory).
 * *default*: 기본 컨트롤러는 MoveIt에서 특정 조인트 세트와 통신하기 위해 선택한 기초 컨트롤러
 * *joints*: 이 인터페이스에서 접근할 수 있는 모든 조인트의 이름

#. GripperCommand Controller Interface

The parameters are:
 * *name*: 컨트롤러의 이름  (중요 참고사항은 아래 디버깅 정보 참조)
 * *action_ns*: 컨트롤러의 action 네임스페이스 (중요 참고사항은 아래 디버깅 정보 참조)
 * *type*: 사용하는 action의 타입 (여기서는 GripperCommand).
 * *default*: 기본 컨트롤러는 MoveIt에서 특정 조인트 세트와 통신하기 위해 선택한 기초 컨트롤러
 * *joints*: 이 인터페이스에서 접근할 수 있는 모든 조인트의 이름
 * *command_joint*: 실제 그리퍼 상태를 제어하는 단일 조인트입니다. 이것은 컨트롤러에게 전송되는 유일한 값입니다. 위의 조인트 중 하나여야 합니다. 지정하지 않으면 대신 *joints* 의 첫 번째 항목이 사용됩니다.
 * *parallel*: 이 옵션을 설정하면 *joints* 의 크기는 2여야 하며, 명령은 해당 2개 조인트의 합이 됩니다.

Optional Allowed Trajectory Execution Duration Parameters
---------------------------------------------------------

(TODO: update for ROS2)

For each controller it is optional to set the *allowed_execution_duration_scaling* and *allowed_goal_duration_margin* parameters. These are controller-specific overrides of the global values *trajectory_execution/allowed_execution_duration_scaling* and *trajectory_execution/allowed_goal_duration_margin*. As opposed to the global values, the contoller-specific ones cannot be dynamically reconfigured at runtime. The parameters are used to compute the allowed trajectory execution duration by scaling the expected execution duration and adding the margin afterwards. If this duration is exceeded the trajectory will be cancelled. The controller-specific parameters can be set as follows ::

 controller_list:
  - name: arm_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    allowed_execution_duration_scaling: 1.2
    allowed_goal_duration_margin: 0.5

Debugging Information
---------------------

(TODO: update for ROS2)

The ``FollowJointTrajectory`` or ``GripperCommand`` interfaces on your robot must be communicating in the namespace: ``/name/action_ns``. In the above example, you should be able to see the following topics (using *ros2 topic list*) on your robot:

 * /panda_arm_controller/follow_joint_trajectory/goal
 * /panda_arm_controller/follow_joint_trajectory/feedback
 * /panda_arm_controller/follow_joint_trajectory/result
 * /hand_controller/gripper_action/goal
 * /hand_controller/gripper_action/feedback
 * /hand_controller/gripper_action/result

You should also be able to see (using ``ros2 topic info topic_name``) that the topics are published/subscribed to by the controllers on your robot and also by the **move_group** node.

Remapping /joint_states topic
-----------------------------

(TODO: update for ROS2)

When you run a :doc:`move group node </doc/examples/move_group_interface/move_group_interface_tutorial>`, you may need to remap the topic /joint_states to /robot/joint_states, otherwise MoveIt won't have feedback from the joints. To do this remapping you could make a simple launch file for your node as follows: ::

  <node pkg="moveit_ros_move_group" type="move_group" name="any_name" output="screen">
    <remap from="joint_states" to="robot/joint_states"/>
  </node>

Or you can make a subscriber with the correct topic name and then ensure that the starting robot state for your move group corresponds to a correct joints angle by using the call back of this subscriber.

Trajectory Execution Manager Options
------------------------------------

There are several options for tuning the behavior and safety checks of the execution pipeline in MoveIt. In your ``moveit_config`` package edit the ``trajectory_execution.launch.xml`` file to change the following parameters:

 - ``execution_duration_monitoring``: when false, will not throw error is trajectory takes longer than expected to complete at the low-level controller side
 - ``allowed_goal_duration_margin``: Allow more than the expected execution time before triggering a trajectory cancel (applied after scaling)
 - ``allowed_start_tolerance``: Allowed joint-value tolerance for validation that trajectory's first point matches current robot state. If set to zero will skip waiting for robot to stop after execution

Example Controller Manager
--------------------------

MoveIt controller managers, somewhat a misnomer, are the interfaces to your custom low level controllers. A better way to think of them are *controller interfaces*. For most use cases, the included :moveit_codedir:`MoveItSimpleControllerManager <moveit_plugins/moveit_simple_controller_manager>` is sufficient if your robot controllers already provide ROS actions for FollowJointTrajectory. If you use *ros_control*, the included :moveit_codedir:`MoveItRosControlInterface <moveit_plugins/moveit_ros_control_interface>` is also ideal.

However, for some applications you might desire a more custom controller manager. An example template for starting your custom controller manager is provided :codedir:`here <examples/controller_configuration/src/moveit_controller_manager_example.cpp>`.

Simulation
----------

If you do not have a physical robot, :code:`ros2_control` makes it very easy to simulate one. Ignition or Gazebo is not required; RViz is sufficient. All examples in the `ros2_control_demos repo <https://github.com/ros-controls/ros2_control_demos>`_ are simulated.

Controller Switching and Namespaces
-----------------------------------

(TODO: update for ROS2)

All controller names get prefixed by the namespace of their ros_control node. For this reason controller names should not contain slashes, and can't be named ``/``. For a particular node MoveIt can decide which controllers to have started or stopped. Since only controller names with registered allocator plugins are handled over MoveIt, MoveIt takes care of stopping controllers based on their claimed resources if a to-be-started controller needs any of those resources.

Controllers for Multiple Nodes
------------------------------

There is a variation on the Ros2ControlManager, the Ros2ControlMultiManager. Ros2ControlMultiManager can be used for more than one ros_control nodes. It works by creating several Ros2ControlManagers, one for each node. It instantiates them with their respective namespace and takes care of proper delegation. To use it must be added to the launch file. ::

  <param name="moveit_controller_manager" value="moveit_ros_control_interface::Ros2ControlMultiManager" />
