Planning Around Objects
=======================

이 튜토리얼에서는 planning scene에 객체를 삽입하고 개체 주변을 planning하는 방법을 소개합니다.

Prerequisites
-------------

만약 아직 완료하지 않았다면, :doc:`Visualizing in RViz </doc/tutorials/visualizing_in_rviz/visualizing_in_rviz>`에 있는 단계들을 완료하세요.
이 프로젝트는 이전 튜토리얼의 마지막 지점인 ``hello_moveit`` 프로젝트를 기반으로 진행됩니다. 튜토리얼만 실행하고 싶다면, :doc:`Docker Guide </doc/how_to_guides/how_to_setup_docker_containers_in_ubuntu>`를 따라 완성된 튜토리얼이 있는 컨테이너를 실행할 수 있습니다.

Steps
-----

1 Add include for Planning Scene Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

소스 파일 맨 위에 있는 include 목록에 아래 내용을 추가하십시오:

.. code-block:: C++

  #include <moveit/planning_scene_interface/planning_scene_interface.h>

2 Change the Target Pose
^^^^^^^^^^^^^^^^^^^^^^^^

먼저 로봇이 다른 위치로 이동하도록 경로를 계획하도록 만들기 위해, 다음 변경 사항을 적용하여 목표 포즈(target pose)를 업데이트하세요.:

.. code-block:: C++

    // Set a target Pose
    auto const target_pose = [] {
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 1.0;
      msg.position.x = 0.28;
      msg.position.y = 0.4;  // <---- This value was changed
      msg.position.z = 0.5;
      return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);

3 Create a Collision Object
^^^^^^^^^^^^^^^^^^^^^^^^^^^

다음 코드 블록에서 충돌 객체를 만듭니다.
첫 번째 주목해야 할 점은 이 객체가 로봇 프레임에 위치하게 된다는 것입니다.
만약 우리가 scene에서 장애물의 위치를 보고해주는 인식 시스템을 가지고 있다면, 이 시스템이 만들어내는 메시지의 형태일 수도 있습니다.
하지만 이번에는 단지 예시이기 때문에 수동으로 만들고 있습니다.
이 코드 블록의 끝에서 주목해야 할 한 가지는 이 메시지의 작업을 ``ADD``로 설정한다는 것입니다.
이로 인해 객체가 충돌 scene에 추가됩니다.
이전 단계의 목표 포즈를 설정하는 것과 계획을 생성하는 것 사이에 아래 코드 블록을 삽입하세요.

.. code-block:: C++

    // Create collision object for the robot to avoid
    auto const collision_object = [frame_id =
                                     move_group_interface.getPlanningFrame()] {
      moveit_msgs::msg::CollisionObject collision_object;
      collision_object.header.frame_id = frame_id;
      collision_object.id = "box1";
      shape_msgs::msg::SolidPrimitive primitive;

      // Define the size of the box in meters
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = 0.5;
      primitive.dimensions[primitive.BOX_Y] = 0.1;
      primitive.dimensions[primitive.BOX_Z] = 0.5;

      // Define the pose of the box (relative to the frame_id)
      geometry_msgs::msg::Pose box_pose;
      box_pose.orientation.w = 1.0;
      box_pose.position.x = 0.2;
      box_pose.position.y = 0.2;
      box_pose.position.z = 0.25;

      collision_object.primitives.push_back(primitive);
      collision_object.primitive_poses.push_back(box_pose);
      collision_object.operation = collision_object.ADD;

      return collision_object;
    }();

4 Add the Object to the Planning Scene
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

마지막으로 이 객체를 충돌 scene에 추가해야 합니다.
이 작업을 위해 ROS 인터페이스를 사용하여 계획 장면(planning scene)의 변경 사항을 MoveGroup에게 전달하는 ``PlanningSceneInterface``라는 객체를 사용합니다.
이 코드 블록은 충돌 객체를 생성하는 코드 블록 바로 다음에 와야 합니다.

.. code-block:: C++

    // Add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);


5 Run the Program and Observe the Change
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

지난 튜토리얼과 마찬가지로 ``demo.launch.py`` 스크립트를 사용하여 RViz를 시작하고 프로그램을 실행하십시오. Docker 튜토리얼 컨테이너 중 하나를 사용하는 경우, 다음과 같이 이미 RvizVisualToolsGui 패널이 추가된 다른 RViz 구성을 지정할 수 있습니다.: ::

   ros2 launch moveit2_tutorials demo.launch.py rviz_config:=panda_hello_moveit.rviz

.. image:: planning_around_object.png

Summary
-------

- MoveIt을 사용하여 scene내에 있는 물체 주변을 계획하도록 작성한 프로그램을 확장했습니다.
- :codedir:`전체 소스 hello_moveit.cpp source<tutorials/planning_around_objects/hello_moveit.cpp>`.

Further Reading
---------------

- :doc:`충돌 및 제약 조건 검사를 위한 Planning Scene 사용 예시 </doc/examples/planning_scene/planning_scene_tutorial>`.
- :doc:`Planning Scene ROS API 사용 예시 </doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial>`.
- :doc:`충돌 개체 시각화 예시 </doc/examples/visualizing_collisions/visualizing_collisions_tutorial>`.
- :doc:`객체와 함께 계획하는 데 사용되는 subframes 예시 </doc/examples/subframes/subframes_tutorial>`.

Next Step
---------

다음 튜터리얼 :doc:`Pick and Place with MoveIt Task Constructor </doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor>`에서, 더 어려운 모션 계획을 해결하도록 설계된 상위 계층 도구를 소개합니다.
이 다음 튜토리얼에서는 물체를 집어서 놓는 프로그램을 만들 것입니다.
