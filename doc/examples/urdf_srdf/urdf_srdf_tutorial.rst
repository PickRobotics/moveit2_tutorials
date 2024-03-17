.. _URDF and SRDF:

URDF 와 SRDF
======================

URDF
----
MoveIt 2는 ROS와 ROS2에서 로봇을 설명하는 기본 형식인 URDF(Unified Robot Description Format) 파일을 사용합니다. 이 튜토리얼에서는 URDF에 대한 리소스, 중요한 팁 그리고 MoveIt 2 특화된 요구사항 목록을 찾을 수 있습니다.

URDF 리소스
^^^^^^^^^^^^^^

* `URDF ROS Wiki Page <http://www.ros.org/wiki/urdf>`_ - URDF에 대한 대부분의 정보를 제공합니다.
* :ros_documentation:`URDF Tutorials <Tutorials/URDF/URDF-Main.html>` -  URDF 사용법에 대한 튜토리얼입니다.
* `SOLIDWORKS URDF Plugin <http://www.ros.org/wiki/sw_urdf_exporter>`_ - SOLIDWORKS 모델에서 직접 URDF 파일을 생성할 수 있는 플러그인입니다.

**주의** : 위 문서들은 ROS용으로 작성되었지만, 명령어만 ROS에서 ROS2 명령어 (예: rosrun -> ros2 run, roslaunch -> ros2 launch) 형식으로 바꾸면 모든 문서 내용은 유효합니다.

중요한 팁
^^^^^^^^^^^^^^
이 섹션에서는 생성한 URDF 파일을 MoveIt 2와 함께 사용할 수 있도록 하는 몇 가지 팁을 제공합니다. 로봇과 함께 MoveIt 2를 사용하기 전에 반드시 이 모든 팁을 살펴보세요.

조인트 이름에서 특수 문자
"""""""""""""""""""""""""""""""""
조인트 이름에는 다음과 같은 특수 문자가 포함되어서는 안됩니다: -,[,],(,),

이이러한 이름 제한이 곧 없어지기를 바랍니다.

안전 제한(Safety Limits)
""""""""""""""""""""""""""""
일부 URDF 파일에는 로봇의 조인트 제한 외에도 안전 제한이 설정되어 있습니다. 다음은 Panda 로봇의 head pan 조인트에 대해 지정한 안전 컨트롤러 예시입니다.: ::

   <safety_controller k_position="100" k_velocity="1.5" soft_lower_limit="-2.857" soft_upper_limit="2.857"/>

"soft_lower_limit" 필드와 "soft_upper_limit" 필드는 이 조인트에 대해서 조인트 위치 제한을 지정합니다. MoveIt 2는 이 제한을 URDF에 지정된 조인트의 하드 제한(hard limit)과 비교하여 보수적인 제한을 선택합니다.

.. note:: safety_controller 내의 "soft_lower_limit"과 "soft_upper_limit"가 0.0으로 설정되면 조인트를 움직일 수 없습니다. MoveIt 2는 올바른 로봇 모델을 지정하는 것을 사용자가 책임지도록 합니다.

충돌 검사
""""""""""""""""""
MoveIt 2는 충돌 검사를 위해 URDF에 지정된 메쉬를 사용합니다. URDF는 시각화와 충돌 검사를 위해 별도로 두 세트의 메쉬를 지정할 수 있게 해줍니다. 일반적으로 시각화 메쉬는 세밀하고 보기 좋을 수 있지만, 충돌 메쉬는 훨씬 덜 세밀해야 합니다. 메쉬의 삼각형 수는 로봇 링크의 충돌 검사 시간에 영향을 미치며 전체 로봇의 삼각형 수는 수천 개 이내여야 합니다.

URDF 테스트
""""""""""""""
URDF를 테스트하여 모든 것이 문제 없는지 확인하는 것은 매우 중요합니다. ROS URDF 패키지는 check_urdf 도구를 제공합니다. check_urdf 도구를 사용하여 URDF를 확인하려면 다음 지침을 따르십시오. `여기 <http://wiki.ros.org/urdf#Verification>`_ 

URDF 예제
^^^^^^^^^^^^^
ROS를 사용하는 로봇에 사용할 수 있는 많은 URDF가 있습니다.

* `URDF 예제 <http://www.ros.org/wiki/urdf/Examples>`_ - ROS 커뮤니티의 URDF 목록입니다.


SRDF
----

SRDF 또는 의미적 로봇 서술 형식(Semantic Robot Description Format)은 URDF를 보완하며, 조인트 그룹, 기본 로봇 구성, 추가 충돌 검사 정보 및 로봇의 포즈를 완전히 지정하는 데 필요한 추가 변환을 지정합니다. SRDF를 생성하는데 권장되는 방법은 MoveIt Setup Assistant을 사용하는 것입니다.

가상 조인트
^^^^^^^^^^^^^^
URDF는 로봇의 물리적 조인트에 대한 정보만 포함하고 있습니다. 종종 로봇의 root 링크의 포즈를 world 좌표계 기준으로 지정하기 위해 추가적인 조인트를 정의해야 합니다. 이러한 경우 가상 조인트를 사용하여 이 연결을 지정합니다. 예를 들어, 평면에서 이동하는 PR2와 같은 이동 로봇은 world 좌표 프레임을 로봇 프레임에 붙이는 평면 가상 조인트를 사용하여 지정됩니다. 고정 로봇(산업용 manipulator 등)은 고정 조인트를 사용하여 world에 붙여져야만 합니다.

수동 조인트(Passive Joints)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
수동 조인트는 로봇의 비구동 조인트(unactuated joint)입니다.(예를 들어, 차동 구동 로봇(differential drive robots)의 수동 캐스터(casters)) 이들은 모션 계획 또는 제어 파이프라인의 다른 컴포넌트가 해당 조인트를 직접 제어할 수 없다는 것을 알 수 있도록 SRDF에서 별도로 지정됩니다. 로봇에 비구동 캐스터가 있는 경우, 수동 캐스터(casters)로 지정해야 합니다.

Groups
^^^^^^
'그룹' (때때로 'JointGroup' 또는 'Planning Group' 이라고도 함)은 MoveIt 2의 핵심 개념입니다. MoveIt 2는 항상 특정 그룹에서 작동합니다. MoveIt 2는 계획 중인 그룹에 있는 조인트만 이동을 고려하며, 다른 조인트는 정지 상태로 유지됩니다. (로봇의 모든 조인트를 움직이게 할 수 있는 모션 계획은 모든 조인트를 포함하는 그룹을 만드는 것으로 달성할 수 있습니다.) 그룹은 단순히 조인트와 링크의 집합입니다. 각 그룹은 여러 방법 중 하나로 지정할 수 있습니다.:

조인트 콜렉션(Collection)
""""""""""""""""""""
그룹은 조인트 컬렉션으로 지정할 수 있습니다. 각 조인트의 모든 자식 링크는 자동으로 그룹에 포함됩니다.

링크 콜렉션(Collection)
"""""""""""""""""""
그룹은 또한 링크 컬렉션으로 지정할 수도 있습니다. 링크의 모든 부모 조인트도 그룹에 포함됩니다.

Serial Chain
""""""""""""
시리얼 체인은 base link와 tip link(끝 링크)를 사용하여 지정됩니다. 체인의 tip link는 체인의 마지막 조인트의 자식 링크입니다. 체인의 base link는 체인의 첫 번째 조인트의 부모 링크입니다.

Sub-Groups 콜렉션(Collection)
""""""""""""""""""""""""""""""""""
그룹은 또한 그룹 컬렉션일 수도 있습니다. 예를 들어, left_arm과 right_arm을 두 개의 그룹으로 정의한 다음 이러한 두 그룹을 포함하는 both_arms라는 새로운 그룹을 정의할 수 있습니다.

End-Effectors
^^^^^^^^^^^^^
로봇의 특정 그룹은 엔드 이펙터(End-Effectors)로 특별히 지정될 수 있습니다. 엔드 이펙터는 일반적으로 고정 조인트를 통해 다른 그룹(팔과 같은)에 연결됩니다. 엔드 이펙터 그룹을 지정할 때는 엔드 이펙터와 연결된 상위 그룹 간에 공통 링크가 없는지 확인하는 것이 중요합니다.

Self-Collisions
^^^^^^^^^^^^^^^
기본 자기 충돌 매트릭스 생성기(Self-Collision Matrix Generator, Setup Assistant의 일부)는 로봇에서 충돌 검사를 안전하게 비활성화할 수 있는 링크 쌍을 검색하여 모션 계획 처리 시간을 단축합니다. 이러한 링크 쌍은 항상 충돌 상태이거나, 충돌하지 않거나, 로봇의 기본 위치에 있거나, 링크가 운동학 체인에서 서로 인접해 있을 때 비활성화됩니다. 샘플링 밀도(sampling density)는 자기 충돌을 확인하기 위해 검사해야 하는 임의 로봇 위치의 수를 지정합니다. 밀도가 높을수록 더 많은 계산 시간이 필요하며 밀도가 낮을수록 비활성화되어서는 안 되는 쌍을 비활성화할 가능성이 높아집니다. 기본값은 10,000개의 충돌 검사입니다. 충돌 검사는 처리 시간을 줄이기 위해 병렬로 수행됩니다.

Robot Poses
^^^^^^^^^^^
SRDF에는 로봇의 고정 설정도 저장할 수 있습니다. 이 경우 SRDF의 일반적인 예는 manipulator의 HOME 위치를 정의하는 것입니다. 설정은 문자열 id와 함께 저장되며 나중에 설정을 복구하는 데 사용할 수 있습니다..

SRDF 문서
^^^^^^^^^^^^^^^^^^
SRDF 문법에 대한 정보는 `ROS SRDF Wiki page <http://www.ros.org/wiki/srdf>`_ 를 참고합니다.

URDF와 SRDF 로딩하기
-------------------------
ROS (Robot Operating System)의 MoveIt 라이브러리에서 :cpp_api:`RobotModel <moveit::core::RobotModel>` 클래스를 사용하는 경우 URDF와 SRDF에 접근해야 정상적으로 작동합니다. ROS 1에서는 각 파일의 XML 내용을 string parameter (각각 ``/robot_description`` 와 ``/robot_description_semantic``)로 로딩하여 전역 파라미터 서버에 저장하는 방식으로 이를 처리했습니다. 하지만 ROS 2는 전역 파라미터 서버가 없기 때문에, 모든 관련 node가 파일에 접근할 수 있도록 설정하는데 약간 더 복잡한 작업이 필요합니다.

Launch 파일 스펙
^^^^^^^^^^^^^^^^^^^^^^^^^
첫 번째 옵션은 필요한 각 node마다 개별적으로 파라미터를 설정하는 방법이며, 일반적으로는 launch 파일을 사용하여 이를 수행합니다.

URDF 로딩은 보통 xacro를 사용하며, launch 파일에서 로딩하는 코드는 다음과 같습니다.


.. code-block:: python

    from launch_ros.parameter_descriptions import ParameterValue
    from launch.substitutions import Command

    robot_description = ParameterValue(Command(['xacro ', PATH_TO_URDF]),
                                       value_type=str)

하지만 SRDF가 명시적으로 읽혀야만 합니다.

.. code-block:: python

    with open(PATH_TO_SRDF, 'r') as f:
        semantic_content = f.read()

다음으로 해당 값들은 각 node로 로딩되어야만 합니다.

.. code-block:: python

    move_group_node = Node(package='moveit_ros_move_group', executable='move_group',
                           output='screen',
                           parameters=[{
                                'robot_description': robot_description,
                                'robot_description_semantic': semantic_content,
                                # More params
                           }],
                           )

String Topic 스펙
^^^^^^^^^^^^^^^^^^^^^^^^^^
두 번째 옵션은 topic으로 2개 문자열을 publish하는 방법입니다. 이 패턴은 이미 `Robot State Publisher <https://github.com/ros/robot_state_publisher/blob/37aff2034b58794b78f1682c8fab4d609f5d2e29/src/robot_state_publisher.cpp#L136>`_ 에서 사용하였으며, ``/robot_description`` topic 으로 ``std_msgs/msg/String`` message를 publish 합니다. launch 파일에서 이를 수행하는 방법은 다음과 같습니다.:

.. code-block:: python

    rsp_node = Node(package='robot_state_publisher',
                    executable='robot_state_publisher',
                    respawn=True,
                    output='screen',
                    parameters=[{
                        'robot_description': robot_description,
                        'publish_frequency': 15.0
                    }]
                    )

또한 MoveIt node에서도 이 topic을 publish하도록 설정할 수 있습니다.

.. code-block:: python

    move_group_node = Node(package='moveit_ros_move_group', executable='move_group',
                           output='screen',
                           parameters=[{
                                'robot_description': robot_description,
                                'publish_robot_description': True,
                                # More params
                           }],
                           )

이 방법을 사용하면 robot description을 topic으로 한 번만 publish하면 되므로 모든 node가 개별적으로 description을 필요로하지 않습니다.

SRDF 또한 마찬가지로 ``std_msgs/msg/String`` message로 publish할 수 있습니다. 이 경우 launch 파일에서 한 node에 대한 파라미터를 설정하고, 추가로 ``publish_robot_description_semantic`` 파라미터를 True로 설정해야 합니다.

.. code-block:: python

    move_group_node = Node(package='moveit_ros_move_group', executable='move_group',
                           output='screen',
                           parameters=[{
                                'robot_description_semantic': semantic_content,
                                'publish_robot_description_semantic': True,
                                # More params
                           }],
                           )

그러면 다른 모든 node들은 publish된 문자열 메시지를 subscribe할 수 있습니다.

Under the Hood: RDFLoader
^^^^^^^^^^^^^^^^^^^^^^^^^
MoveIt 코드내 여러 곳에서, robot description과 semantics는  :moveit_codedir:`RDFLoader<moveit_ros/planning/rdf_loader/include/moveit/rdf_loader/rdf_loader.h>` 클래스를 사용하여 로딩됩니다.
이 클래스는 node로부터 파라미터를 읽어 들이려고 시도하며, 읽기에 실패할 경우 짧은 시간 동안 문자열 topic을 subscribe하려고 시도합니다. 두 가지 방법 모두 파라미터를 가져오는데 실패하면 경고 메시지가 콘솔에 출력됩니다.
