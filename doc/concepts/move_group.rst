=======================
The ``move_group`` node
=======================

아래 그림은 MoveIt에서 제공하는 핵심 node인 :cpp_api:`move_group <move_group::MoveGroupExe>`의 하이레벨 시스템 아키텍처를 보여줍니다.
이 node는 통합자 역할을 하며, 개별 component들을 모두 모아 사용자가 사용할 수 있는 ROS action과 service 세트를 제공합니다.

.. image:: /_static/images/move_group.png

User Interface
--------------

사용자는 다음 두 가지 방법으로 ``move_group``에서 제공하는 action과 service에 액세스할 수 있습니다.:

- **C++ 에서** - :cpp_api:`move_group_interface <moveit::planning_interface::MoveGroupInterface>` 패키지를 사용하여 move_group에 대해서 쉽게 설정이 가능한 C++ 인터페이스를 제공

- **GUI를 통해서** - RViz (ROS 시각화 도구)의 :doc:`Motion Planning plugin to Rviz <../tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial>` (the ROS visualizer) 사용


Configuration
-------------

``move_group`` 은 ROS node입니다.
이는 ROS 매개변수 서버를 사용하여 3가지 종류의 정보를 얻습니다.:

1. URDF

   * ``move_group`` 은 로봇에 대한 URDF를 가져오기 위해서 ``robot_description`` 파라미터를 찾습니다.

2. SRDF

   * ``move_group`` 은 로봇에 대한 SRDF를 가져오기 위해서 ``robot_description_semantic`` 파라미터를 찾습니다. SRDF는 일반적으로 MoveIt Setup Assistant를 사용하여 사용자가 (한 번) 생성합니다.

3. MoveIt configuration

   * ``move_group`` 은 관절 제한(joint limits), 운동학(kinematics), 모션 계획, 인식 및 기타 정보를 포함하여 MoveIt에 특화된 설정을 찾습니다. 이러한 components에 대한 설정 파일은 MoveIt Setup Assistant에 의해 자동으로 생성되고 해당 로봇의 MoveIt config package의 ``config`` 디렉토리에 저장됩니다.

Robot Interface
---------------

``move_group``은 ROS topic과 action을 통해 로봇과 통신합니다.
로봇의 현재 상태 정보 (joint의 위치 등)를 얻고, 로봇 센서로부터 point cloud 및 기타 센서 데이터를 가져오고, 로봇의 컨트롤러와 통신 등을 위해 로봇과 통신합니다.

Joint State Information
-----------------------

``move_group`` 은 ``/joint_states`` topic을 listen하여 로봇의 각 joint 위치 등과 같이 현재 상태 정보를 결정합니다.
또한 이 토픽에 여러 publishers가 부분적인 로봇 상태 정보만 publish한다고 하더라도 ``move_group`` 은 이들을 모두 수신할 수 있습니다. (예: 로봇 팔과 mobile base에 대해 별도의 publisher를 사용할 수 있음)
주의: ``move_group`` 은 자체 joint state publisher를 구성하지 않습니다. - 이는 각 로봇에서 구현해야 하는 것입니다.

Transform Information
---------------------

``move_group`` 은 ROS TF 라이브러리를 사용하여 transformation 정보를 모니터링합니다. 
이를 통해 node는 로봇의 pose에 대한 전역 정보 (다른 것들 중에서)를 얻을 수 있습니다.
예를 들어, ROS navigation stack은 로봇의 map frame과 base frame 간의 변환을 TF에 publish합니다.
``move_group`` 은 TF를 사용하여 내부적인 사용을 위한 이 transformation을 파악할 수 있습니다.
주의: ``move_group``은 TF만 수신합니다.
로봇에서 TF 정보를 publish하려면 로봇쪽에서 ``robot_state_publisher`` node를 실행해야 합니다.

Controller Interface
--------------------

``move_group`` 은 FollowJointTrajectoryAction 인터페이스를 사용하여 로봇의 컨트롤러와 통신합니다.
이는 ROS action 인터페이스입니다.
로봇쪽의 서버는 이 action을 서비스해야 합니다. - 이 서버는 ``move_group`` 자체에서 제공되지 않습니다.
``move_group`` 은 로봇쪽에 있는 이 controller action server와 통신하는 클라이언트만 인스턴스화합니다.

Planning Scene
--------------

``move_group`` 은 Planning Scene Monitor를 사용하여 **계획 장면(planning scene)**을 유지 관리합니다. 이 계획 장면은 world와 로봇의 현재 상태를 나타내는 것입니다.
로봇 상태는 로봇에 견고하게 부착된(함께 이동되는) 것으로 간주되는 모든 물체를 포함할 수 있습니다. **planning scene**을 유지 관리 및 업데이트하는 아키텍처에 대한 자세한 내용은 아래의 Planning Scene 섹션에 나와 있습니다.

Extensible Capabilities
-----------------------

``move_group`` 은 쉽게 확장할 수 있도록 설계되었습니다. pick and place, 운동학, 모션 계획과 같은 개별 기능은 실제로 공통 기본 클래스를 가진 별도의 플러그인으로 구현됩니다.
플러그인은 ROS yaml 파라미터와 ROS pluginlib 라이브러리를 사용하는 ROS를 사용하여 설정됩니다. 대부분의 사용자는 MoveIt Setup Assistant에서 생성된 시작 파일에서 자동으로 구성되므로 move_group 플러그인을 설정할 필요가 없습니다.
