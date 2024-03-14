Issac 로봇 시뮬레이션에 명령 전달 방법
=======================================

이 튜터리얼을 진행하기 위해서는 컴퓨터에  ``Isaac Sim 2023.1.x`` (권장) 혹은 ``Isaac Sim 2022.2.x`` 가 설치되어 있어야 합니다.
아이작 시뮬레이터의 요구사항 및 설치 방법은 `Omniverse 문서 <https://docs.omniverse.nvidia.com/isaacsim/latest/index.html>`_ 에서 확인하실 수 있습니다.
Isaac 시뮬레이터를 ROS 2와 함께 사용하도록 설정하는 방법은 `이 가이드 <https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#running-native-ros>`_ 에서 확인하세요.

이 튜터리얼은 다음과 같은 시스템 설정을 전제합니다.:

1. NVIDIA Isaac 시뮬레이터가 기본 위치에 설치되어 있어야 합니다. Docker 기반 설치도 지원되지만 시스템 설정은 사용자에게 달려 있습니다.
2. Docker 설치되어 있어야 합니다.
   만약 MoveIt과 함께 GPU를 사용하려면 `nvidia-docker <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installing-on-ubuntu-and-debian>`_ 을 설치해야 합니다.
3. 이 저장소를 복제하여 Ubuntu 22.04 Humble 기반 Docker 이미지를 빌드하고 이를 통해 Isaac와 통신하여 이 튜터리얼을 실행할 수 있습니다.

ros2_control 소개
----------------------------

MoveIt이 계산한 궤적을 실행하는 권장 방법 중 하나는 `ros2_control <https://control.ros.org/master/index.html>`_
프레임워크를 사용하여 로봇(실제 또는 시뮬레이션)을 제어하고 통신하는 것입니다. 이 방법은 개발자에게 공통 API를 제공하여 몇 가지 launch 인자만 변경하면
다양한 로봇 유형과 내장 센서 간에 소프트웨어를 전환할 수 있기 때문에 매우 권장됩니다.
예를 들어 Panda 로봇의 ``ros2_control.xacro`` 파일을 살펴보면  ``use_fake_hardware`` 플래그를 사용하여 시뮬레이션과 실제 로봇 연결 간에 전환하는 것을 확인할 수 있습니다.

.. code-block:: XML

    <hardware>
      <xacro:if value="${use_fake_hardware}">
        <plugin>mock_components/GenericSystem</plugin>
      </xacro:if>
      <xacro:unless value="${use_fake_hardware}">
        <plugin>franka_hardware/FrankaHardwareInterface</plugin>
        <param name="robot_ip">${robot_ip}</param>
      </xacro:unless>
    </hardware>


`Hardware 컴포넌트들 <https://control.ros.org/master/doc/getting_started/getting_started.html#hardware-components>`_ 은 여러 가지 타입이 있지만, plugin ``<plugin>mock_components/GenericSystem</plugin>`` 은 매우 간단한 ``System`` 으로 들어오는 ``command_interface`` 값을 조인트의 추적되는 ``state_interface`` (즉, 시뮬레이션된 조인트의 완벽한 제어)로 전달합니다.

Panda 로봇을 Isaac 시뮬레이터로 확장하기 위해서는 먼저 `topic_based_ros2_control <https://github.com/PickNikRobotics/topic_based_ros2_control>`_ 를 도입해야 합니다.
이 하드웨어 인터페이스는 설정된 토픽에서 subscribe와 publish를 수행하는 ``System`` 입니다.
이 튜토리얼에서는 topic ``/isaac_joint_states`` 에 로봇의 현재 상태가 포함되며, ``/isaac_joint_commands`` 는 로봇을 구동하는 데 사용됩니다.
이 튜토리얼에서 사용하고 있는 `moveit_resources_panda_moveit_config <https://github.com/ros-planning/moveit_resources/blob/humble/panda_moveit_config/config/panda.ros2_control.xacro#L7>`_ 은 하드웨어 연결을 지원하지 않기 때문에 ``ros2_control.xacro`` 파일이 업데이트하여 플래그 ``ros2_control_hardware_type`` 이 ``isaac`` 값으로 설정되면 ``TopicBasedSystem`` 플러그인을 로딩하도록 합니다.

.. code-block:: XML

    <xacro:if value="${ros2_control_hardware_type == 'mock_components'}">
        <plugin>mock_components/GenericSystem</plugin>
    </xacro:if>
    <xacro:if value="${ros2_control_hardware_type == 'isaac'}">
        <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
        <param name="joint_commands_topic">/isaac_joint_commands</param>
        <param name="joint_states_topic">/isaac_joint_states</param>
    </xacro:if>

이 튜토리얼에서는 Panda 로봇을 로딩하고 ROS topic을 publish/subscribe하여 로봇을 제어하는 `OmniGraph <https://docs.omniverse.nvidia.com/isaacsim/latest/gui_tutorials/tutorial_gui_omnigraph.html>`_ 를 만드는 Python 스크립트를 포함하고 있습니다.
OmniGraph는 또한 Panda 손에 장착된 카메라로부터 RGB 이미지와 Depth 이미지를 publish하는 node도 포함합니다.
RGB 이미지는 topic ``/rgb`` 에, 카메라 정보는 ``/camera_info`` 에, Depth 이미지는 ``/depth`` 에 게시됩니다. 카메라 프레임의 프레임 ID는 ``/sim_camera`` 입니다.
Isaac 시뮬레이터 로봇을 ROS 2와 통신하도록 설정하는 방법에 대해서는 Omniverse의 `Joint Control tutorial <https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_manipulation.html>`_ 을 참조하십시오.

Computer 설정
--------------

1. `Isaac Sim <https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html>`_ 설치하기

2. MoveIt 2 튜터리얼 저장소를 복제하기(shallow clone)

.. code-block:: bash

  git clone https://github.com/ros-planning/moveit2_tutorials.git -b main

1. 복제한 폴더로 이동한 후 다음 디렉토리로 변경합니다.

.. code-block:: bash

  cd moveit2_tutorials/doc/how_to_guides/isaac_panda

4. Docker 이미지를 빌드합니다. 이 이미지는 pytorch도 포함합니다.
``pytorch``.

.. code-block:: bash

  docker compose build base


Mock 컴포넌트로 MoveIt 상호작용 Marker 데모 실행하기
---------------------------------------------------------------

이 섹션은 Isaac 시뮬레이션 대신 ``mock_components/GenericSystem`` 하드웨어 인터페이스를 테스트합니다.

1. ``mock_components/GenericSystem`` 을 테스트하기 위해 하드웨어 인터페이스를 실행합니다.:

.. code-block:: bash

  docker compose up demo_mock_components

명령어를 실행하면 RViz가 열리고 Panda 로봇이 ``mock_components`` 를 사용하여 시뮬레이션되고 궤적이 실행됩니다.

처음으로 MoveIt을 RViz와 함께 사용하는 경우 :doc:`Quickstart in RViz </doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial>` 튜토리얼을 참고하세요.

테스트가 끝나면 터미널에서 ``Ctrl+C`` 를 눌러 컨테이너를 종료하십시오.

Isaac 시뮬레이션으로 MoveIt 상호작용 Marker 데모 실행하기
---------------------------------------------------------

1. 호스트 컴퓨터에서 튜토리얼 launch 디렉토리로 이동하십시오.

.. code-block:: bash

  cd moveit2_tutorials/doc/how_to_guides/isaac_panda/launch

1. 다음 명령을 실행하여 이 튜토리얼과 함께 작동하도록 미리 설정된 Panda 로봇을 로딩하십시오.

.. note:: 이 단계는 호스트 시스템에 호환되는 버전의 Isaac Sim이  ``$HOME/.local/share/ov/pkg/" directory`` 에 설치되어 있다고 가정합니다.
  또한 이 단계는 assets을 다운로드하고 Isaac Sim을 설정하는 데 몇 분이 소요되므로 참을성을 가지고 시뮬레이터가 시작되는 동안 나타나는 ``Force Quit`` (강제 종료) 대화 상자를 클릭하지 마십시오.

.. code-block:: bash

  ./python.sh isaac_moveit.py

1. ``moveit2_tutorials/doc/how_to_guides/isaac_panda`` 디렉토리로 이동하여 ``topic_based_ros2_control/TopicBasedSystem`` 하드웨어 인터페이스를 사용하여 Isaac Sim에 연결하는 컨테이너를 구동시킵니다.

.. code-block:: bash

  docker compose up demo_isaac

이는 RViz를 열어 시뮬레이션된 Panda 로봇과 통신하고  ``TopicBasedSystem`` 인터페이스를 사용하여 궤적을 실행합니다.

.. raw:: html

    <div style="position: relative; padding-bottom: 5%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="700px" height="400px" src="https://www.youtube.com/embed/EiLaJ7e4M-4" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
    </div>
