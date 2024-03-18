:moveit1:

..
   Once updated for MoveIt 2, remove all lines above title (including this comment and :moveit1: tag)

IKFast Kinematics Solver
========================

.. image:: openrave_panda.png
   :width: 700px

이 섹션에서는 MoveIt을 위한 IKFast 플러그인 설정 방법을 소개합니다.

IKFast란?
---------------

IKFast(Robot Kinematics Compiler)는 Rosen Diankov의 `OpenRAVE <http://openrave.org>`_ 모션 플래닝 소프트웨어에서 제공되는 강력한 역 운동학 솔버(inverse kinematics solver)입니다. IKFast는 일반적인 패턴을 가진 모든 복잡한 운동학 체인을 자동으로 분석하여 해석 솔루션을 생성하고 이를 찾는 C++ 코드를 생성합니다.
결과적으로 IKFast는 최근 프로세서에서 몇 마이크로초 내에 찾을 수 있는 매우 안정적인 솔루션을 제공합니다.

MoveIt IKFast
---------------

MoveIt는 OpenRAVE에서 생성된 cpp 파일을 사용하여 MoveIt용 IKFast 운동학 플러그인을 생성하는 도구를 제공합니다.
이 튜터리얼에서는 로봇을 설정하여 IKFast의 기능을 활용하도록 안내합니다.
MoveIt IKFast는 ROS Melodic에서 6자유도 및 7자유도 로봇 암 manipulator에서 테스트되었습니다.
이론적으로는 작동하지만 현재 MoveIt IKFast는 7자유도 이상의 팔을 지원하지 않습니다.

시작하기
-----------------
:doc:`Getting Started </doc/tutorials/getting_started/getting_started>` 을 먼저 완료하세요.

:doc:`Setup Assistant </doc/examples/setup_assistant/setup_assistant_tutorial>` 을 사용하여 생성된 로봇용 MoveIt 설정 패키지가 있어야 합니다.

OpenRAVE는 MoveIt 자체만큼이나 복잡한 플래닝 프레임워크이며 설치가 다소 어렵습니다. 특히 공개 문서가 더 이상 유지 관리되지 않기 때문입니다.
다행히 personalrobotics는 Ubuntu 14.04에 OpenRAVE 0.9.0과 ROS Indigo가 설치된 `docker image <https://hub.docker.com/r/personalrobotics/ros-openrave>`_ 를 제공하며, 이를 사용하여 solver 코드를 한 번 생성할 수 있습니다.

따라서 IKFast 코드 생성기를 실행하는 가장 쉬운 방법은 이 docker 이미지를 통하는 것입니다.
수동 빌드 지침(Ubuntu 16.04에 맞춤)은 `Kinetic version 튜터리얼 <http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html>`_ 을 참조하십시오.
추천하는 docker 기반 접근법을 따른다면, docker가 설치되고 시작되었는지 아래와 같이 확인하세요.: ::

 sudo apt-get install docker.io
 sudo service docker start

다음 명령은 사용자 계정으로 docker를 실행할 수 있도록 해줍니다 ($USER를 docker 그룹에 추가합니다): ::

 sudo usermod -a -G docker $USER

실제로 이 권한 변경을 활성화하려면 로그아웃/로그인해야 합니다.

Debian 패키지 혹은 소스 코드에서 MoveIt IKFast 패키지를 설치하세요.

**Binary 설치**: ::

 sudo apt-get install ros-${ROS_DISTRO}-moveit-kinematics

**소스**

catkin 워크스페이스의 ``./src`` 디렉토리 내에서: ::

 git clone https://github.com/ros-planning/moveit.git
 rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO}
 catkin build

IKFast MoveIt plugin 생성하기
---------------------------------

복사-붙여넣기를 용이하게 하기 위해, 로봇 이름을 환경 변수로 정의하는 것을 권장합니다.: ::

  export MYROBOT_NAME="panda_arm"

OpenRAVE는 로봇을 설명하기 위해 URDF 대신 Collada를 사용합니다. 로봇의 URDF를 자동으로 Collada로 변환하려면 .urdf 파일을 제공해야 합니다.
.urdf 파일이 `xacro <http://wiki.ros.org/xacro/>`_ 파일에서 생성된 경우, 다음 명령을 사용하여 URDF를 생성할 수 있습니다.: ::

  rosrun xacro xacro -o $MYROBOT_NAME.urdf $MYROBOT_NAME.urdf.xacro

IK Type 선택
^^^^^^^^^^^^^^
IK 타입 선택에 관련하여 더 자세한 정보는 `이 페이지 <http://openrave.org/docs/latest_stable/openravepy/ikfast/#ik-types>`_ 를 참조하십시오.
가장 일반적인 IK 타입은 *transform6d* 입니다.

Planning Group 선택
^^^^^^^^^^^^^^^^^^^^^
로봇 팔 또는 "planning group" 이 여러 개이고 각각에 대해 IKFast 솔버를 생성하고 싶다면 다음 과정을 각 그룹에 대해 반복해야 합니다.
다음 설명은 여러분이 ``<planning_group_name>`` 을 선택했다고 가정합니다. 또한 솔루션을 구할 체인의 base link와 엔드 이펙터 링크의 이름을 알아야 합니다.

IKFast MoveIt plugin 생성하기
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

IKFast MoveIt 플러그인을 생성하려면 다음 명령을 실행하십시오.: ::

  rosrun moveit_kinematics auto_create_ikfast_moveit_plugin.sh --iktype Transform6D $MYROBOT_NAME.urdf <planning_group_name> <base_link> <eef_link>

이 프로세스의 속도와 성공 여부는 로봇의 복잡성에 따라 달라집니다. 일반적으로 base와 wrist에 3개의 교차 축을 가진 6 DOF manipulator는 솔버 코드를 생성하는 데 불과 몇 분밖에 걸리지 않습니다. 생성 절차에 대한 자세한 설명과 프로세스의 추가 조정은 `Tweaking the creation process`_ 을 참조하십시오.

위 명령은 현재 폴더 내에 ``$MYROBOT_NAME_<planning_group_name>_ikfast_plugin`` 이라는 이름의 새로운 ROS 패키지를 생성합니다.
따라서 새 패키지가 찾을 수 있도록 작업 공간을 다시 빌드해야 합니다.: ::

  catkin build

Usage
-----
IKFast 플러그인은 기본 KDL IK 솔버를 대체하는 드롭인 방식으로 사용할 수 있지만 성능이 크게 향상됩니다. MoveIt 설정 파일은 생성 스크립트에 의해 자동으로 편집되지만 경우에 따라 실패할 수도 있습니다. 이 경우 로봇의 kinematics.yaml 파일에서 *kinematics_solver* 파라미터를 사용하여 KDL과 IKFast 솔버 간에 전환할 수 있습니다.: ::

  rosed "$MYROBOT_NAME"_moveit_config kinematics.yaml

Edit these parts: ::

 <planning_group>:
   kinematics_solver: <myrobot_name>_<planning_group>_ikfast_plugin/IKFastKinematicsPlugin

플러그인을 테스트
^^^^^^^^^^^^^^^^^^^^^^
MoveIt RViz Motion Planning 플러그인을 사용하여 대화형 마커를 통해 올바른 IK 솔루션을 찾았는지 확인하십시오.

Plugin을 업데이트
-------------------

IKFast 또는 MoveIt의 향후 변경 사항이 있을 경우, 제공된 스크립트를 사용하여 이 플러그인을 다시 생성해야 할 수도 있습니다. 이를 쉽게 하려면, IKFast MoveIt 패키지 루트에 자동으로 *update_ikfast_plugin.sh* 라는 bash 스크립트를 생성되도록 합니다. 이 스크립트는 OpenRAVE에서 생성된 .cpp 솔버 파일로부터의 플러그인을 다시 생성합니다.

Tweaking the creation process
-----------------------------

IKFast MoveIt 플러그인을 생성 프로세스는 다음과 같은 몇 가지 단계로 구성되어 있으며, 생성 스크립트에 의해 하나씩 수행됩니다.:

1. `personalrobotics <https://hub.docker.com/r/personalrobotics/ros-openrave>`_ 가 제공하는 docker 이미지 다운로드 받기
2. ROS URDF 파일을 OpenRAVE에 필요한 Collada 형식으로 변환: ::

     rosrun collada_urdf urdf_to_collada $MYROBOT_NAME.urdf $MYROBOT_NAME.dae

   때때로 URDF 파일을 Collada로 변환하는 과정에서 부동 소수점 문제가 발생하여 OpenRAVE가 IK 솔루션을 찾지 못할 수 있습니다.
   utility 스크립트를 사용하여 .dae 파일의 모든 숫자를 간편하게 n 자리 소수점 이하로 반올림할 수 있습니다.
   경험상 5자리 소수점을 권장하지만, OpenRave ikfast 생성기가 솔루션을 찾는 데 너무 오랜 시간이 걸린다면 (예: 1시간 이상) 정확도를 낮추는 것이 도움이 될 것입니다. 예를 들어: ::

     rosrun moveit_kinematics round_collada_numbers.py $MYROBOT_NAME.dae $MYROBOT_NAME.rounded.dae 5

3. OpenRAVE IKFast 도구를 실행하여 C++ solver 코드 생성
4. 생성된 solver를 래핑(wrapping)하는 MoveIt IKFast 플러그인 패키지 생성

``auto_create_ikfast_moveit_plugin.sh`` 스크립트는 입력 파일의 확장자를 평가하여 실행할 단계를 결정합니다. 중간 단계 (예: ``.dae`` 파일의 정확도를 조정한 후)에서 스크립트를 다시 실행하려면, 이전 단계의 해당 출력 (``.dae`` 또는 ``.cpp``)을 초기 ``.urdf`` 파일 대신에 입력으로 제공하기만 하면 됩니다.
