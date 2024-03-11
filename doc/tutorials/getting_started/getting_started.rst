시작하기
===============

여기서 튜토리얼을 가장 잘 실행하기 위한 환경을 설정할 것입니다. colcon 워크스페이스를 만들고, 모든 최신 MoveIt 소스 코드를 다운로드하고, 모든 것을 소스에서 빌드하여 최신 수정 사항과 개선 사항을 보장합니다.

MoveIt의 모든 소스 코드를 빌드하는 데는 20-30분이 걸릴 수 있습니다. 컴퓨터의 CPU 속도와 사용 가능한 RAM에 따라 다르지만, 성능이 떨어지는 시스템에서 시작하거나 일반적으로 빨리 시작하려는 경우 :doc:`Docker 가이드 </doc/how_to_guides/how_to_setup_docker_containers_in_ubuntu>` 를 확인하십시오.

ROS 2와 colcon 설치하기
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
MoveIt 2는 현재 여러 버전의 ROS를 지원합니다.
여러분이 원하는 버전을 설치하십시오.
우리는 주로 Ubuntu 22.04에 설치된 ROS를 지원하지만, 아래 지침에 약간의 변경 사항을 적용하여 다른 방법과 플랫폼에서도 작동할 수 있습니다.
여러분이 처음 시작하는 경우 가장 원활한 경험을 위해 Ubuntu 22.04에서 최신 안정 ROS 버전(Iron)을 사용하는 것이 좋습니다.

* `Rolling Ridley <https://docs.ros.org/en/rolling/Installation.html>`_ - Rolling 개발 버전
* `Iron Irwini <https://docs.ros.org/en/iron/Installation.html>`_ - 최신 안정버전 - May 2023
* `Humble Hawksbill <https://docs.ros.org/en/humble/Installation.html>`_ - LTS Release - May 2022

ROS 2 설치 튜토리얼을 진행할 때 진행 단계를 놓치기 쉽습니다. 다음 단계들에서 오류가 발생하면 ROS 2를 올바르게 설치했는지 확인하는 것이 좋은 시작점입니다.
사용자가 공통적으로 잊어버리는 것 중 하나는 ROS 2 설치 자체를 source하는 것입니다.
여러분이 설치한 ROS 버전을 source하는 것에 주의하십시오. ::

  source /opt/ros/iron/setup.bash

.. note:: ROS 1 설정 스크립트와 달리 ROS 2 설정 스크립트는 사용 중인 ROS 버전을 전환하려고 시도하지 않습니다. 이는 이전에 다른 버전의 ROS를 소스했거나 ``.bashrc`` 파일 내에서 source했을 경우 빌드 단계 중에 오류가 발생합니다. 이를 수정하려면 ``.bashrc``에서 source하는 내용을 변경하고 새 터미널을 시작하십시오.

`rosdep <http://wiki.ros.org/rosdep>`_ 를 설치하여 시스템 의존성을 설치합니다. : ::

  sudo apt install python3-rosdep

일단 ROS 2가 설치되면 가장 최신 패키지를 가지고 있는지 확인하십시오: ::

  sudo rosdep init
  rosdep update
  sudo apt update
  sudo apt dist-upgrade

ROS 2 빌드 시스템 :ros_documentation:`Colcon <Tutorials/Colcon-Tutorial.html#install-colcon>` 을 `mixin <https://github.com/colcon/colcon-mixin-repository>`_ 와 함께 설치합니다.: ::

  sudo apt install python3-colcon-common-extensions
  sudo apt install python3-colcon-mixin
  colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
  colcon mixin update default

`vcstool <https://index.ros.org/d/python3-vcstool/>`_ 을 설치합니다 : ::

  sudo apt install python3-vcstool

.. _create_colcon_workspace:

colcon 워크스페이스 만들기 및 튜토리얼 다운로드
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
튜터리얼을 위해서는 :ros_documentation:`colcon <Tutorials/Colcon-Tutorial.html#install-colcon>` 워크스페이스를 설정해야 합니다. ::

  mkdir -p ~/ws_moveit/src

MoveIt의 소스 코드와 튜터리얼을 다운로드
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Colcon 워크스페이스로 이동해서 MoveIt 튜토리얼 소스를 다운로드합니다.: ::

  cd ~/ws_moveit/src
  git clone https://github.com/ros-planning/moveit2_tutorials

다음으로 MoveIt의 나머지 소스 코드를 다운로드합니다. ::

  vcs import < moveit2_tutorials/moveit2_tutorials.repos

import 명령은 GitHub 자격 증명을 요청할 수 있습니다. 그냥 Enter를 누르면 계속 진행할 수 있습니다 ("Authentication failed" 오류를 무시하십시오).

Colcon 워크스페이스 빌드하기
^^^^^^^^^^^^^^^^^^^^^^^^^^^
다음 내용은 아직 워크스페이스에 대한 설치되지 않은 의존성 패키지를 Debian에서 설치합니다.
이 단계는 MoveIt과 모든 의존성을 설치하는 단계입니다.: ::

  sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

다음 명령은 Colcon 워크스페이스를 설정합니다.: ::

  cd ~/ws_moveit
  colcon build --mixin release


이 빌드 명령은 컴퓨터 속도와 사용 가능한 RAM(32GB 권장)에 따라 오랜 시간(20분 이상)이 걸릴 수 있습니다.

.. warning::
  이 명령으로 빌드하는 일부 패키지는 최대 16GB의 RAM을 필요로 합니다. 기본적으로 ``colcon`` 은 가능한 한 많은 패키지를 동시에 빌드하려고 시도합니다.
  컴퓨터 메모리 부족인 경우 또는 빌드가 일반적으로 컴퓨터에서 완료하는 데 문제가 있는 경우 위의 ``colcon`` 명령에 ``--executor sequential`` 을 추가하여 한 번에 한 패키지만 빌드하거나 ``--parallel-workers <X>`` 를 사용하여 동시 빌드 수를 제한할 수 있습니다.

모든 것이 순조롭게 진행되면 "Summary: X packages finished" 메시지가 표시되어야 합니다. 여기서 X는 50일 수도 있습니다. 문제가 발생하면 `ROS Installation <https://docs.ros.org/en/rolling/Installation.html>`_ 를 다시 확인하십시오.

Colcon 워크스페이스 설정
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Colcon 워크스페이스를 source하기: ::

  source ~/ws_moveit/install/setup.bash

옵션: ``.bashrc`` 에 이전 명령을 추가하기: ::

   echo 'source ~/ws_moveit/install/setup.bash' >> ~/.bashrc

.. note:: ``~/.bashrc`` 에서 자동으로 ``setup.bash`` 를 source하는 것은
   고급 사용자에게는 필요하지 않지만 단순화하기 위해서 이 방식을 추천합니다.

다음 단계
^^^^^^^^^
잘했습니다!
다음으로 :doc:`RViz을 위해서 상호작용하는 motion planning plugin을 사용하여 로봇 시각화하기 </doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial>` 를 해봅시다.
