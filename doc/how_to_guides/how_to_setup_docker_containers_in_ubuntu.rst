Ubuntu에서 MoveIt 2 Docker 컨테이너 설정 방법
===================================================
이 가이드는 MoveIt 2 의존 항목과 함께 Docker 컨테이너를 빠르게 설정하는 방법에 대한 과정을 제공합니다.
여기에는 docker-compose 설정 파일이 포함되어 있으며, 이를 사용하면 MoveIt에서 빠르게 작업을 시작할 수 있습니다.
이 가이드는 많은 설정을 수행하지 않고도 MoveIt 작업을 위한 별도 환경을 빠르게 구축하고 실행하고자 하는 사용자를 대상으로 합니다. 이 가이드에서는 ROS2 Rolling 환경을 설정합니다.

학습 목표
-------------------

- ``docker compose`` 를 사용하여 MoveIt 2 Docker 컨테이너 및 튜토리얼 실행 방법

요구사항
------------

- Ubuntu 20.04 혹은 22.04
- `Ubuntu용 Docker 설치 <https://docs.docker.com/engine/install/ubuntu/>`_
- `Docker용 Nvidia drivers <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit>`_
- `docker compose (Docker Desktop을 통해 설치하지 않은 경우) <https://docs.docker.com/compose/install/>`_

단계
-----
1. Docker 및 docker-compose 설치 (요구사항 섹션에 링크 참고). 그리고 반드시 `Linux Post Install <https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user>`_ 지침을 따르십시오. 이 추가 단계를 완료하지 않으면 모든 ``docker`` 명령 앞에 ``sudo`` 를 붙여야 합니다.

2. 터미널 세션 열기 및 docker-compose.yml 다운로드

  .. code-block:: bash

    wget https://raw.githubusercontent.com/ros-planning/moveit2_tutorials/main/.docker/docker-compose.yml

3. 컨테이너 실행 (docker-compose V1을 사용하는 경우 하이픈을 삽입해야 할 수 있음)
4. 컨테이너 실행 (V1을 사용하는 경우 ``docker-compose`` 에 하이펀을 삽입해야 할 수 있음)

   .. code-block:: bash

    DOCKER_IMAGE=rolling-tutorial docker compose run --rm --name moveit2_container gpu

   ``rolling-tutorial`` 을 다른 태그 이미지(예: ``humble-tutorial`` )로 바꿀 수 있습니다. 마찬가지로 Nvidia GPU 드라이버를 사용하지 않으려는 경우 ``gpu`` 를 ``cpu`` 로 바꾸고 컨테이너 이름을 ``moveit2_container`` 대신 다른 이름으로 변경할 수 있습니다. ``--rm`` 인자는 컨테이너를 정지(또는 종료)할 때 제거하며, 그렇지 않으면 수정된 컨테이너를 디스크에 유지하고 ``docker start moveit2_container`` 를 사용하여 시작할 수 있습니다.

5. 이제 완성된 :doc:`Planning Around Objects </doc/tutorials/planning_around_objects/planning_around_objects>` 와 :doc:`Pick and Place with MoveIt Task Constructor </doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor>` 튜토리얼과 함께 워크스페이스 디렉터리의 Docker 컨테이너 내부에 있어야 합니다. ``ros2 launch moveit2_tutorials demo.launch.py`` 와 같은 launch 명령 중 하나를 사용해보세요.

  다른 터미널을 통해 컨테이너에 접속하고 싶다면 다음 명령을 사용하세요:

   .. code-block:: bash

    docker exec -it moveit2_container /bin/bash

추가 읽을꺼리
---------------
- MoveIt과 관련된 Docker 모범 사례에 대한 자세한 내용은 PickNik의 Vatan Aksoy Tezer과 Brennard Pierce의 `이 블러그 포스트 <https://picknik.ai/ros/robotics/docker/2021/07/20/Vatan-Aksoy-Tezer-Docker.html>`_ 를 참조하세요.

- 태그 튜터리얼 이미지의 목록은 `here <https://github.com/ros-planning/moveit2_tutorials/pkgs/container/moveit2_tutorials>`__ 에서 찾을 수 있습니다.이미지 목록은 여기: [invalid URL removed] 찾을 수 있습니다. `here <https://hub.docker.com/r/moveit/moveit2/tags>`__ 있는 이미지들은 ``rolling-source`` 및 ``humble-source`` MoveIt 2 Docker 이미지를 기반으로 만들어졌으며, ``rolling`` 과 ``humble`` 의 두 가지 태그가 있습니다.

- `here <https://hub.docker.com/r/moveit/moveit2/tags>`__ 에서 MoveIt 2 Docker 컨테이너에 대한 더 많은 태그 이미지를 찾을 수 있습니다. 태그 이미지는 ROS2 버전 릴리스와 일치합니다. 컨테이너의 ``release`` 버전은 MoveIt 2가 바이너리를 통해 설치된 환경을 제공합니다. Docker 이미지의 ``source`` 버전은 MoveIt 2를 소스 코드에서 빌드합니다. 위 링크에서 태그 이름( ``rolling-source`` 등)을 사용하여 DOCKER_IMAGE 환경 변수를 대체하여 이 이미지 중 하나를 사용할 수 있지만, 대신 `이 docker-compose.yml <https://raw.githubusercontent.com/ros-planning/moveit2_tutorials/main/_scripts/docker-compose.yml>`_ 파일을 사용해야 합니다 (다른 위치로 복사하여 해당 위치에서 ``docker compose`` 명령어를 실행하기만 하면 됩니다).
