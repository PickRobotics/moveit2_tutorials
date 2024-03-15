이 사이트에 기여하는 방법
==============================

여기에서는 이 사이트에 성공적으로 변경 사항을 제출하는데 필요한 정보를 설명합니다.

학습 목표
-------------------
- 이 웹사이트를 로컬 빌드하는 방법
- 깨진 링크 확인 방법
- 철자 및 서식 확인 방법
- 코드 변경 사항을 검사하기 위해 로컬에서 CI 실행하는 방법

요구사항
------------
- Ubuntu 20.04
- ROS 2 Galactic
- Docker
- colcon 워크스페이스에 `the moveit2_tutorials repo <https://github.com/ros-planning/moveit2_tutorials>`_ 복사본 (없다면, :doc:`/doc/tutorials/getting_started/getting_started` 에서 워크스페이스 생성 과정을 안내합니다.)

단계
-----

1. 웹사이트를 로컬로 빌드 및 보기

  먼저, ``cd`` 명령으로 moveit2_tutorials 저장소의 root 디렉토리로 이동합니다. (:doc:`Getting Started </doc/tutorials/getting_started/getting_started>` 튜토리얼을 따랐다면, 이 위치는 ``~/ws_moveit/src/moveit2_tutorials`` 가 됩니다.) 해당 디렉토리에서 다음 명령어를 실행하십시오:

  .. code-block:: bash

    make html
    # Or run the following lines if you want to automatically rebuild the website on new changes
    while inotifywait -re modify,move,create,delete .; do
      make html
    done

  그런 다음 웹 브라우저에서 로컬 빌드된 사이트를 다음 위치에서 열 수 있습니다: ``./build/html/index.html``. 예를 들어, 기본 브라우저에서 로컬 사이트를 열려면 다음 명령어를 사용하십시오:

  .. code-block:: bash

    xdg-open ./build/html/index.html

2. 깨진 링크 테스트

  깨진 링크가 있는지 테스트하려면 다음 명령을 실행하십시오.:

  .. code-block:: bash

    ./htmlproofer.sh

3. 형식 검사 및 스펠링체크 실행하기

  형식 및 철자 검사기를 실행하기 위해 `pre-commit <https://pre-commit.com/>`_ 를 사용합니다.
  다음과 같이 pip을 사용하여 설치할 수 있습니다.:

  .. code-block:: bash

    python3 -m pip install --user pre-commit

  로컬에서 pre-commit을 실행하여 형식 및 철자 문제를 수정하려면:

  .. code-block:: bash

    pre-commit run --all

4. 로컬에서 industrial_ci 실행하여 CI 실행

  - 워크스페이스에 `industrial_ci <https://github.com/ros-industrial/industrial_ci>`_ 복사본을 복제합니다.

  - 빌드하고 워크스페이스를 source 합니다.

  - CI에서 수행하는 것처럼 코드 변경 사항을 테스트하려면 워크스페이스 디렉터리에서 이 명령을 실행하십시오.:

    .. code-block:: bash

      ros2 run industrial_ci rerun_ci src/moveit2_tutorials \
        DOCKER_IMAGE='moveit/moveit2:rolling-source' \
        UPSTREAM_WORKSPACE='moveit2_tutorials.repos' \
        TARGET_CMAKE_ARGS='-DCMAKE_BUILD_TYPE=Release' \
        CCACHE_DIR="$HOME/.ccache" \
        CLANG_TIDY='true'

추가 읽을꺼리
---------------

- :doc:`how_to_write_tutorials`
- :doc:`how_to_write_how_to_guides`
- :doc:`how_to_cross_reference`
