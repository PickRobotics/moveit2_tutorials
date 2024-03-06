======================
Planning Scene Monitor
======================

``planning scene``은 로봇 주변 world의 표현과 로봇 자체의 상태를 저장하는데 사용되는 객체입니다. ``planning_scene`` 객체의 내부 상태는 일반적으로 ``planning_scene_monitor`` component에 의해 관리되며, 이 component는 thread-safe 방식으로 상태 읽기 및 쓰기를 가능하게 합니다.

.. image:: /_static/images/planning_scene_monitor.svg

World Geometry Monitor
----------------------

world geometry monitor는 로봇의 LiDAR, depth 카메라와 같은 센서 정보와 사용자 입력을 사용하여 world geometry를 구축합니다.
이 모니터는 아래 설명하는 ``occupancy map monitor``를 사용하여 로봇 주변 환경의 3D 표현을 구축하고 ``planning_scene`` topic에 있는 객체 정보를 추가하여 이를 보강합니다.

3D Perception
-------------

MoveIt에서 3D 인식은 ``occupancy map monitor``가 처리합니다. occupancy map monitor는 플러그인 아키텍처를 사용하여 위 그림과 같이 다양한 종류의 센서 입력을 처리합니다. 특히, MoveIt은 아래 2가지 종류의 입력 처리를 기본 기능으로 지원합니다.:

- 포인트 클라우드(**Point clouds**): ``PointCloudOccupancyMapUpdater`` 플러그인에 의해 처리됩니다.

- 깊이 이미지(**Depth images**): ``DepthImageOccupancyMapUpdater`` 플러그인에 의해 처리됩니다.

occupancy map monitor에 플러그인 형태로 사용자 정의 업데이터 타입을 추가할 수도 있습니다.

Octomap
-------

Occupancy map monitor는 `Octomap <https://octomap.github.io/>`_ 을 사용하여 환경의 occupancy map를 유지 관리합니다.
Octomap은 실제로 개별 셀에 대한 확률 정보를 인코딩할 수 있지만, 이 정보는 현재 MoveIt에서 사용되지 않습니다.
Octomap은 MoveIt에서 사용하는 충돌 검사 라이브러리인 FCL에 직접 전달할 수 있습니다.

Depth Image Occupancy Map Updater
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

depth image occupancy map 업데이터는 자체 필터(*self-filter*)를 포함합니다. 즉 로봇의 시각적인 부분을 depth map에서 제거하는 기능을 포함합니다.
이 작업을 수행하기 위해 로봇(robot state)에 대한 현재 정보를 사용합니다.
