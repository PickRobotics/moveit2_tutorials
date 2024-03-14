Warehouse - 영구적인 Scenes와 상태
===========================================

RViz의 "MotionPlanning" 플러그인은 완전한 계획 scene과 로봇 상태를 영구적으로 저장하는 기능을 제공합니다.
현재 두 가지 storage 플러그인 (`warehouse_ros <https://github.com/ros-planning/warehouse_ros>`_) 을 사용할 수 있습니다.:

* `warehouse_ros_mongo <https://github.com/ros-planning/warehouse_ros_mongo>`_, MongoDB를 백엔드로 사용합니다.
* `warehouse_ros_sqlite <https://github.com/ros-planning/warehouse_ros_sqlite>`_, SQLite를 백엔드로 사용합니다.

여러분이 선호하는 패키지 관리자를 사용하여 둘 다 설치할 수 있습니다 (예: ``apt-get install ros-{DISTRO}-warehouse-ros-mongo``) 혹은
:doc:`소스 코드에서 빌드 </doc/tutorials/getting_started/getting_started>` 할 수 있습니다.(물론, 이를 위해서는 관련 저장소를 src 폴더에 check out해야 합니다.)

Storage plugin 선택
------------------------

MoveIt 설정의 launch 파일에서 warehouse 플러그인과 설정이 지정되어야 합니다.
MongoDB 플러그인을 사용하지 않으려면 ``persistent_scenes_demo.launch.py`` 파일을 수정해야 합니다.
storage 플러그인은 ``warehouse_plugin`` 파라미터에 의해 결정됩니다.
유효한 옵션은 MongoDB의 경우 ``warehouse_ros_mongo::MongoDatabaseConnection`` 이고, SQLite의 경우 ``warehouse_ros_sqlite::DatabaseConnection`` 입니다.
또한, ``warehouse_host`` 와 ``warehouse_port`` 파라미터는 연결 세부 정보를 설정합니다.
SQLite 플러그인의 경우 ``warehouse_host`` 는 데이터베이스 파일 경로를 포함하며, ``warehouse_port`` 는 사용되지 않습니다.
데이터베이스 파일이 아직 없으면, 빈 데이터베이스가 생성됩니다.

.. tutorial-formatter:: ./launch/persistent_scenes_demo.launch.py

storage 백엔드에 연결하기
---------------------------------

데모를 실행하려면 ``git lfs install`` 명령을 실행하여 git lfs를 설치하고 [moveit_benchmark_resources](https://github.com/ros-planning/moveit_benchmark_resources.git) 를 워크스페이스에 복제해야 합니다.

storage 플러그인을 선택하고 launch.py 파일을 설정한 후 다음 명령으로 RViz를 실행하십시오 ::

   ros2 launch moveit2_tutorials persistent_scenes_demo.launch.py

RViz에서 "MotionPlanning" 창의 "Context" 탭으로 이동하십시오.
연결 세부 정보(MongoDB의 경우 host/port, SQLite의 경우 파일 경로)를 확인하고 "Connect" 버튼을 클릭하십시오.

.. image:: rviz_connect.png
    :width: 600px

그 후 대화 상자가 나타나 RViz에서 현재 모든 상태와 scenes을 지울지를 묻습니다 (데이터베이스는 아니며, 영구 데이터에는 영향을 받지 않습니다).
방금 RViz를 시작했으므로 안전하게 "yes"를 선택할 수 있습니다.

scenes와 states를 저장/로딩하기
--------------------------------

성공적으로 연결되었으므로 이제 로봇 상태와 계획된 scenes을 저장 및 복원할 수 있습니다.
이는 RViz의 "Stored Scenes" 또는 "Stored States" 탭에서 수행할 수 있습니다.

시작 상태를 저장하려면 녹색 manipulator를 원하는 위치로 끌고 "Save Start" 버튼을 클릭하십시오.
목표 상태 (오렌지색 manipulator)는 "Save Goal" 버튼을 사용하여 저장할 수 있습니다.
상태를 복원하려면 목록에서 선택한 다음 "Set as Start" 또는 ""Set as Goal"" 버튼을 클릭하십시오.
