pick_ik Kinematics Solver
=========================

`pick_ik <https://github.com/PickNikRobotics/pick_ik>`_ 은 PickNik Robotics가 개발한 MoveIt 2와 호환되는 inverse kinematics(IK) solver입니다. 강건하고 커스텀이 가능한 IK 솔루션을 제공하도록 설계되었으며 다양한 기능을 제공합니다.

``pick_ik``의 솔버는 전역 최적화 알고리즘과 국부 최적화 알고리즘을 통합한 `bio_ik <https://github.com/TAMS-Group/bio_ik>`_ 을 재구현한 것입니다. 전역 최적화 알고리즘은 진화 알고리즘을 사용하여 해 공간 내의 대안 솔루션을 탐색하고 전역 최적값을 식별합니다. 전역 최적화 알고리즘에서 얻은 결과를 기반으로 국부 최적화 알고리즘은 경사 하강법(gradient descent)를 적용하여 솔루션을 반복적으로 개선합니다. 이는 전역 최적화 알고리즘에서 제공하는 잠재적인 최적 솔루션을 입력으로 받아 정확도를 향상시키고 궁극적으로 가장 최적의 솔루션으로 수렴하는 것을 목표로 합니다..

시작하기
---------------
:doc:`Getting Started Guide </doc/tutorials/getting_started/getting_started>` 에 설명된 단계를 먼저 완료하세요.

또한, 여러분의 로봇에 맞게 특별히 수정한 MoveIt 설정 패키지가 필요합니다.
:doc:`MoveIt Setup Assistant </doc/examples/setup_assistant/setup_assistant_tutorial>` 를 사용하여 이 패키지를 만들 수 있습니다.

설치
------------

바이너리용
^^^^^^^^^^^^^
ROS 2 설치를 source 하고 다음 명령을 실행하십시오 ::

    sudo apt install ros-$ROS_DISTRO-pick-ik

소스용
^^^^^^^^^^^

colcon 워크스페이스를 생성하세요. ::

    export COLCON_WS=~/ws_moveit2/
    mkdir -p $COLCON_WS/src

워크스페이스의 src 디렉토리 내에 이 레포지토리를 복제하세요. ::

    cd $COLCON_WS/src
    git clone -b main https://github.com/PickNikRobotics/pick_ik.git

colcon mixins를 설정하세요. ::

    sudo apt install python3-colcon-common-extensions
    sudo apt install python3-colcon-mixin
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
    colcon mixin update default

워크스페이스를 빌드하세요. ::

    cd /path/to/your/workspace
    colcon build --mixin release

사용법
-------

pick_ik을 Kinematics 플러그인으로 사용하기
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

:doc:`MoveIt Setup Assistant </doc/examples/setup_assistant/setup_assistant_tutorial>` 을 사용하여 로봇에 대한 설정 파일을 만들어 MoveIt과 함께 사용하거나, 로봇의 설정 디렉토리에 있는 ``kinematics.yaml`` 파일을 편집하여 pick_ik을 IK 솔버로 사용할 수 있습니다. ::

    panda_arm:
        kinematics_solver: pick_ik/PickIkPlugin
        kinematics_solver_timeout: 0.05
        kinematics_solver_attempts: 3
        mode: global
        position_scale: 1.0
        rotation_scale: 0.5
        position_threshold: 0.001
        orientation_threshold: 0.01
        cost_threshold: 0.001
        minimal_displacement_weight: 0.0
        gd_step_size: 0.0001


.. note::

   다음 명령을 사용하여 ``pick_ik`` 을 사용하는 미리 설정된 데모를 실행할 수 있습니다.:

   .. code-block::

      ros2 launch moveit2_tutorials demo_pick_ik.launch.py

   이 명령은 :doc:`MoveIt Quickstart in RViz </doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial>` 튜토리얼과 유사한 데모를 시작하지만, 이 데모는 특히 ``moveit2_tutorials/doc/pick_ik/config`` 디렉토리에 있는 로봇 운동학 설정 파일 ``kinematics_pick_ik.yaml`` 을 사용합니다. :codedir:`here<how_to_guides/pick_ik/config/kinematics_pick_ik.yaml>`

파라미터 설명
^^^^^^^^^^^^^^^^^^^^^

파라미터의 전체 목록은 `파라미터 YAML file <https://github.com/PickNikRobotics/pick_ik/blob/main/src/pick_ik_parameters.yaml>`__ 에서 참조하십시오.

시작하기 좋은 몇 가지 주요 파라미터는 다음과 같습니다. :

- ``mode``: ``local`` 을 선택하면 이 솔버는 로컬 경사 하강만 수행하며, ``global`` 을 선택하면 진화 알고리즘도 사용하도록 설정됩니다. 글로벌 솔버를 사용하면 성능이 떨어지지만 로컬 최소값에서 벗어나는 데 문제가 있는 경우 도움이 될 수 있습니다. 상대 모션/데카르트 보간/엔드포인트 조깅과 같은 경우에는 ``local`` 을 사용하고, 초기 조건이 먼 목표를 해결해야 하는 경우에는 ``global`` 을 사용하는 것이 좋습니다.

- ``memetic_<property>``: ``global`` 솔버를 사용하는 경우에만 작동하는 모든 속성입니다. 주요 속성은 ``memetic_num_threads`` 이며, 진화 알고리즘이 여러 스레드에서 해결하도록 설정했습니다.

- ``position_threshold`` / ``orientation_threshold``: 최적화는 포즈 차이가 각각 미터와 라디안 단위의 이 임계값보다 작을 때만 성공합니다.
``position_threshold`` 가 0.001이면 1mm 정확도를 의미하며 ``orientation_threshold`` 가 0.01이면 0.01 라디안 정확도를 의미합니다.

- ``cost_threshold``: 이 솔버는 포즈가 얼마나 먼지, 초기 추측치에 상대적으로 얼마나 많이 조인트가 움직이는지, 사용자가 추가할 수 있는 사용자 정의 비용 함수를 기반으로 비용 함수를 설정하여 작동합니다.
최적화는 비용이 ``cost_threshold`` 보다 작을 때만 성공합니다. 커스텀 비용 함수를 추가하는 경우 이 임계값을 상당히 높게 설정하고
결정 요인으로 ``position_threshold`` 및 ``orientation_threshold`` 를 사용하는 것이 좋지만 이는 더 가이드라인에 가깝습니다.

- ``approximate_solution_position_threshold`` / ``approximate_solution_orientation_threshold``:
  end-point 서보 제어와 같은 응용 프로그램에서 근사 IK 솔루션을 사용할 때,
``pick_ik`` 는 때때로 목표 프레임과 상당히 먼 솔루션을 반환할 수 있습니다.
이러한 솔루션내에서의 점프 문제를 방지하기 위해 이러한 파라미터들은 최대 병진 및 회전 변위를 정의합니다.
대부분의 응용 프로그램에서 몇 센티미터와 몇 도 정도의 값으로 설정하는 것이 좋습니다.

- ``position_scale``: 회전만 IK를 원하는 경우 이 값을 0.0으로 설정합니다. 커스텀 ``IKCostFn`` (  ``setFromIK()`` 호출에서 제공)을 풀고 싶은 경우
``position_scale`` 과 ``rotation_scale`` 모두 0.0으로 설정합니다. 또한 다른 값을 사용하여 위치 목표 가중치를 지정할 수도 있습니다.
비용 함수의 일부입니다. ``position_scale = 0.0`` 을 사용하면 ``position_threshold`` 를 사용하는 모든 검사가 무시됩니다.


- ``rotation_scale``: 위치만 IK를 원하는 경우 이 값을 0.0으로 설정합니다. 위치와 방향을 동등하게 처리하려면 이 값을 1.0으로 설정합니다.
그 사이의 모든 값도 사용할 수 있습니다. 비용 함수의 일부입니다. ``rotation_scale = 0.0`` 을 사용하면 ``orientation_threshold`` 를 사용하는 모든 검사가 무시됩니다.

- ``minimal_displacement_weight``: 이것은 초기 추측치와 솔루션 간의 조인트 각도 차이를 확인하는 표준 비용 함수 중 하나입니다.
먼 목표를 풀고 있다면 0으로 두십시오. 경로를 따라 데카르트 보간 또는 서보 제어를 위한 엔드포인트 조깅과 같은 작업을 수행하는 경우 작지만 0이 아닌 값 (예: 0.001)으로 유지하십시오.

RViz에서 이 솔버를 실시간으로 테스트할 수 있습니다. 이 플러그인은 매번 풀이를 할때마다 파라미터 변경에 응답하기 위해 `generate_parameter_library <https://github.com/PickNikRobotics/generate_parameter_library>`_ 패키지를 사용합니다.
즉, ROS 2 명령줄 인터페이스를 사용하여 즉시 값을 변경할 수 있습니다. 예를 들면:

.. code-block::

    ros2 param set /rviz2 robot_description_kinematics.panda_arm.mode global

    ros2 param set /rviz2 robot_description_kinematics.panda_arm.minimal_displacement_weight 0.001
