Kinematics Cost Functions(운동학 비용 함수)
===============================================

IK 솔루션을 쿼리할 때 goal pose 또는 데카르트 경로 목표에 대한 비용 함수를 지정할 수 있습니다. 이 *cost functions* 는 솔루션의 적합성을 평가하는 데 사용됩니다.

시작하기
---------------
:doc:`Getting Started </doc/tutorials/getting_started/getting_started>` 를 먼저 완료하세요.

이 튜터리얼에서 또한 역 운동학 플러그인으로 `bio_ik <https://github.com/PickNikRobotics/bio_ik>`_ 를 사용해야 합니다. 먼저 이 저장소의 "ros2" 브랜치를 작업 공간의 src 디렉토리에 복제하십시오: ::

  git clone -b ros2 https://github.com/PickNikRobotics/bio_ik.git

이제는 워크스페이스 내에 있는 moveit_resources/panda_moveit_config/config/kinematics.yaml 파일에 아래 내용을 복사하여 Panda 로봇의 운동학 플러그(kinematics plugin)인을 변경하십시오.: ::

    panda_arm:
        kinematics_solver: bio_ik/BioIKKinematicsPlugin
        kinematics_solver_search_resolution: 0.005
        kinematics_solver_timeout: 0.005
        kinematics_solver_attempts: 1
        mode: gd_c # use the gradient descent solver

변경한 후에 워크스페이스를 다시 빌드하세요: ::

  colcon build --mixin release

코드 실행하기
----------------
두 개의 터미널을 열어주세요. 첫 번째 터미널에서 RViz를 시작하고 모든 로딩이 완료될 때까지 기다리십시오.: ::

  ros2 launch moveit2_tutorials move_group.launch.py

2번째 터미널에서 튜터리얼 launch 파일을 실행시키세요: ::

  ros2 launch moveit2_tutorials kinematics_cost_function_tutorial.launch.py

각 데모 단계를 진행하려면 화면 하단의 **RvizVisualToolsGui** 패널에서 **Next** 버튼을 누르거나, 화면 상단의 **Tools** 패널에서 **Key Tool** 를 선택한 다음 RViz에 포커스가 맞춰져 있는 상태에서 키보드의 **0** 을 누르세요.

예상 출력
---------------
RViz에서 다음과 같은 내용을 확인할 수 있어야 합니다.:
 1. 로봇은 팔을 오른쪽에 있는 포즈 목표(pose goal)로 이동합니다. 비용 함수를 지정한 경우와 지정하지 않은 경우의 joint 이동의 L2 norm이 튜토리얼 터미널에 기록됩니다.
 2. 로봇 팔이 직선 데카르트 이동을 통해 왼쪽의 포즈 목표(pose goal)로 이동합니다.

전체 Code
---------------
전체 코드는 :codedir:`here in the MoveIt GitHub project<how_to_guides/kinematics_cost_function/src/kinematics_cost_function_tutorial.cpp>` 에서 볼 수 있습니다. 다음은 코드를 하나씩 살펴보면서 기능을 설명합니다.

.. tutorial-formatter:: ./src/kinematics_cost_function_tutorial.cpp

Launch 파일
---------------
전체 launch 파일은 Github의 :codedir:`here<how_to_guides/kinematics_cost_function/launch/kinematics_cost_function_tutorial.launch.py>` 에 있습니다. 이 튜토리얼의 모든 코드는 MoveIt setup의 일부인 **moveit2_tutorials** 패키지에서 실행할 수 있습니다.
