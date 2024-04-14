Robot Model 과 Robot State
===========================

.. image:: panda_tf.png
   :width: 700px
   :alt: "That is a sweet robot" you might say.

이 섹션에서는 MoveIt에서 운동학을 사용하기 위한 C++ API를 소개합니다.

RobotModel 과 RobotState Classes
-------------------------------------
:moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>` 과 :moveit_codedir:`RobotState<moveit_core/robot_state/include/moveit/robot_state/robot_state.h>` 클래스는 로봇의 운동학에 접근할 수 있는 핵심 클래스입니다.

:moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>` 클래스는 URDF에서 로딩된 모든 링크와 조인트 간의 관계, 그리고 이들의 조인트 제한 속성을 포함합니다. RobotModel은 또한 SRDF에서 정의된 플래닝 그룹으로 로봇의 링크와 조인트를 분리합니다. :doc:`URDF 및 SRDF에 대한 별도 튜토리얼 </doc/examples/urdf_srdf/urdf_srdf_tutorial>` 을 참고하세요.

:moveit_codedir:`RobotState<moveit_core/robot_state/include/moveit/robot_state/robot_state.h>` 는 특정 시점에서 로봇에 대한 정보를 포함하며, 조인트 위치 벡터 저장하며 옵션으로 속도 및 가속도를 저장합니다. 이 정보는 엔드 이펙터의 자코비안과 같은 현재 상태에 의존하는 로봇의 운동학 정보를 얻는데 사용할 수 있습니다.

RobotState는 또한 엔드 이펙터 위치(데카르트 포즈)를 기반으로 팔 위치 설정과 데카르트 궤적 계산을 위한 도움말 함수를 포함하고 있습니다.

이 예제에서는 이러한 클래스들을 Panda와 함께 사용하는 절차를 소개합니다.

시작하기
---------------
먼저 :doc:`Getting Started </doc/tutorials/getting_started/getting_started>` 를 완료하세요.

코드 실행하기
----------------
이 튜토리얼의 모든 코드는 MoveIt 설정 과정에서 가지고 있는 ``moveit2_tutorials`` package에서 컴파일하고 실행할 수 있습니다.

moveit2_tutorials 패키지에서 직접 코드를 실행하기 위해서 launch 파일을 실행합니다.::

 ros2 launch moveit2_tutorials robot_model_and_robot_state_tutorial.launch.py

예상 출력
---------------
예상 출력은 다음과 같은 형식입니다. 랜덤 조인트 값을 사용하기 때문에 숫자는 일치하지 않을 것입니다.: ::


 ... [robot_model_and_state_tutorial]: Model frame: world
 ... [robot_model_and_state_tutorial]: Joint panda_joint1: 0.000000
 ... [robot_model_and_state_tutorial]: Joint panda_joint2: 0.000000
 ... [robot_model_and_state_tutorial]: Joint panda_joint3: 0.000000
 ... [robot_model_and_state_tutorial]: Joint panda_joint4: 0.000000
 ... [robot_model_and_state_tutorial]: Joint panda_joint5: 0.000000
 ... [robot_model_and_state_tutorial]: Joint panda_joint6: 0.000000
 ... [robot_model_and_state_tutorial]: Joint panda_joint7: 0.000000
 ... [robot_model_and_state_tutorial]: Current state is not valid
 ... [robot_model_and_state_tutorial]: Current state is valid
 ... [robot_model_and_state_tutorial]: Translation:
 -0.368232
 0.645742
 0.752193

 ... [robot_model_and_state_tutorial]: Rotation:
  0.362374 -0.925408  -0.11093
  0.911735  0.327259  0.248275
  -0.193453 -0.191108  0.962317

 ... [robot_model_and_state_tutorial]: Joint panda_joint1: 2.263889
 ... [robot_model_and_state_tutorial]: Joint panda_joint2: 1.004608
 ... [robot_model_and_state_tutorial]: Joint panda_joint3: -1.125652
 ... [robot_model_and_state_tutorial]: Joint panda_joint4: -0.278822
 ... [robot_model_and_state_tutorial]: Joint panda_joint5: -2.150242
 ... [robot_model_and_state_tutorial]: Joint panda_joint6: 2.274891
 ... [robot_model_and_state_tutorial]: Joint panda_joint7: -0.774846
 ... [robot_model_and_state_tutorial]: Jacobian:
   -0.645742     -0.26783   -0.0742358    -0.315413    0.0224927    -0.031807 -2.77556e-17
   -0.368232     0.322474    0.0285092    -0.364197   0.00993438     0.072356  2.77556e-17
           0    -0.732023    -0.109128     0.218716   2.9777e-05     -0.11378 -1.04083e-17
           0    -0.769274    -0.539217     0.640569     -0.36792     -0.91475     -0.11093
           0    -0.638919      0.64923   -0.0973283     0.831769     -0.40402     0.248275
           1  4.89664e-12     0.536419     0.761708     0.415688  -0.00121099     0.962317


**Note:** 출력형태가 다른 ROS 콘솔 포맷이더라도 신경쓰지마세요.

전체 코드
---------------
전체 코드는 다음에서 볼 수 있습니다 :codedir:`here in the MoveIt GitHub project<examples/robot_model_and_robot_state/src/robot_model_and_robot_state_tutorial.cpp>`.

.. tutorial-formatter:: ./src/robot_model_and_robot_state_tutorial.cpp

Launch 파일
^^^^^^^^^^^^^^^
코드를 실행하려면 아래 2가지를 수행하는 launch 파일이 필요합니다.:
 * Panda의 URDF 와 SRDF를 파라미터 서버에 로드하고
 * 이 튜토리얼에서 클래스를 인스턴스화하는 노드의 네임스페이스내에 MoveIt Setup Assistant에서 생성한 kinematics_solver 설정을 ROS 파라미터 서버에 설정합니다.

.. literalinclude:: ./launch/robot_model_and_robot_state_tutorial.launch.py
