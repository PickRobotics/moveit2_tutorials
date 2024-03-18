:moveit1:

..
   Once updated for MoveIt 2, remove all lines above title (including this comment and :moveit1: tag)

Hand-Eye Calibration
====================
`MoveIt Calibration <http://www.github.com/ros-planning/moveit_calibration>`_ 패키지는 플러그인과 그래픽 인터페이스를 제공하여 손-눈 카메라 캘리브레이션을 수행할 수 있게 해줍니다.
캘리브레이션은 로봇 베이스 프레임에 단단히 장착된 카메라 (eye-to-hand)와 end-effector에 장착된 카메라 (eye-in-hand)에 대해 수행할 수 있습니다.
이 튜토리얼에서는 eye-in-hand 케이스를 다루고 있습니다.

.. image:: images/hand_eye_calibration_demo.jpg

시작하기
---------------
대부분의 튜토리얼은 시뮬레이션만으로도 진행할 수 있지만, 실제 캘리브레이션을 완료하려면 로봇 팔과 카메라가 필요합니다.

:doc:`Getting Started </doc/tutorials/getting_started/getting_started>` 을 먼저 완료하세요.
또한 :doc:`Setup Assistant Tutorial </doc/examples/setup_assistant/setup_assistant_tutorial>` 에 설명된대로 MoveIt와 함께 작동하도록 arm을 설정하십시오.

이 튜토리얼에서는 또한 카메라, 이미지 publish, 우수한 내부 캘리브레이션 매개 변수와 정확한 좌표 프레임을 가진 ``sensor_msgs/CameraInfo`` topic을 필요로 합니다. (필요한 경우 `camera_calibration <http://wiki.ros.org/camera_calibration>`_ 패키지를 사용하여 내부 카메라 캘리브레이션을 수행하십시오.)

MoveIt Calibration 저장소 복제 및 빌드하기
-------------------------------------------
워크스페이스의 ``src`` 디렉토리에 MoveIt Calibration을 복제합니다.::

  git clone git@github.com:ros-planning/moveit_calibration.git

다음으로 필요한 의존성 라이브러리가 설치되어 있는지 확인하고 패키지를 빌드합니다. ::

  rosdep install -y --from-paths . --ignore-src --rosdistro melodic
  catkin build
  source devel/setup.sh

RViz 실행 및 Calibration 플러그인 로드하기
--------------------------------------------------
로봇에 맞는 MoveIt 데모를 launch하세요. 예를 들어 ``roslaunch panda_moveit_config demo.launch`` 명령어를 사용할 수 있습니다.
RViz의 "Panels" 메뉴에서 "Add New Panel"를 선택하십시오.:

.. image:: images/choose_new_panel.png

그런 다음 "HandEyeCalibration" 패널 타입을 선택하십시오.:

.. image:: images/add_handeye_panel.png

"Target" 탭이 활성화된 상태로 패널이 추가됩니다.

타겟 생성 및 프린트
-------------------------
이제 시각적 calibration 타겟을 만들어 보겠습니다. 이 타겟은 이미지 데이터에서 쉽게 식별할 수 있는 독특한 패턴을 가지고 있으며, 타겟 크기를 측정하여 카메라 좌표계에서 타겟의 위치를 추정할 수 있습니다.
손-눈 calibration을 수행할 때, 타겟의 정확한 위치를 알 필요가 없습니다. 타겟이 로봇의 base 프레임에서 정지 상태만 유지되면 5개 이상의 포즈 시퀀스에서 손-눈 calibration을 추정할 수 있습니다.

"Target" 탭의 "Target Params" 섹션에서 기본 타겟 파라미터를 사용합니다.:

- **markers, X**: 3
- **markers, Y**: 4
- **marker size (px)**: 200
- **marker separation (px)**: 20
- **marker border (bits)**: 1
- **ArUco dictionary**: DICT_5X5_250

타겟 이미지를 생성하기 위해서 "Create Target" 버튼을 누릅니다.:

.. image:: images/aruco_target_handeye_panel.png

"Save Target" 버튼을 이용하여 타겟 이미지를 저장하고 출력하세요. 타겟 파라미터를 조정하며 어떤 영향을 미치는지 실험해보세요. 하지만 출력한 타겟에 사용된 파라미터 값을 반드시 기억하세요. 타겟을 인식하려면 동일한 파라미터 값을 입력해야 합니다.

카메라가 타겟을 정확하게 인식하려면 타겟이 평평해야 합니다. 평평한 바닥에 놓거나 판에 부착하는 것만으로도 충분합니다. 마커 너비(검은색 사각형 하나의 외곽 치수)와 마커 간의 간격 거리를 측정하세요. 측정한 값을 미터 단위로 "Target Params" 섹션의 해당 입력란에 입력하고, "Image Topic" 및 "CameraInfo Topic" 드롭다운 메뉴에서 적합한 항목을 선택하세요.

마지막으로 로봇 근처에 타겟을 놓아서 카메라가 쉽게 볼 수 있어야 합니다.

Geometric Context
-----------------
두 번째 탭인 "Context"에는 캘리브레이션 수행에 필요한 기하학적 정보가 포함되어 있습니다.

1. "Sensor configuration" 을 "Eye-in-hand" 로 설정하세요.
2. "Sensor frame" 은 카메라 광학 프레임입니다.( `REP 103
   <https://www.ros.org/reps/rep-0103.html>`_ 에 명시된 바와 같이 우측-하단-전방 기준을 사용합니다.)
3. "Object frame" 은 calibration 타겟에 의해 정의된 프레임이며, 기본적으로 "handeye_target"이라고 불립니다.
4. "End-effector frame"은 카메라에 견고하게 연결된 로봇 링크입니다.
5. "Robot base frame"은 calibration 타겟이 정지 상태인 프레임입니다.

.. image:: images/context_tab.png

FOV 섹션은 RViz에서 카메라의 시야각 렌더링을 제어합니다. FOV를 확인하려면 "MarkerArray" 디스플레이를 추가하고 "/rviz_visual_tools" topic을 수신하도록 설정하십시오.(즉시 나타나지 않을 수도 있음)

마지막으로 초기 카메라 포즈에 대한 추측값을 설정하지 않아도 되지만, 캘리브레이션을 계산한 후에는 이 필드가 새로운 캘리브레이션 값으로 업데이트된다는 점에 유의하십시오.

Collect Dataset
---------------
다음으로 캘리브레이션 데이터셋을 캡쳐합니다. 칼리브레이션을 잘 되려면 여러 샘플을 캡쳐해야합니다. 로봇 운동학은 로봇 base 프레임에서 엔드 이펙터의 포즈를 제공하며, 캘리브레이션 타겟의 포즈는 앞서 언급한 대로 카메라 프레임에서 추정할 수 있습니다.
로봇 base 프레임에서 타겟의 포즈를 정확하게 알고 있다면, 엔드-이펙터에 대한 카메라 포즈 변환을 복원하는 데 한번의 카메라-타겟 변환 관찰만으로 가능하다. 직접적인 카메라-대-엔드이펙터 변환은 복합적인 카메라-타겟-베이스-링크-엔드이펙터 변환과 동일합니다. 하지만 `Kostas Daniilidis의 논문 <https://scholar.google.com/scholar?cluster=11338617350721919587>`_ 에서 설명된 것처럼 여러 포즈의 정보를 결합하여 방정식에서 base 프레임에서의 타겟 포즈를 제거하는 것이 더 나은 선택입니다.

따라서 캘리브레이션 데이터셋의 각 샘플은 : 로봇 base 프레임에서의 엔드 이펙터 포즈와 카메라 프레임에서의 캘리브레이션 타겟 포즈의 쌍으로 구성됩니다.
이러한 샘플 5개를 수집하면 캘리브레이션을 계산할 수 있습니다.

"Calibrate" 탭은 데이터셋을 수집하고 캘리브레이션을 계산 및 내보내는 도구를 제공합니다.
이 시점에서 RViz 디스플레이에 이미지 패널을 추가하여 ``/handeye_calibration/target_detection`` 로 publish되는 카메라 뷰에서 타겟 감지를 확인하는 것이 유용합니다.

.. image:: images/calibrate_tab.png

"Calibrate" 탭에서 "AX=XB Solver" 드롭다운 메뉴를 사용하여 사용할 캘리브레이션 solver를 선택할 수 있습니다. Daniilidis solver(위의 참조 논문)가 기본값이며 대부분 상황에서 좋은 선택입니다. "Planning Group" 은 기록될 조인트 그룹이므로 팔에 대해서 적합한 그룹으로 설정해야 합니다(``panda_moveit_config`` package, ``panda_arm`` group 을 사용해야 함).

타겟이 암 카메라에 표시되고 축이 타겟 감지 이미지의 타겟에 렌더링되면 첫 번째 캘리브레이션 샘플(포즈 쌍)을 캡처할 준비가 된 것입니다.
"Manual calibration" 섹션에서 "Take sample" 버튼을 클릭하면 패널 왼쪽의 "Pose samples" 목록에 새로운 샘플이 추가됩니다.
샘플을 확장하면 Base-to-EndEffector와 Camera-to-Target의 두 변환을 포함하고 있음을 확인할 수 있습니다.

다음으로 "MotionPlanning" 패널을 사용하여 로봇 팔을 새로운 포즈로 이동하거나, 로봇의 티칭 펜던트 또는 프리 드라이브 모드(있는 경우)를 사용하여 다시 "Take sample" 버튼을 클릭할 수 있습니다.
각 포즈 쌍 사이에 약간의 회전을 포함시키고, 항상 동일한 축을 중심으로 회전하지 마십시오. 최소한 두 개의 회전 축이 필요합니다 (그 이유는 위의 링크된 Daniilidis 논문 참조).

수동 샘플을 가져오면 로봇 조인트 상태가 기록되므로 동일한 포즈를 다시 사용하여 향후에 다시 칼리브레이션을 할 수 있습니다.
기록된 상태의 수는 패널 하단의 진행률 표시줄 오른쪽에 표시되며, 해당 상태들은 "Settings" 섹션의 "Save joint states" 버튼을 사용하여 파일에 저장할 수 있습니다.

칼리브레이션 계산하기
-----------------------
다섯 개의 샘플을 수집하면, 칼리브레이션이 자동으로 수행되고, 새 샘플이 추가될 때마다 업데이트됩니다.
칼리브레이션은 몇 개 더 많은 샘플로 크게 향상되며 일반적으로 약 12~15개의 샘플 후에 정체됩니다.
위에서 언급한 대로 위치와 방향은 "Context" 탭에 표시되며 publish된 TF도 업데이트됩니다.
칼리브레이션 결과를 내보내려면 "Save camera pose"을 클릭하십시오.
이렇게하면 칼리브레이션된 카메라 변환을 포함하는 static transform(정적 변환) publisher를 가진 launch 파일이 생성되게 됩니다.
