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
다음으로 캘리브레이션 데이터셋을 캡쳐합니다. 칼리브레이션을 잘 하려면 여러 샘플을 캡쳐해야합니다. 로봇 운동학은 로봇 base 프레임에서 엔드 이펙터의 포즈를 제공하며, 캘리브레이션 타겟의 포즈는 앞서 언급한 대로 카메라 프레임에서 추정할 수 있습니다. 로봇 기본 프레임에서 타겟의 포즈를 정확하게 알고 있다면, 카메라-대-엔드-이펙터 변환을 복원하는 데 단일 카메라-타겟 변환 관찰만 필요합니다. 직접적인 카메라-대-엔드-이펙터 변환은 복합적인 카메라-타겟-베이스-링크-엔드-이펙터 변환과 동일합니다. 하지만 `Kostas Daniilidis의 논문 <https://scholar.google.com/scholar?cluster=11338617350721919587>`_ 에서 설명된 것처럼 여러 포즈의 정보를 결합하여 방정식에서 기본 프레임에서의 타겟 포즈를 제거하는 것이 더 나은 선택입니다.
Next, we will capture a calibration dataset. We need to capture several samples to ensure a good calibration. The robot
kinematics provide the end-effector's pose in the robot base frame, and the calibration target's pose in the camera
frame can be estimated, as mentioned above. If the target's pose in the robot base frame were known accurately, only a
single observation of the camera-target transform would be necessary to recover the camera's pose in the end-effector
frame. The direct camera-to-end-effector transform is equivalent to the composite
camera-to-target-to-base-link-to-end-effector transform. A better option, however, is to combine the information from
several poses to eliminate the target pose in the base frame from the equation, as described in `this paper by Kostas
Daniilidis <https://scholar.google.com/scholar?cluster=11338617350721919587>`_.

따라서 캘리브레이션 데이터셋의 각 샘플은 로봇 기본 프레임에서의 엔드 이펙터 포즈와 카메라 프레임에서의 캘리브레이션 타겟 포즈의 쌍으로 구성됩니다. 이러한 샘플 5개를 수집하면 캘리브레이션을 계산할 수 있습니다.
Each sample in our calibration dataset, then, comprises a pair of poses: the end-effector's pose in the robot base frame
paired with the calibration target's pose in the camera frame. Once five such samples have been collected, the
calibration can be calculated.

"캘리브레이션" 탭은 데이터셋을 수집하고 캘리브레이션을 계산 및 내보내는 도구를 제공합니다. 이 시점에서 RViz 디스플레이에 이미지 패널을 추가하여 /handeye_calibration/target_detection에 게시된 카메라 뷰에서 타겟 감지를 확인하는 것이 유용합니다.The "Calibrate" tab provides the tools to collect the dataset and calculate and export the calibration. At this point,
it is also helpful to add an image panel to the RViz display to see the target detection in the camera view, which is
published on ``/handeye_calibration/target_detection``.

.. image:: images/calibrate_tab.png

"캘리브레이션" 탭에서 "AX=XB 솔버" 드롭다운 메뉴를 사용하여 사용할 캘리브레이션 솔버를 선택할 수 있습니다. Daniilidis 솔버(위의 참조 논문)가 기본값이며 대부분 상황에서 좋은 선택입니다. "플래닝 그룹"은 기록될 조인트 그룹이므로 팔에 적합한 그룹으로 설정해야 합니다(panda_moveit_config 패키지에서 panda_arm 그룹을 사용해야 함).On the "Calibrate" tab, you can select which calibration solver to use in the "AX=XB Solver" drop-down. The Daniilidis
solver (from the paper referenced, above) is the default and is a good choice in most situations. The "Planning Group"
is the joint group that will be recorded, so should be set to the appropriate group for the arm (in the
``panda_moveit_config`` package, the ``panda_arm`` group should be used).

타겟이 암 카메라에 표시되고 축이 타겟 감지 이미지의 타겟에 렌더링되면 첫 번째 캘리브레이션 샘플(포즈 쌍)을 캡처할 준비가 된 것입니다. "수동 캘리브레이션" 섹션에서 "샘플 캡처" 버튼을 클릭하면 패널 왼쪽의 "포즈 샘플" 목록에 새로운 샘플이 추가됩니다. 샘플을 확장하면 베이스-투-엔드-이펙터와 카메라-투-타겟의 두 변환을 포함하고 있음을 확인할 수 있습니다.When the target is visible in the arm camera, and the axis is rendered on the target in the target detection image, you
are ready to take your first calibration sample (pose pair). Click the "Take sample" button in the "Manual calibration"
section, and a new sample will be added to the "Pose samples" list on the left side of the panel. If you expand a
sample, you will see it contains two transforms, base-to-end-effector, and camera-to-target.

Next, you can move the arm to a new pose using the "MotionPlanning" panel, or use your robot's teaching pendant or free
drive mode, if it has one, and click "Take sample" again. Be sure to include some rotation between each pair of poses,
and don't always rotate around the same axis--at least two rotation axes are needed to uniquely solve for the
calibration (see the Daniilidis paper, linked above, for the explanation why).

As you take manual samples, the robot joint states are recorded, so that the same poses can be used again to
recalibrate in the future. The number of recorded states is shown to the right of the progress bar at the bottom of the
panel, and the states can be saved to a file using the "Save joint states" button in the "Settings" section.

Calculate a Calibration
-----------------------
Once you have collected five samples, a calibration will be performed automatically, and updated each time a new sample
is added. The calibration will improve significantly with a few more samples, and will typically plateau after about 12
or 15 samples. The position and orientation will be displayed on the "Context" tab, as mentioned above, and the
published TF will be updated as well. Click "Save camera pose" to export the calibration result. This will create a
launch file with a static transform publisher containing the calibrated camera transform.
