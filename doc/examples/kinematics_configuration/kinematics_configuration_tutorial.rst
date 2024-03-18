:moveit1:

..
   Once updated for MoveIt 2, remove all lines above title (including this comment and :moveit1: tag)

Kinematics 설정
=================================

이번 섹션에서는 로봇의 운동학 설정을 위한 파라미터 몇 가지를 살펴보겠습니다.

The kinematics.yaml 파일
-----------------------------

MoveIt Setup Assistant가 생성하는 kinematics.yaml 파일은 MoveIt의 운동학 설정을 위한 주요 설정 파일입니다. :panda_codedir:`panda_moveit_config GitHub project <config/kinematics.yaml>` 에서 Panda 로봇의 전체 예시 파일을 볼 수 있습니다.: ::

 panda_arm:
   kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
   kinematics_solver_search_resolution: 0.005
   kinematics_solver_timeout: 0.05
   kinematics_solver_attempts: 3

파라미터들
^^^^^^^^^^
The set of available parameters include:
 * *kinematics_solver*: 운동학 솔버 플러그인의 이름입니다. 이는 플러그인 description 파일에 지정한 이름과 일치해야 합니다. 예를 들어 ``example_kinematics/ExampleKinematicsPlugin`` 
 * *kinematics_solver_search_resolution*: solver가 redundant 공간에서 역 운동학을 위해 검색하는 데 사용할 수 있는 resolution을 지정합니다. 예를 들어 7 DOF 팔의 경우 여분 조인트로 지정된 조인트 중 하나를 사용합니다.
 * *kinematics_solver_timeout*: 역 운동학 solver가 수행할 수 있는 각 내부 반복에 대해 지정된 기본 시간 제한(초)입니다. 일반적인 반복(예: 수치 솔버)은 시드 상태에서의 임의 재시작과 솔루션 사이클(이 시간 제한이 적용됨)로 구성됩니다. 솔버는 여러 번 재시작을 시도할 수 있으며, 기본 재시작 횟수는 아래의 kinematics_solver_attempts 매개 변수에 의해 정의됩니다.
 * *kinematics_solver_attempts*: solver에서 수행될 임의 재시작 횟수입니다. 재시작 후 각 솔루션 사이클에는 위의 kinematics_solver_timeout 파라미터에 의해 정의된 시간 제한이 적용됩니다. 일반적으로 이 시간 제한을 낮게 설정하여 개별 솔루션 사이클에서 빠르게 실패하는 것이 좋습니다.


The KDL Kinematics Plugin
^^^^^^^^^^^^^^^^^^^^^^^^^

KDL 운동학 플러그인은 Orocos KDL 패키지가 제공하는 수치 역 운동학 솔버를 래핑합니다.
 * 이 플러그인은 현재 MoveIt에서 사용되는 기본 운동학 플러그인입니다.
 * URDF에 지정된 조엔트 한계를 준수하며 (URDF에 안전 한계가 지정된 경우 안전 한계를 사용합니다).
 * KDL 운동학 플러그인은 현재 직렬 체인(seiral chain)에서만 작동합니다.

The LMA Kinematics Plugin
^^^^^^^^^^^^^^^^^^^^^^^^^

LMA (Levenberg-Marquardt) kinematics 플러그인은 Orocos KDL 패키지가 제공하는 수치 역 운동학 솔버를 래핑합니다.
 * URDF에 지정된 조인트 제한을 따르며 (URDF에 안전 제한이 지정되어 있으면 안전 제한을 사용합니다).
 * LMA kinematics 플러그인은 현재 직렬 연결만 지원합니다.
 * 사용 방법: ``kinematics_solver: lma_kinematics_plugin/LMAKinematicsPlugin`` 

The Cached IK Plugin
^^^^^^^^^^^^^^^^^^^^

Cached IK Kinematics Plugin은 IK 솔루션의 지속적인 캐시를 만듭니다. 이 캐시는 다른 모든 IK 솔버의 속도를 높이기 위해 사용됩니다. IK 솔버를 호출하면 캐시에서 유사한 상태를 IK 솔버의 시드(seed)로 사용합니다. 이 방법으로 솔루션을 찾지 못할 경우, 사용자가 지정한 시드 상태를 사용하여 다시 IK 솔버를 호출합니다. 캐시내에 있는 상태와 충분히 다른 새로운 IK 솔루션은 캐시에 추가시킵니다. 캐시는 주기적으로 디스크에 저장됩니다.

Cached IK Kinematics 플러그인을 사용하려면 로봇의 ``kinematics.yaml`` 파일을 수정해야 합니다. 다음과 같은 라인을 : ::

 manipulator:
   kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin

이렇게 변경하세요: ::

 manipulator:
   kinematics_solver: cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin
   # optional parameters for caching:
   max_cache_size: 10000
   min_pose_distance: 1
   min_joint_config_distance: 4

캐시 크기는 절대 상한값(``max_cache_size``) 또는 엔드 이펙터 포즈(``min_pose_distance``) 또는 로봇 조인트 상태(``min_joint_config_distance``)에 대한 거리 임계값으로 제어할 수 있습니다. 일반적으로 캐시 파일은 현재 작업 디렉토리(``${HOME}/.ros`` 이며, ``roslaunch`` 를 실행한 디렉토리가 아님)에 로봇별 하위 디렉토리로 저장됩니다. ``kinematics_solver`` 에 대해서 가능한 값은 다음과 같습니다.:

- *cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin*: 기본 KDL IK 솔버의 래퍼입니다.
- *cached_ik_kinematics_plugin/CachedSrvKinematicsPlugin*: 외부 IK 솔버와 통신하기 위해 ROS service 호출을 사용하는 솔버의 래퍼입니다.
- *cached_ik_kinematics_plugin/CachedTRACKinematicsPlugin*: `TRAC IK solver <https://bitbucket.org/traclabs/trac_ik>`_ 의 래퍼입니다. 이 솔버는 컴파일 타임에 TRAC IK kinematics 플러그인을 찾은 경우에만 사용할 수 있습니다.
- *cached_ik_kinematics_plugin/CachedUR5KinematicsPlugin*: UR5 arm용 분석 IK 솔버의 래퍼입니다 (UR3 및 UR10에도 유사한 솔버가 존재합니다). 이것은 예시 목적으로만 제공되며, 캐싱은 솔버에 추가적인 오버헤드를 야기합니다.

자세한 정보는 `Cached IK README <https://github.com/ros-planning/moveit/blob/master/moveit_kinematics/cached_ik_kinematics_plugin/README.md>`_ 를 참고합니다.

Position Only IK
----------------
KDL Kinematics Plugin을 사용하는 경우에만 kinematics.yaml 파일에 다음과 같은 라인을 추가하면 Position Only IK를 쉽게 활성화할 수 있습니다(IK를 풀고 싶은 특정 그룹에 대해).: ::

  position_only_ik: True
