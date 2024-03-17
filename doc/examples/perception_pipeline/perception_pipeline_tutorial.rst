:moveit1:

..
   Once updated for MoveIt 2, remove all lines above title (including this comment and :moveit1: tag)

Perception Pipeline 튜터리얼
============================

MoveIt은  `Octomap <http://octomap.github.io/>`_ 을 사용하여 3D 센서를 원활하게 통합할 수 있도록 해줍니다.
설정이 완료되면 rviz에서 다음과 같은 내용을 볼 수 있어야 합니다.:

.. image:: perception_configuration_demo.png
   :width: 700px

시작하기
---------------
먼저 :doc:`Getting Started </doc/tutorials/getting_started/getting_started>` 를 완료하세요..

설정
-------------

이 섹션에서는 MoveIt에서 로봇의 3D 센서를 설정하는 방법을 안내합니다. MoveIt에서 3D 인식을 처리하는 주요 컴포넌트는 Occupancy Map Updater 입니다. 이 updater는 플러그인 아키텍처를 사용하여 다양한 타입의 입력을 처리합니다. 현재 MoveIt에서 사용 가능한 플러그인은 다음과 같습니다.:

* The PointCloud Occupancy Map Updater: point clouds(``sensor_msgs/PointCloud2``)를 입력을 받을 수 있습니다.

* The Depth Image Occupancy Map Updater: Depth Images (``sensor_msgs/Image``)을 입력으로 받을 수 있습니다.

Occupancy Map Updater를 사용하려면 ROS parameter server에서 적절한 파라미터를 설정하고  ``PlanningSceneMonitor`` 에서 ``startWorldGeometryMonitor`` 를 호출하기만 하면 됩니다. 이 마지막 단계는 이 튜터리얼 예제와 같이 Move Group 기능을 사용할 때, 자동으로 수행되므로, 이 경우 octomap과 octomap updater에 대한 파라미터만 설정하면 됩니다.

YAML 설정 파일 (Point Cloud)
+++++++++++++++++++++++++++++++++++++

3D 센서를 설정하기 위한 YAML 설정 파일을 생성해야 합니다. point cloud 처리를 위해서 :panda_codedir:`예제 파일<config/sensors_kinect_pointcloud.yaml>` 을 확인하세요.

이 파일을 로봇의 moveit_config 패키지 내 config 폴더에 "sensors_kinect_pointcloud.yaml"라는 이름으로 저장하십시오.: ::

 sensors:
   - sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
     point_cloud_topic: /camera/depth_registered/points
     max_range: 5.0
     point_subsample: 1
     padding_offset: 0.1
     padding_scale: 1.0
     max_update_rate: 1.0
     filtered_cloud_topic: filtered_cloud

**일반 파라미터들:**

* *sensor_plugin*: 사용하는 플러그인의 이름

* *max_update_rate*: octomap 표현은 이 값 이하 또는 같은 속도로 업데이트됩니다.

**Point cloud updater 전용 파라미터들:**

* *point_cloud_topic*: point cloud를 수신할 topic을 지정합니다.

* *max_range*:  (m 단위) 이 거리보다 먼 포인트는 사용되지 않습니다.

* *point_subsample*: *point_subsample* points 중 1개의 포인트를 선택합니다.

* *padding_offset*: (cm 단위) 패딩 사이즈

* *padding_scale*: 패딩 스케일

* *filtered_cloud_topic*: 필터링된 cloud가 publish되는 topic입니다 (주로 디버깅용). 필터링된 cloud는 자체 필터링이 수행된 후 결과 cloud입니다.


YAML 설정 파일 (Depth Map)
+++++++++++++++++++++++++++++++++++

3D 센서 설정을 위한 YAML 설정 파일을 생성해야 합니다. :panda_codedir:`깊이 이미지 처리를 위한 예제 파일 <config/sensors_kinect_depthmap.yaml>` 은 panda_moveit_config 저장소에서도 찾을 수 있습니다.
이 파일을 로봇의 moveit_config 패키지 내 config 폴더에 "sensors_kinect_depthmap.yaml" 라는 이름으로 저장하십시오.: ::

 sensors:
   - sensor_plugin: occupancy_map_monitor/DepthImageOctomapUpdater
     image_topic: /camera/depth_registered/image_raw
     queue_size: 5
     near_clipping_plane_distance: 0.3
     far_clipping_plane_distance: 5.0
     shadow_threshold: 0.2
     padding_scale: 4.0
     padding_offset: 0.03
     max_update_rate: 1.0
     filtered_cloud_topic: filtered_cloud

**일반 파라미터들:**

* *sensor_plugin*: 사용하는 플러그인의 이름
* *max_update_rate*: octomap 표현은 이 값 이하 또는 같은 속도로 업데이트됩니다.

**Depth Map updater에 특화된 파라미터들:**

* *image_topic*: depth image를 위해 수신할 topic을 지정합니다.

* *queue_size*: queue에 넣을 이미지의 개수

* *near_clipping_plane_distance*: 가시 거리 이전의 최소 거리

* *far_clipping_plane_distance*: 가시 거리 이후의 최대 거리

* *shadow_threshold*: entity 아래의 다이내믹 섀도가 표시되기 위한 섀도 맵의 최소 밝기입니다.

* *padding_offset*: 패딩 사이즈 (cm)

* *padding_scale*: 패딩의 스케일

* *filtered_cloud_topic*: 필터링된 cloud가 publish될 topic입니다 (주로 디버깅용). 필터링된 cloud는 자체 필터링이 수행된 후 결과 cloud입니다.


launch 파일 업데이트
++++++++++++++++++++++++++

launch 스크립트에 YAML 파일 추가하기
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
이제 panda_moveit_config 디렉토리의 "launch" 디렉토리에 있는 *sensor_manager.launch* 파일을 이 센서 정보로 업데이트해야 합니다 (이 파일은 Setup Assistant에 의해 자동 생성되지만 비어 있습니다). 해당 파일에 다음과 같은 라인을 추가하여 MoveIt에서 사용할 센서 소스 세트를 설정해야 합니다.: ::

 <rosparam command="load" file="$(find panda_moveit_config)/config/sensors_kinect_pointcloud.yaml" />

depthmap을 사용하는 경우, yaml 파일의 이름을 ``sensors_kinect_depthmap.yaml`` 로 변경하세요.
위에서 생성한 올바른 파일 경로를 입력해야 합니다.

Octomap 설정
^^^^^^^^^^^^^^^^^^^^^
또한 *sensor_manager.launch* 파일에 다음과 같은 라인을 추가하여 `Octomap <http://octomap.github.io/>`_ 을 설정해야 합니다.: ::

 <param name="octomap_frame" type="string" value="odom_combined" />
 <param name="octomap_resolution" type="double" value="0.05" />
 <param name="max_range" type="double" value="5.0" />

MoveIt은 주변 world를 표현하기 위해 octree-based 프레임워크를 사용합니다. 위의 *Octomap* 파라미터는 이 표현에 대한 설정 파라미터입니다.:
 * *octomap_frame*: 이 표현이 저장될 좌표계 프레임을 지정합니다. 모바일 로봇과 함께 작업하는 경우, 이 프레임은 world의 고정된 프레임이어야 합니다.
 * *octomap_resolution*: 이 표현을 유지할 resolution(미터 단위)를 지정합니다.
 * *max_range*: 이 node에 대한 모든 센서 입력에 적용될 최대 범위 값을 지정합니다.

장애물 회피
------------------

로봇의 시작 위치와 목표 위치 사이에 직선 경로가 없도록 설정하면, planner가 자동으로 octomap을 피하고 경로를 계획합니다.

.. image:: obstacle_avoidance.gif
   :width: 700px

인터페이스 실행하기
+++++++++++++++++++++
moveit_tutorials에서 roslaunch 명령어를 이용하여 launch 파일을 실행시키세요.: ::

 roslaunch moveit_tutorials obstacle_avoidance_demo.launch

이 튜토리얼 처음 부분에 나온 이미지와 같은 화면을 볼수 있습니다.
만약 이미지와 다른 화면이 나타난다면, `known OpenGL rendering issue <http://wiki.ros.org/rviz/Troubleshooting>`_ 문제가 발생했을 수 있습니다. 이 문제를 해결하기 위해서는 다음 명령어를 사용하여 CPU 기반 렌더링을 강제로 설정할 수 있습니다.:

 export LIBGL_ALWAYS_SOFTWARE=1

목표 위치를 직접 설정한 후, 경로 계획 및 실행을 통해 장애물 회피 기능을 직접 테스트해 볼 수 있습니다. 테스트 방법에 대한 자세한 내용은 :doc:`MoveIt Quickstart in RViz </doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial>` 문서를 참고하세요.

Collision Object로 물체 감지 및 추가하기
-----------------------------------------------

이 절에서는 point cloud로부터 실린더 형태의 물체를 추출하고, 관련 값을 계산하여 계획 화면(planning scene)에 충돌 객체(collision object)로 추가하는 방법에 대한 예시를 설명합니다.
point cloud 데이터를 사용하는 예제지만, depth maps를 사용하는 경우에도 유사하게 구현할 수 있습니다.

코드를 실행한 후에는 RViz에서 다음과 같은 화면을 볼 수 있어야 합니다.:

.. image:: cylinder_collision_object.png
   :width: 700px

코드 실행하기
++++++++++++++++
moveit_tutorials에서 roslaunch 명령어를 이용하여 launch 파일을 실행시키세요.: ::

 roslaunch moveit_tutorials detect_and_add_cylinder_collision_object_demo.launch

알려진 이슈 - 데모를 실행할때 아래와 같은 에러가 발생할 수 있습니다 ::

  ros.moveit_ros_planning.planning_scene_monitor: Transform error: Lookup would require extrapolation into the future.  Requested time 1527473962.793050157 but the latest data is at time 1527473962.776993978, when looking up transform from frame [panda_link2] to frame [camera_rgb_optical_frame]
  ros.moveit_ros_perception: Transform cache was not updated. Self-filtering may fail.

현재 수정 중이며, 데모 동작을 멈추게 하지는 않습니다.
`issue tracker <https://github.com/ros-planning/moveit_tutorials/issues/192>`_ 에서 상태를 확인할 수 있습니다.

관련 코드
+++++++++++++++
전체 코드는 moveit_tutorials GitHub 프로젝트 :codedir:`here<examples/perception_pipeline>` 에서 확인할 수 있습니다.

이 튜터리얼은 각각의 perception pipeline 기능 구현에 대한 세부 사항을 설명하지 않습니다. 이러한 세부 사항은 `여기 <http://wiki.ros.org/pcl/Tutorials>`_ 에 잘 문서화되어 있기 때문입니다.

.. |br| raw:: html

   <br />

.. |code_start| raw:: html

   <code>

.. |code_end| raw:: html

   </code>

.. tutorial-formatter:: ./src/cylinder_segment.cpp
