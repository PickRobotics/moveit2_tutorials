CHOMP Planner 사용하기
=========================

.. image:: chomp.png
   :width: 700px

변곡 공변량 해밀턴 최적화 운동 계획(Covariant Hamiltonian Optimization for Motion Planning (CHOMP))은 많은 일상적인 운동 계획 문제를 간단하게 만들고 훈련 가능하게 해주는 기울기 기반 궤적 최적화 절차입니다 (Ratliff et al., 2009c). 대부분의 고차원 운동 계획은 궤적 생성을 별개의 계획 단계와 최적화 단계로 나누지만, 이 알고리즘은 공변 기울기와 함수적 기울기 접근 방식을 최적화 단계에 활용하여 완전히 궤적 최적화에 기반한 운동 계획 알고리즘을 설계합니다. 실행 불가능한 단순 궤적이 주어진 경우, CHOMP는 주변 환경에 반응하여 빠르게 충돌하지 않는 궤적을 얻고, 동시에 조인트 속도 및 가속도와 같은 동적 변수를 최적화합니다. 이는 로봇에서 효율적으로 실행할 수 있는 부드럽고 충돌 없는 궤적으로 빠르게 수렴시킵니다. `더 자세한 정보 <http://www.nathanratliff.com/thesis-research/chomp>`_ 를 참고하세요.

시작하기
---------------
:doc:`Getting Started </doc/tutorials/getting_started/getting_started>` 을 먼저 완료합니다.

:doc:`Visualization with MoveIt RViz Plugin </doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial>` 문서도 완료합니다.

사전준비
--------------
CHOMP를 로봇과 함께 사용하려면 로봇에 맞는 MoveIt 설정 패키지가 필요합니다. 예를 들어 Panda 로봇을 사용하는 경우 패키지 이름은 ``panda_moveit_config`` 이며 :moveit_resources_codedir:`here <panda_moveit_config/>` 위치에서 찾을 수 있습니다. 일반적으로 이러한 설정은 :doc:`MoveIt Setup Assistant </doc/examples/setup_assistant/setup_assistant_tutorial>` 를 사용하여 설정합니다..

여러분 로봇에 CHOMP 사용하기
---------------------------
**Note:** `ros-planning/moveit_resources <https://github.com/ros-planning/moveit_resources/tree/ros2>`_ 저장소에서 ``panda_moveit_config`` 패키지를 사용하려는 경우, 이 단계는 이미 완료되었으므로 이 섹션을 건너뛸 수 있습니다. 그렇지 않은 경우에는 로봇의 설정을 추가하려면 다음을 수행해야만 합니다.:

#. MoveIt 설정 패키지의 launch 디렉토리내에 :codedir:`chomp_demo.launch.py<examples/chomp_planner/launch/chomp_demo.launch.py>` 파일을 만듭니다.
#. ``chomp_demo.launch.py`` 에서 Panda에 대한 모든 참조를 커스텀 설정을 가리키도록 수정합니다.
#. MoveIt 설정 패키지의 config 디렉토리에 :moveit_resources_codedir:`chomp_planning.yaml <panda_moveit_config/config/chomp_planning.yaml>` 파일이 포함되어 있는지 확인합니다.
#. ``chomp_planning.yaml`` 파일을 편집기로 열고 ``animate_endeffector_segment: "panda_rightfinger"`` 를 로봇에 적합한 링크 이름으로 변경합니다. 필요에 따라 다른 파라미터도 수정할 수 있습니다.

데모 실행하기
----------------
``panda_moveit_config`` 패키지가 `ros-planning/moveit_resources <https://github.com/ros-planning/moveit_resources/tree/ros2>`_ 레포지토리에 있고  moveit2_tutorials 패키지도 있다면 다음 명령어를 이용하여 데모를 실행할 수 있습니다.
If you have the ``panda_moveit_config`` from the `ros-planning/moveit_resources <https://github.com/ros-planning/moveit_resources/tree/ros2>`_  repository along with ``moveit2_tutorials`` you can run the demo using: ::

  ros2 launch moveit2_tutorials chomp_demo.launch.py rviz_tutorial:=True

Note: 편의를 위해 RViz 설정 파일을 제공하지만,  ``rviz_tutorial``  값을  False 로 설정하거나 생략하면 여러분의 선호에 따라 RViz를 설정할 수 있습니다.

Scene에 장애물 추가하기
+++++++++++++++++++++++++++++
장애물을 scene에 추가하려면 :codedir:`this node<examples/collision_environments/src/collision_scene_example.cpp>` 코드를 사용하여 장애물이 있는 scene을 생성할 수 있습니다.

장애물이 있는 상황에서 CHOMP 플래너를 실행하려면 두 번째 터미널을 여십시오. 첫 번째 터미널에서 (이전 단계의 터미널을 닫았다면 다시 열고) RViz를 시작하고 모든 로딩이 완료될 때까지 기다리십시오.: ::

  ros2 launch moveit2_tutorials chomp_demo.launch.py rviz_tutorial:=True

2번째 터미널에서 아래 명령을 실행합니다: ::

  ros2 run moveit2_tutorials collision_scene_example

그 다음은 RViz에서 MotionPlanning 패널의 Context 탭에서  CHOMP를 선택하십시오. 마커를 사용하여 end-effector를 원하는 시작 및 목표 위치로 이동하여 상태를 설정한 다음 MotionPlanning 패널의  Planning 탭에서 Plan 버튼을 클릭하여 계획을 시작하십시오. 이제 플래너는 지정된 시작 위치와 목표 위치 사이에서 실행 가능한 경로를 찾으려고 시도합니다.

CHOMP의 파라미터 수정하기
-----------------------------------------
CHOMP 알고리즘은 최적화 파라미터를 몇 가지 가지고 있습니다. 이 파라미터들은 사용하고 있는 로봇과 환경에 맞게 수정할 수 있으며, 보통 로봇 설정 폴더 내의 :moveit_resources_codedir:`chomp_planning.yaml <panda_moveit_config/config/chomp_planning.yaml>` 에 위치하고 있습니다. 해당 파일이 없으면 직접 생성하여 원하는 파라미터 값을 설정할 수 있습니다. ``chomp_planning.yaml`` 내에 파라미터 값이 어떤 용도로 사용되는지에 대한 정보는 아래와 같습니다:

- *planning_time_limit*: 최적화 과정에서 솔루션을 찾기까지 최대 시간 제한입니다.

- *max_iterations*: planner가 최적화 동안 적절한 솔루션을 찾기 위한 최대 반복 횟수입니다.

- *max_iterations_after_collision_free*: 충돌 없는 경로를 찾은 후 수행하는 최대 반복 횟수입니다.

- *smoothness_cost_weight*:  CHOMP의 최적화를 위한 마지막 비용 함수 내에서 부드러움에 대한 가중치입니다.

- *obstacle_cost_weight*: 장애물에 대한 가중치입니다. 0.0은 장애물을 무시하고, 1.0은 장애물에 강제 제약입니다.

- *learning_rate*: 최적화 함수가 전체 비용을 줄이면서 지역/전역 최소값(local/global minima)을 찾는 데 사용하는 학습률입니다.

- *smoothness_cost_velocity, smoothness_cost_acceleration, smoothness_cost_jerk*: 속도, 가속도, 저크(jerk)에 대한 비용과 관련된 변수들입니다.

- *ridge_factor*: Noise added to the diagonal of the total :moveit_codedir:`quadratic cost matrix<moveit_planners/chomp/chomp_motion_planner/src/chomp_cost.cpp#L62/>` in the objective function. Addition of small noise (e.g., 0.001) allows CHOMP to avoid obstacles at the cost of smoothness in trajectory.

- *use_pseudo_inverse*: Enables pseudo inverse calculations when ``true``.

- *pseudo_inverse_ridge_factor*: Set the ridge factor if pseudo inverse is enabled.

- *joint_update_limit*: Update limit for the robot joints.

- *collision_clearance*: Minimum distance from obstacles needed to avoid collision.

- *collision_threshold*: The cost threshold that that must be maintained to avoid collisions.

- *use_stochastic_descent*: Use stochastic descent while optimizing the cost when set to ``true``. In stochastic descent, a random point from the trajectory is used, rather than all the trajectory points. This is faster and guaranteed to converge, but it may take more iterations in the worst case.

- *enable_failure_recovery*: When ``true``, CHOMP will tweak certain parameters in an attempt to find a solution when one does not exist with the default parameters specified in the ``chomp_planning.yaml`` file.

- *max_recovery_attempts*: Maximum times that CHOMP is run with a varied set of parameters after the first attempt with the default parameters fails.

- *trajectory_initializaiton_method*: The type of trajectory initialization given to CHOMP, which can be ``quintic-spline``, ``linear``, ``cubic`` or ``fillTrajectory``. The first three options refer to the interpolation methods used for trajectory initialization between start and goal states. ``fillTrajectory`` provides an option of initializing the trajectory with a path computed from an existing motion planner like OMPL.

Choosing parameters for CHOMP requires some intuition that is informed by the planning environment. For instance, the default parameters for CHOMP work well in environments without obstacles; however, in environments with many obstacles the default parameters will likely cause CHOMP to get stuck in local minima. By tweaking parameters, we can improve the quality of plans generated by CHOMP.

Some of the unused/commented parameters are *hmc_stochasticity*, *hmc_annealing_factor*, *hmc_discretization*, *use_hamiltonian_montecarlo*, *animate_endeffector*, *animate_endeffector_segment*, *animate_path*, *random_jump_amount*, *add_randomness*.

CHOMP와 OMPL에서 얻어낸 계획 사이의 차이점
---------------------------------------------------
Optimizing planners optimize a cost function that may sometimes lead to surprising results: moving through a thin obstacle might be lower cost than a long, winding trajectory that avoids all collisions. In this section we make a distinction between paths obtained from CHOMP and contrast it to those obtained from OMPL.

OMPL is a open source library for sampling based / randomized motion planning algorithms. Sampling based algorithms are probabilistically complete: a solution would be eventually found if one exists, however, non-existence of a solution cannot be reported. These algorithms are efficient and usually find a solution quickly. OMPL does not contain any code related to collision checking or visualization, as the designers of OMPL did not want to tie it to a particular collision checker or visualization front end. The library is designed so it can be easily integrated into systems that provide the additional components. MoveIt integrates directly with OMPL and uses the motion planners from OMPL as its default set of planners. The planners in OMPL are abstract; i.e. OMPL has no concept of a robot. Instead, MoveIt configures OMPL and provides the back-end for OMPL to work with problems in robotics.

CHOMP: While most high-dimensional motion planners separate trajectory generation into distinct planning and optimization stages, CHOMP capitalizes on covariant gradient and functional gradient approaches to the optimization stage to design a motion planning algorithm based entirely on trajectory optimization. Given an infeasible naive trajectory, CHOMP reacts to the surrounding environment to quickly pull the trajectory out of collision while simultaneously optimizing dynamic quantities such as joint velocities and accelerations. It rapidly converges to a smooth, collision-free trajectory that can be executed efficiently on the robot. A covariant update rule ensures that CHOMP quickly converges to a locally optimal trajectory.

For scenes containing obstacles, CHOMP often generates paths which do not prefer smooth trajectories by addition of some noise (*ridge_factor*) in the cost function for the dynamic quantities of the robot (like acceleration, velocity). CHOMP is able to avoid obstacles in most cases, but it can fail if it gets stuck in local minima due to a bad initial guess for the trajectory. OMPL can be used to generate collision-free seed trajectories for CHOMP to mitigate this issue.
