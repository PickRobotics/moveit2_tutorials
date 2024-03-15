planning pipeline 벤치마킹 방법
=======================================

시작하기
---------------
:doc:`Getting Started </doc/tutorials/getting_started/getting_started>` 을 먼저 완료하세요.

:moveit_codedir:`benchmarking 패키지 <moveit_ros/benchmarks>` 는 OMPL Planner Arena를 사용하여 MoveIt 계획 파이프라인을 벤치마킹하고 통계를 집계/플롯하는 방법을 제공합니다.
아래 예시는 벤치마킹을 실행하는 방법을 보여줍니다.

예제
-------

예시를 실행하려면 ``git lfs install`` 명령어를 실행하여 git lfs를 설치하고 [moveit_benchmark_resources](https://github.com/ros-planning/moveit_benchmark_resources.git) 를 워크스페이스에 복제해야 합니다.

다음을 실행하여 벤치마크를 시작합니다.: ::

    ros2 launch moveit2_tutorials run_benchmarks.launch.py


이 작업은 ``benchmarks.yaml`` 의 설정에 따라 시간이 걸릴 수 있습니다. 벤치마킹 결과는 ``/tmp/moveit_benchmarks/`` 디렉토리에 저장됩니다.
벤치마킹 데이터를 검토하려면 로그 파일을 데이터베이스로 변환해야 합니다. 이 작업은 moveit_ros_benchmarks 패키지에서 제공하는 스크립트를 사용하여 수행할 수 있습니다.: ::

    ros2 run moveit_ros_benchmarks moveit_benchmark_statistics.py LOG_FILE_1 ... LOG_FILE_N

이 명령은 포함된 모든 벤치마킹 로그 파일의 데이터를 포함하는 데이터베이스를 생성합니다. 더 쉬운 방법은 지정된 저장소의 모든 로그 파일을 가지고 데이터베이스를 만드는 것입니다.
예를 들어, 인수 ``/tmp/moveit_benchmarks/*`` 를 사용하여 지정된 디렉토리의 모든 로그 파일을 단일 데이터베이스로 수집할 수 있습니다. 이 데이터베이스는 명령이 실행된 위치에 ``benchmark.db`` 라는 이름으로 생성됩니다.
데이터베이스는 파일을 `plannerarena.org <http://plannerarena.org>`_ 에 업로드하여 시각화하며 결과를 대화형으로 시각화할 수 있습니다.


.. image:: planners_benchmark.png
   :width: 700px

ROS 2 parameters로 벤치마크 설정하기
-----------------------------------------

벤치마킹은 ROS 2 파라미터 세트로 설정합니다. 이러한 매개변수에 대해서는 :moveit_codedir:`BenchmarkOptions.h <moveit_ros/benchmarks/include/moveit/benchmarks/BenchmarkOptions.h>` 파일에서 더 자세히 알아볼 수 있습니다.


The BenchmarkExecutor Class
---------------------------

이 클래스는 제공된 ``BenchmarkOptions`` 인스턴스에 지정된 매개변수을 보고 일련의 ``MotionPlanRequests`` 를 생성한 다음 지정된 각 플래너에 대해 요청을 실행합니다.  ``BenchmarkOptions`` 에서 쿼리, ``goal_constraints`` 및 ``trajectory_constraints`` 는 별도의 쿼리로 취급됩니다.  ``start_states`` 세트가 지정되면 각 쿼리, ``goal_constraint`` 및 ``trajectory_constraint`` 가 각 시작 상태(쿼리의 기존 시작 상태는 무시됨)와 함께 시도됩니다. 마찬가지로, (선택적) 경로 제약 세트는 시작 쿼리 및 시작 ``goal_constraint`` 쌍과 함께 조합적으로 결합됩니다(쿼리의 기존 ``path_constraint`` 는 무시됨). 워크스페이스가 지정되면 기존 워크스페이스 파라미터를 무시합니다.

벤치마킹 파이프라인은 ``MoveGroup`` 을 사용하지 않습니다.
대신, 모든 지정된 ``PlanningRequestAdapters`` 를 포함하여 플래닝 파이프라인을 직접 초기화하고 실행합니다.
이는 특히 smoothing 어댑터의 효과를 벤치마킹하는 데 유용합니다.

``BenchmarkExecutor`` 클래스를 상속하고 하나 이상의 가상 함수를 재정의하면 벤치마크 실행을 커스터마이징할 수 있습니다.
예를 들어, ``initializeBenchmarks()`` 혹은 ``loadBenchmarkQueryData()`` 함수를 재정의하면 ROS warehouse를 사용하지 않고 벤치마크 쿼리를 직접 지정하고 커스텀 planning scene을 제공할 수 있습니다.

추가로, 상속받은 클래스에서 쉽게 커스텀할 수 있도록 다음과 같은 함수 세트가 있습니다.:

- ``preRunEvent``: solve를 호출하기 직전에 호출됩니다.
- ``postRunEvent``: solve를 호출한 직후에 호출됩니다.
- ``plannerSwitchEvent``: 벤치마킹 중 플래너가 변경될 때 호출됩니다.
- ``querySwitchEvent``: 새로운 벤치마크 문제 실행이 시작되기 전에 호출됩니다.

위의 내용에서 벤치마크는 ``PlanningScene`` , 시작 상태, 목표 제약 조건 / ``trajectory_constraints``, (옵션) ``path_constraints`` 의 구체적인 인스턴스입니다. 실행은 특정 플래너가 벤치마크를 해결하기 위해서 한 번 시도합니다.
