여러 Planning Pipe을 MoveItCpp로 병렬로 실행하기
==============================================================

MoveItCpp는 다음과 같은 기능을 허용하는 API를 제공합니다:

1. 서로 다른 planners를 사용하여 병렬로 여러 계획 파이프라인 실행
2. 솔루션을 찾지 못한 파이프라인을 종료하기 위한 커스텀 종료 기준을 정의하기
3. 가장 적합한 솔루션을 선택하는 커스텀 함수를 정의하기

여러 파이프라인을 사용하는 것은 다음과 같은 몇 가지 이유로 유용할 수 있습니다. :

- 최적의 솔루션을 생성하는 planner는 사전에 알 수 없습니다.
- 선호하는 planner 실패할 가능성이 있고 백업 솔루션을 사용할 수 있어야 합니다.

MoveItCpp에 대한 일반적인 소개는 :doc:`/doc/examples/moveit_cpp/moveitcpp_tutorial` 에서 찾을 수 있습니다.

Parallel Planning Interface
---------------------------

MoveItCpp를 이용한 병렬 계획은 단일 파이프라인 계획과 비슷하지만, planning 컴포넌트의 :code:`plan(...)` 함수의 구현이 다릅니다:

.. code-block:: c++

    MotionPlanResponse PlanningComponent::plan(
      const MultiPipelinePlanRequestParameters& parameters,
      SolutionCallbackFunction solution_selection_callback,
      StoppingCriterionFunction stopping_criterion_callback)

이 함수는 시작 상태에서 목표 상태까지 제약 조건을 만족하는 궤적을 계획하려고 시도합니다.
:code:`parameters` 에 의해 제공되는 설정을 기반으로 여러 스레드가 실행되고, 각 스레드는 서로 다른 계획 파이프라인을 사용하여 계획 문제를 해결하려고 합니다. 해결책이 없을 수도 있다는 점에 유의하십시오.
일단 모든 파이프라인이 종료되면 :code:`solution_selection_callback` 함수가 호출되어 어떤 해결책을 :code:`MotionPlanResponse` 로 반환할지 결정합니다.
기본적으로 모든 파이프라인은 :code:`MultiPipelinePlanRequestParameters` 의 :code:`planning_time` 필드에 정의된 시간까지 사용하지만, :code:`stopping_criterion_callback` 을 사용하여 병렬 계획을 일찍 종료할 수도 있습니다.
이 함수는 병렬 계획 과정에서 파이프라인이 하나의 해결책을 생성할 때마다 호출되며, 종료 기준을 충족하면 아직 해결책을 찾지 못한 파이프라인을 종료합니다.

예제
-------

다음 데모는 MoveItCpp의 병렬 계획 인터페이스를 설정 및 사용하는 방법을 보여줍니다.
먼저, 데모 실행 방법은 다음과 같습니다.: ::

  ros2 launch moveit2_tutorials parallel_planning_example.launch.py

데모를 실행하면 복잡한 주방 scene이 로드되고 두 가지 planning 문제가 해결됩니다. 첫 번째 문제는 end-effector를 바닥쪽으로 작게 움직이는 것입니다. 이 문제는 세 가지 계획 모두 해결할 수 있지만, 계획 시간에 상당한 차이가 있습니다. 두 번째 문제는 훨씬 어려우며 :code:`RRTConnect` 만 성공할 가능성이 높습니다. 이 데모는 병렬 계획 설정을 잘 구성한다는 것은 다양하며 광범위한 모션 계획 문제에 사용될 수 있음을 시사합니다.

병렬 계획을 사용하는 데 어떤 코드가 필요한가?
------------------------------------------------

먼저, 계획 문제를 해결할 :code:`moveit_cpp` 및 계획 컴포넌트를 초기화해야 합니다. 다음으로 시작 상태 및 목표 제약 조건을 설정해야 합니다.:

.. code-block:: c++

    planning_component_->setGoal(*goal_state);
    planning_component_->setStartStateToCurrentState();

추가로 :code:`MultiPipelinePlanRequestParameters` 설정이 필요합니다.

.. code-block:: c++

    moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters multi_pipeline_plan_request{
      node_, { "ompl_rrtc", "pilz_lin", "chomp" }
    };

이 클래스의 생성자는 node의 파라미터 네임스페이스인 :code:`"ompl_rrtc"`, :code:`"pilz_lin"`, :code:`"chomp"` 에서 제공하는 설정에 따라 여러 :code:`PlanningRequestParameter` 클래스 멤버를 초기화합니다.
이를 제공하기 위해 :code:`moveit_cpp.yaml` 파일을 확장할 수 있습니다.:

.. code-block:: yaml

    # PlanRequestParameters for the first parallel pipeline that uses OMPL - RRTConnect
    ompl_rrtc:
      plan_request_params:
        planning_attempts: 1
        planning_pipeline: ompl
        planner_id: "RRTConnectkConfigDefault"
        max_velocity_scaling_factor: 1.0
        max_acceleration_scaling_factor: 1.0
        planning_time: 0.5

    # PlanRequestParameters for a second parallel pipeline using Pilz with the LIN planner
    pilz_lin:
      plan_request_params:
        planning_attempts: 1
        planning_pipeline: pilz_industrial_motion_planner
        planner_id: "LIN"
        max_velocity_scaling_factor: 1.0
        max_acceleration_scaling_factor: 1.0
        planning_time: 0.8

    # PlanRequestParameters for a third parallel pipeline that uses CHOMP as planner
    chomp:
      plan_request_params:
        planning_attempts: 1
        planning_pipeline: chomp
        max_velocity_scaling_factor: 1.0
        max_acceleration_scaling_factor: 1.0
        planning_time: 1.5

  # Another OMPL planner using a second OMPL pipeline named 'ompl_rrt_star'
  ompl_rrt_star:
    plan_request_params:
      planning_attempts: 1
      planning_pipeline: ompl_rrt_star # Different OMPL pipeline name!
      planner_id: "PRMkConfigDefault"
      max_velocity_scaling_factor: 1.0
      max_acceleration_scaling_factor: 1.0
      planning_time: 1.5

옵션으로 커스텀 종료 기준 및/또는 솔루션 선택 함수를 정의하는 것도 가능합니다.
:code:`plan(...)` 함수에 인자로 아무것도 전달하지 않으면, 모든 파이프라인은 전체 계획 시간 예산을 사용하고, 그 후에 가장 짧은 경로를 선택합니다.

이 예제에서는 기본 종료 기준과 가장 짧은 솔루션을 선택하는 솔루션 선택 기준을 사용합니다.:

.. code-block:: c++

    planning_interface::MotionPlanResponse getShortestSolution(const std::vector<planning_interface::MotionPlanResponse>& solutions)
    {
      // Find trajectory with minimal path
      auto const shortest_solution = std::min_element(solutions.begin(), solutions.end(),
        [](const planning_interface::MotionPlanResponse& solution_a,
           const planning_interface::MotionPlanResponse& solution_b) {
          // If both solutions were successful, check which path is shorter
          if (solution_a && solution_b)
          {
            return robot_trajectory::pathLength(*solution_a.trajectory_) <
                   robot_trajectory::pathLength(*solution_b.trajectory_);
          }
          // If only solution a is successful, return a
          else if (solution_a)
          {
            return true;
          }
          // Else return solution b, either because it is successful or not
          return false;
        });
      return *shortest_solution;
    }

여기서는 커스텀 종료 기준의 예제를 보여줍니다. 이 기준은 :code:`RRTConnect` 가 솔루션을 찾으면 즉시 다른 계획 파이프라인을 종료합니다.:

.. code-block:: c++

    // Stop parallel planning as soon as RRTConnect finds a solution
    bool stoppingCriterion(
        moveit_cpp::PlanningComponent::PlanSolutions const& plan_solutions,
        moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters const& plan_request_parameters)
    {
      // Read solutions that are found up to this point from a thread safe storage
      auto const& solutions = plan_solutions.getSolutions();

      // Stop parallel planning if the pipeline using RRTConnect finds a solution
      for (auto const& solution : solutions)
      {
          if (solution.planner_id_ == "RRTConnectkConfigDefault")
          {
            // Return true to abort the other pipelines
            return true;
          }
      }
      // Return false when parallel planning should continue
      return false;
    }

:code:`MultiPipelinePlanRequestParameters` 와 선택적으로 :code:`SolutionCallbackFunction` 및/또는 :code:`StoppingCriterionFunction` 이 정의되면 :code:`plan(...)` 함수를 호출합니다.:

.. code-block:: c++

    auto plan_solution = planning_component_->plan(multi_pipeline_plan_request, &getShortestSolution);

Tips
----

- 같은 파이프라인의 서로 다른 플래너 (예: PTP 와 LIN을 갖는 Pilz 플래너)를 병렬로 사용하려는 경우, 여러 개의 계획 파이프라인을 MoveItCpp에서 초기화하는 것이 여러 병렬 계획 요청에서 동일한 파이프라인을 사용하는 것보다 더 효율적입니다. 이 예제에서는 두 개의 OMPL 파이프라인이 로드됩니다.
