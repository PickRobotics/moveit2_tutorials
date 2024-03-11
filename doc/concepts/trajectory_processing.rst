===================================
궤적 프로세싱(Trajectory Processing)
===================================

Time parameterization
---------------------

모션 플래너는 일반적으로 “paths(경로)”만 생성하며, 즉 경로와 관련된 타이밍 정보가 없습니다.
MoveIt에는 이러한 경로에서 작동하고 개별 joint에 부과된 최대 속도 및 가속도 제한을 고려하여 적절하게 시간 매개 변수화된(time-parameterized) 궤적을 생성할 수 있는 몇 가지 :cpp_api:`trajectory processing <trajectory_processing>` 알고리즘이 포함되어 있습니다.
이러한 제약은 각 로봇에 대해 지정된 특수 ``joint_limits.yaml`` 설정 파일에서 읽어옵니다.
구성 파일은 선택 사항이며 URDF의 모든 속도 또는 가속도 제한을 재정의합니다.
2023년 1월 현재 권장되는 알고리즘은 :cpp_api:`TimeOptimalTrajectoryGeneration <trajectory_processing::TimeOptimalTrajectoryGeneration>` (TOTG)입니다.
이 알고리즘에 대한 주의 사항은 로봇이 정지 상태에서 시작하고 끝나야 한다는 것입니다.
기본적으로 TOTG timestep은 0.1초입니다.
