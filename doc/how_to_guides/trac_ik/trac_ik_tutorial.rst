TRAC-IK Kinematics Solver
=========================

`TRAC-IK <https://bitbucket.org/traclabs/trac_ik>`_ 은 TRACLabs 에서 개발된 inverse kinematics solver이며, 두 개의 IK 구현 방법을 스레딩을 통해 결합하여 기존의 오픈소스 IK 솔버보다 더 신뢰할 수 있는 솔루션을 제공합니다.
문서 내용 요약:

  TRAC-IK는 KDL 라이브러리의 일반적인 역 야코비안 방법에 대한 대안을 제공합니다.
  KDL의 해당 알고리즘은 뉴턴 방식을 기반으로 하며, 많은 로봇 플랫폼에서 일반적인 조인트 제한 상황에서 잘 작동하지 않습니다.
  TRAC-IK는 동시에 두 개의 IK 구현 방법을 실행합니다.
  첫 번째 방법은 KDL의 뉴턴 방식 기반 해당 알고리즘을 확장하여 랜덤 점프를 통해 관절 제한으로 인한 국소 최소값 문제를 감지하고 완화하는 방법입니다.
  두 번째 방법은 비선형 최적화 접근법인 SQP(순차 이차 QP)를 사용하며, 준-뉴턴 방법을 통해 관절 제한을 더 잘 처리합니다.
  기본적으로 어느 알고리즘이든 답을 찾으면 즉시 IK 검색을 종료합니다.
  또한 최적의 IK 솔루션을 얻기 위해 거리 및 조작성과 같은 보조 제약 조건도 제공됩니다.
  (TRAC-IK) provides an alternative Inverse Kinematics solver to the popular inverse Jacobian methods in KDL.
  Specifically, KDL's convergence algorithms are based on Newton's method, which does not work well in the presence of joint limits --- common for many robotic platforms.
  TRAC-IK concurrently runs two IK implementations.
  One is a simple extension to KDL's Newton-based convergence algorithm that detects and mitigates local minima due to joint limits by random jumps.
  The second is an SQP (Sequential Quadratic Programming) nonlinear optimization approach which uses quasi-Newton methods that better handle joint limits.
  By default, the IK search returns immediately when either of these algorithms converges to an answer.
  Secondary constraints of distance and manipulability are also provided in order to receive back the "best" IK solution.

`trac_ik_kinematics_plugin <https://bitbucket.org/traclabs/trac_ik/src/rolling-devel/trac_ik_kinematics_plugin/>`_ 패키지는 MoveIt의 ``KinematicsBase`` 인터페이스를 제공하며 이를 통해 기본 KDL 솔버를 교체할 수 있습니다.
현재는 모방한 관절을 지원하지 않습니다.
현재 미믹 관절을 지원하지 않습니다.

설치
-------

The ``rolling-devel`` branch of the TRAC-IK repository has the latest ROS 2 implementation.
For now, the repository must be built from source in your ROS 2 workspace, for example ``~/moveit2_ws``. ::

  cd ~/moveit2_ws/src
  git clone -b rolling-devel https://bitbucket.org/traclabs/trac_ik.git

Usage
-----

- Find the MoveIt :doc:`kinematics.yaml </doc/examples/kinematics_configuration/kinematics_configuration_tutorial>` file created for your robot.
- Replace ``kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin`` (or similar) with ``kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin``
- Make sure you add a ``<depend>trac_ik_kinematics_plugin</depend>`` tag to your package's corresponding ``package.xml`` file.
- Set parameters as desired:

  - **kinematics\_solver\_timeout** (timeout in seconds, e.g., ``0.005``) and **position\_only\_ik** **ARE** supported.
  - **solve\_type** can be ``Speed``, ``Distance``, ``Manip1``, ``Manip2`` (see below for details). Defaults to ``Speed``.
  - **epsilon** is the Cartesian error distance used to determine a valid solution. Defaults to ``1e-5``.
  - **kinematics\_solver\_attempts** parameter is unneeded: unlike KDL, TRAC-IK solver already restarts when it gets stuck.
  - **kinematics\_solver\_search\_resolution** is not applicable here.

From the `trac_ik_lib <https://bitbucket.org/traclabs/trac_ik/src/rolling-devel/trac_ik_lib/>`_ package documentation (slightly modified), here is some information about the solve type parameter:

  - ``Speed``: returns very quickly the first solution found.
  - ``Distance``: runs for the full timeout, then returns the solution that minimizes sum of squares error (SSE) from the seed.
  - ``Manip1``: runs for full timeout, returns solution that maximizes ``sqrt(det(J*J^T))`` (the product of the singular values of the Jacobian).
  - ``Manip2``: runs for full timeout, returns solution that minimizes the ratio of min to max singular values of the Jacobian.
