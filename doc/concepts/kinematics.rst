==========
운동학(Kinematics)
==========

The Kinematics Plugin
---------------------

MoveIt은 plugin 구조를 사용한다. 특히 사용자가 자신의 역운동학(inverse kinematics) 알고리즘을 작성할 수 있도록 하기 위해 만들어졌다. 전방 운동학(forward kinematics)과 자코비안을 찾는 것은 RobotState 클래스 자체에 통합되어 있다.
MoveIt의 기본 역운동학 플러그인은 `KDL <https://github.com/orocos/orocos_kinematics_dynamics>`_ 수치 자코비안-기반 솔버를 사용하여 설정한다.
이 플러그인은 MoveIt Setup Assistant에서 자동으로 설정한다.

******************
충돌 검사(Collision Checking)
******************

MoveIt에서 충돌 검사는 ``CollisionWorld`` 객체를 사용하여 Planning Scene 내에서 설정된다.
운이 좋게도 MoveIt은 사용자가 충돌 검사가 어떻게 일어나는지에 대해 걱정할 필요가 없도록 설정되어 있다.
MoveIt에서의 충돌 검사는 주로 MoveIt의 주요 충돌 검사 라이브러리인 `FCL <https://flexible-collision-library.github.io/>`_ 패키지를 사용하여 수행된다.

Collision Objects
-----------------

MoveIt은 여러 유형 객체의 충돌 검사를 지원한다.:

- **Meshes** - robot 링크와 같은 객체를 서술하는데 ``.stl`` (standard triangle language) 혹은 ``.dae`` (digital asset exchange) 포맷을 사용 가능하다.

- **Primitive Shapes** - e.g. 박스, 실린더, 콘, 구, 평면(boxes, cylinders, cones, spheres and planes)

- **Octomap** - ``Octomap`` 객체를 충돌 검사에 직접 사용 가능하다.

Allowed Collision Matrix (ACM)
------------------------------

충돌 검사는 운동 계획 중에 가장 비싼 연산 중 하나이다. 충돌 검사는 로봇이나 월드에 있는 물체들 사이의 충돌을 확인하는데 사용된다.
``Allowed Collision Matrix`` 혹은 ``ACM``은 두 물체 사이의 충돌 검사가 필요한지를 나타내는 이진값을 인코딩한다. (로봇이나 world에 있는 물체들)
만약 두 물체에 대한 값이 ACM에서 ``true``로 설정되어 있다면, 두 물체 사이의 충돌 검사가 필요하지 않거나 원하지 않는다는 것을 나타낸다.
충돌 검사가 필요하지 않을 수도 있고, 두 물체가 항상 멀리 떨어져 있어서 항상 충돌하지 않을 수도 있다. 또는 두 물체가 기본적으로 서로 접촉하고 있을 수도 있다. 이 경우 ACM에서 두 물체에 대한 충돌 검사를 비활성화해야 한다.
