MoveIt 하우투 가이드 작성 방법
==================================

이 가이드에서는 MoveIt 문서 작성을 위한 방법 설명을 다루고 있습니다. 주된 목적은 PickNik Robotics 직원들이 사용 방법 문서를 표준화하는 것을 돕는 것이지만, 새로운 가이드를 제출하고 싶은 모든 기여자도 사용할 수 있습니다.
MoveIt 사용 방법 가이드를 찾고 있다면 :doc:`/doc/how_to_guides/how_to_guides` 에서 확인하세요.

학습 목표
-------------------
- 사용 방법 가이드에 포함되어야 하는 정보 종류
- MoveIt 문서 섹션에서의 적절한 포맷팅

요구사항
------------
- Ubuntu 20.04
- ROS 2 Galactic
- MoveIt 2

단계
-----

1. `MoveIt 2 Tutorials repository <https://github.com/ros-planning/moveit2_tutorials.git>`_ 를 fork하고 직관적인 이름의 새로운 브랜치를 생성합니다 (예제: ``jack/how-to-write-how-tos`` ).

#. .rst 확장자로 ``doc/how_to_guides`` 디렉토리에 새 파일을 만듭니다. 제목은 "How to" 다음에 해답하는 질문을 명확하게 명시합니다 (예제: "MoveIt 사용 방법 가이드 작성 방법").

#. 적절한 하우투 가이드 페이지에 여러분의 가이드에 대한 링크를 추가합니다.:

   - :doc:`사용자 가이드 </doc/how_to_guides/how_to_guides>`

   - :doc:`기여자 가이드 </doc/how_to_contribute/how_to_contribute>`

#. 다음과 같은 지침을 사용하여 reStructuredText (.rst)로 소개 부분을 작성합니다.:

   - 제목은 파일 이름과 동일해야 합니다.

   - 소개 부분에서는 이 사용 방법 가이드의 목적과 대상 독자를 설명해야 합니다.

   - 실수로 이 가이드를 찾는 사람이 많을 것 같으면 적절한 리소스 링크를 추가합니다.

#. 독자가 읽고 나면 알게 되는 내용 (즉, 학습 목표)을 구체적으로 명시합니다.

#. 사용자가 이 가이드가 자신에게 적합한지 알 수 있도록 시스템 또는 장비 요구 사항을 추가합니다.

#. 개별 작업 단계를 명확하게 설명하고 필요한 중간 단계를 생략하지 않습니다.

#. 추가적인 정보를 제공하는 링크가 있는 "추가 읽을꺼리" 섹션을 만듭니다.

#. 새로운 페이지를 `MoveIt 2 Tutorials repository <https://github.com/ros-planning/moveit2_tutorials.git>`_ 에 PR로 제출합니다.

Template
--------

.. code-block::

  <Title>
  -------

  <Brief description of the How-To Guide with image or GIF showing the outcome.>

  Learning Objectives
  -------------------

  <List of things the user will learn.>

  Requirements
  ----------

  <Explanation of what the user should understand. Unlike a tutorial, these guides stand alone and can assume the user has much more background.>

  Steps
  -----

  <A list of steps to take to solve the problem.>

  Further Reading
  ---------------

  <A list of links to related content on and off this website.>

Further Reading
---------------
- :doc:`/doc/how_to_contribute/how_to_contribute_to_site`
- :doc:`MoveIt Concepts: How-To Guide </doc/how_to_guides/how_to_guide>`
