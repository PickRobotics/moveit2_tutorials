MoveIt 튜터리얼 작성 방법
==============================

이 가이드는 MoveIt 문서에서 튜토리얼 작성 방법을 설명합니다.
튜토리얼은 많은 새로운 사용자가 처음 접하는 부분이기 때문에 가장 유용한 기여 방식 중 하나입니다.
이 가이드는 새로운 튜토리얼을 제출하고자 하는 모든 기여자를 위한 것입니다.
이 저장소의 `README <https://github.com/ros-planning/moveit2_tutorials/blob/main/README.md>`_ 튜토리얼 기여에 대한 추가적인 품질 기준과 방법 설명이 많이 포함되어 있습니다.

학습 목표
-------------------
- 튜터리얼에 포함되어야 정보 유형
- MoveIt 문서 섹션의 튜토리얼 올바른 형식

요구사항
------------
- Ubuntu 20.04
- ROS 2 Galactic
- MoveIt 2

단계
-----

1. `MoveIt 2 튜터리얼 저장소 <https://github.com/ros-planning/moveit2_tutorials.git>`_ 를 fork하고 직관적인 이름의 새로운 브랜치를 생성합니다. (예: ``jack/how-to-tutorials`` ).

#. ``doc/tutorials`` 디렉토리에 ``.rst`` 확장자를 가진 새로운 파일을 만듭니다. 제목은 간결한 설명이어야 합니다. (예: "RViz를 이용한 MoveIt  시작")

#. :doc:`tutorials page </doc/tutorials/tutorials>` 에 여러분의 튜토리얼 링크를 추가합니다.

#. reStructuredText (.rst) 를 사용하여 다음과 같은 지침을 따르며 소개 부분을 작성합니다:

   - 소개 부분은 이 튜토리얼의 목적과 대상 독자를 설명해야 합니다.

   - 사용자가 실수로 이 가이드를 찾을 수 있다고 생각되면 적절한 리소스 링크를 추가합니다.

#. 독자가 튜토리얼을 읽고 난 후 무엇을 알게 될지, 구체적인 학습 목표를 작성합니다.

#. 사용자가 이 튜토리얼이 자신에게 적합한지 알 수 있도록 시스템 또는 장비 요구 사항을 추가합니다.

#. 독자가 쉽게 따라갈 수 있도록 충분한 세부 사항과 함께 수행해야 하는 대화형 단계를 구성합니다.

#. 튜토리얼을 요약하고 추가 리소스를 제공하는 결론을 작성합니다.

#. 독자가 따라야 하는 다음 튜토리얼 링크를 추가합니다.

#. 새로운 페이지를 PR로 `MoveIt 2 Tutorials repository <https://github.com/ros-planning/moveit2_tutorials.git>`_ 에 제출합니다.

Template
--------

.. code-block::

  <Title>
  -------

  <Brief description of the tutorial with image showing what will be accomplished.>

  Background
  ----------

  <Explanation of what the user should have already done before this tutorial.>

  Steps
  -----

  1. <First Step>
  ---------------

  <This should describe an action that the user should take such as creating a ROS project or typing up some code.>

  1.1) <Explanation First Step>
  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  <Use sub-steps like this to walk the user through an explanation of what they did.>

  1.2) <Action First step>
  ^^^^^^^^^^^^^^^^^^^^^^^

  <Use a sub-step like this to describe running the new code and what the results should be.>

  2. <Second Step>
  ----------------

  <...>

  Conclusion
  ----------

  <Here is where you explain what the user has read and provide additional references.>

  Next Step
  ---------

  <Link to the next tutorial here.>

Further Reading
---------------

- :doc:`how_to_contribute_to_site`
- :doc:`how_to_cross_reference`
