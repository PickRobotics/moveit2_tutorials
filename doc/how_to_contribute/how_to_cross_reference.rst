How to Cross-Reference Content
==============================

여기서는 이 웹사이트와 API 문서 간에 성공적으로 링크를 연결하는 방법에 대한 기본 지침입니다.I.

콘텐츠를 참조하는 방법은 다양하고 때로는 너무 많아서 새로운 기여자들이 어떤 방법을 사용해야 할지 혼란스러울 수도 있습니다.
일부 방법은 로컬에서는 작동하지만 배포된 웹사이트에서는 링크 기능을 생성하지 못할 수도 있습니다.
따라서 이 웹사이트의 콘텐츠를 크로스-레퍼런싱할 때는 제안된 Sphinx 역할만을 사용하도록 기여자 여러분께 요청합니다.

학습 목표
-------------------
* Sphinx의 :ref: 및 :doc: 역할을 사용하여 문서 및 섹션 링크
* autosectionlabel 확장자에서 생성된 ``:ref:`` 사용
* *doxylink* 확장자의 ``:cpp_api:`` 역할을 사용하여 C++ API 참조

다른 문서나 섹션에 링크걸기
---------------------------------------

Sphinx는 콘텐츠 크로스-레퍼런싱을 위한 ``:doc:`` 와 ``:ref:`` 역할을 제공하며(`cross-referencing content <https://docs.readthedocs.io/en/stable/guides/cross-referencing-with-sphinx.html#cross-referencing-using-roles>`_ ), 다른 Sphinx 확장 프로그램과의 호환성 및 다중 배포판 지원을 보장하기 위해 이 역할을 사용하는 것이 좋습니다.

다른 문서에 링크하려면 다음과 같이 ``:doc:`` 역할을 사용할 수 있습니다: :doc:`/doc/tutorials/getting_started/getting_started` (``:doc:`/doc/tutorials/getting_started/getting_started```). ``:ref:`` 역할은 페이지의 명확한 대상에 링크하는 id를 허용합니다. 편의를 위해 모든 섹션에 대해 고유하고 사람이 읽을 수 있는 참조 대상을 만드는 Sphinx 확장 프로그램 `autosectionlabel <https://www.sphinx-doc.org/en/master/usage/extensions/autosectionlabel.html>`_ 을 활성화했습니다. 모든 문서의 섹션은 문서 경로와 섹션 제목을 제공하여 링크할 수 있습니다: :ref:`like this <doc/tutorials/getting_started/getting_started:Install ROS 2 and Colcon>` (``:ref:`like this <doc/tutorials/getting_started/getting_started:Install ROS 2 and Colcon>```).
``:doc:`` 역할은 절대 경로를 요구하며 ``/`` 로 시작해야 하지만, *autosectionlabel* 확장자는 이를 포함하지 않고 ``:ref`` 경로 레이블을 만듭니다.

API 문서 참조
---------------------------------

API 페이지는 Doxygen을 사용하여 생성되며 Sphinx를 사용하지 않으므로 ``:doc:`` 및 ``:ref:`` 역할은 API 페이지를 찾을 수 없습니다.
우리는 `doxylink <https://sphinxcontrib-doxylink.readthedocs.io/en/stable/>`_ 와 커스텀 ``:cpp_api:`` 역할을 사용하여 심볼로부터 API 페이지에 대한 링크를 생성합니다.

다음은 몇 가지 예제이며, 일부 링크는 제목을 사용하고 일부는 사용하지 않습니다.:

- namespaces: ``:cpp_api:`moveit::core``` -> :cpp_api:`moveit::core`
- classes:
  ``:cpp_api:`moveit::core::RobotModel``` -> :cpp_api:`moveit::core::RobotModel`
- functions and members:

  - ``:cpp_api:`RobotModel::getName() <moveit::core::RobotModel::getName>``` -> :cpp_api:`RobotModel::getName() <moveit::core::RobotModel::getName>`
  - ``:cpp_api:`moveit::core::RobotModel::enforcePositionBounds(double *state) const``` -> :cpp_api:`moveit::core::RobotModel::enforcePositionBounds(double *state) const`
  - ``:cpp_api:`RobotModel::root_link_ <moveit::core::RobotModel::root_link_>``` -> :cpp_api:`RobotModel::root_link_ <moveit::core::RobotModel::root_link_>`
- files:
  ``:cpp_api:`robot_model.cpp``` -> :cpp_api:`robot_model.cpp`

특정 심볼 연결 방법에 확실하지 않은 경우, ``MoveIt.tag`` 파일 내에서 모든 Doxygen 참조를 찾을 수 있습니다.
이 파일은 빌드 유형에 따라  ``build/html/api/`` 혹은 ``build/html/<branch>/api/`` 디렉토리 안에 위치합니다.

추천과 비추천
---------------

이렇게 **하세요** :

- 가능한 한 교차 참조를 많이 사용하십시오. 특히 코드에 대한 참조가 중요합니다.
- 링크 제목을 명확하게 하거나 API 심볼을 줄여 읽기 쉽게 만드십시오.

이렇게는 **하지 마세요** :

- 튜토리얼이나 API 참조 시 원래 URL을 사용합니다.
- GitHub 소스 파일에 링크를 연결할 때는 Doxygen 페이지를 우선적으로 사용하십시오.

추가 읽을꺼리
---------------

- :doc:`how_to_contribute_to_site`
- :doc:`how_to_write_tutorials`
- :doc:`how_to_write_how_to_guides`
