API Doxygen 참조 문서를 로컬에서 생성하는 방법
================================================


moveit2 저장소의 root에서 doxygen 실행

요구사항
------------

-  ``doxygen`` 와 ``graphviz`` 설치:

.. code-block:: bash

    sudo apt-get install doxygen graphviz

단계
-----

- moveit2 저장소로 이동:

.. code-block:: bash

  cd ~/ws_moveit/src/moveit2

- 원하는 출력 디렉토리 경로와 함께 레퍼런스 생성 명령어를 실행하세요.:

.. code-block:: bash

  DOXYGEN_OUTPUT_DIRECTORY=~/docs doxygen

- 브라우저에서 문서의 진입점은 index.html이며 다음과 같이 접근할 수 있습니다:

.. code-block:: bash

  firefox ~/docs/index.html
