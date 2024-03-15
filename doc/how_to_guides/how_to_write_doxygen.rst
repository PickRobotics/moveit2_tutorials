Doxygen 주석 기여 방법
==================================

이 가이드는 Doxygen 주석 기여를 위한 좋은 사례에 대한 소개와 개요를 제공합니다.

학습 목표
-------------------

- 유용한 Doxygen 주석 작성 방법
- 몇 가지 유용한 Doxygen 플러그인

Instructions
------------
MoveIt (또는 실제로 모든 코드)에 기여할 때 코드 전체가 읽기 쉽고 주석이 잘 달려 있어야 합니다.
Doxygen 주석을 사용하면 문서 표준화가 가능하며 모든 기여물에 특정 정보가 포함되는 것이 보장합니다.
Doxygen의 주요 장점 중 하나는 일관되고 읽기 쉬운 형식으로 API 문서를 자동으로 생성할 수 있다는 점입니다.


Doxygen 문서 생성 자동화시켜주는 플러그인들:


- `SublimeText <https://packagecontrol.io/packages/DoxyDoxygen>`_
- `VIM <https://www.vim.org/scripts/script.php?script_id=987>`_
- `VSCode <https://marketplace.visualstudio.com/items?itemName=cschlosser.doxdocgen>`_

그리고 `많은 다른 IDEs <https://www.doxygen.nl/helpers.html>`_ 에도 존재합니다.

일반적으로 Doxygen 주석에는 최소한 주석이 붙어 있는 항목에 대해서 간략한 설명이 포함되어야 합니다.
입력 파라미터(있는 경우) 및 출력 파라미터(있는 경우)에 대한 설명도 유용합니다.

아래 몇 가지 예가 제공됩니다.:


    .. code-block:: c++

        /** @brief Check for robot self collision. Any collision between any pair of links is checked for, NO collisions are
        *   ignored.
        *
        *  @param req A CollisionRequest object that encapsulates the collision request
        *  @param res A CollisionResult object that encapsulates the collision result
        *  @param state The kinematic state for which checks are being made */
        virtual void checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                        const moveit::core::RobotState& state) const = 0;



    .. code-block:: c++

        /** @brief A bounding volume hierarchy (BVH) implementation of a tesseract contact manager */
        class BulletBVHManager
        {
        ...

    .. code-block:: c++

        	/** @brief Instantiate and return a instance of a subclass of Type using our
            *         pluginlib::ClassLoader.
            * @param class_id A string identifying the class uniquely among
            *        classes of its parent class.  rviz::GridDisplay might be
            *        rviz/Grid, for example.
            * @param error_return If non-NULL and there is an error, *error_return is set to a description of the problem.
            * @return A new instance of the class identified by class_id, or NULL if there was an error.
            *
            * If makeRaw() returns NULL and error_return is not NULL, *error_return will be set.
            * On success, *error_return will not be changed. */
         	virtual Type* makeRaw(const QString& class_id, QString* error_return = nullptr) {

이 예제들은 함수 또는 클래스가 수행하는 작업을 간략하게 요약하고 입력 및 출력의 타입과 설명을 제공합니다.


추가 읽을꺼리
---------------

Doxygen 주석의 추가 예제를 보려면 저장소를 자유롭게 둘러보세요.
기여할 코드와 비슷한 부분을 보고 주석을 살펴보는 것이 배우기 가장 쉬운 방법입니다.

Doxygen API를 로컬에서 생성하는 방법에 대한 가이드는 :doc:`여기 <./how_to_generate_api_doxygen_locally>` 를 참고하세요.

Doxygen 문서 작성 가이드는 `여기 <https://www.doxygen.nl/manual/docblocks.html>`_ 에서 확인할 수 있습니다.
