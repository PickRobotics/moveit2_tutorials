# MoveIt 2 튜터리얼

MoveIt 프로젝트에 대한 문서입니다.

## Build Status

이 repository는 현재 Github Actions에 의해 자동으로 빌드됩니다:

- main: [![CI](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/ci.yaml/badge.svg?branch=main)](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/ci.yaml?query=branch%3Amain)
- main: [![Format](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/format.yml/badge.svg?branch=main)](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/format.yml?query=branch%3Amain)
- humble: [![CI](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/ci.yaml/badge.svg?branch=humble)](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/ci.yaml?query=branch%3Ahumble)
- humble: [![Format](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/format.yml/badge.svg?branch=humble)](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/format.yml?query=branch%3Ahumble)

## 기여하기

여러분이 참여하여 MoveIt의 문서를 개선하는 것을 권장합니다. 이 튜토리얼을 개선하는 데 도움을 주시기 바랍니다. ROS 1의 이전 튜토리얼을 포팅하고 새로운 튜토리얼을 작성하는 방식으로 이 튜터리얼을 개선하는데 도움을 주세요. 아래의 품질 기준을 읽어보시고 [MoveIt 튜터리얼 작성하기](https://moveit.picknik.ai/main/doc/how_to_contribute/how_to_write_tutorials.html) 페이지를 참조하십시오.

이 튜터리얼에서 이슈를 발견했는데 여러분이 수정하기 어렵다면 [GitHub에 이슈를 오픈]((https://github.com/ros-planning/moveit2_tutorials/issues/new) 혹은 PR을 제안해 주세요.

## ROS 2로 튜터리얼 포팅 도움주기

각 튜터리얼을 ROS 2로 포팅하는데 이슈가 발생합니다. 각 튜터리얼의 맨 위에 ":moveit1:" 태그를 제거하면 튜터리얼이 성공적으로 업데이트된 것입니다.

아래에 포팅하는데 도움이 되는 링크가 있습니다.

* [colcon](https://colcon.readthedocs.io/en/released/user/how-to.html)
* [ament](https://index.ros.org/doc/ros2/Tutorials/Ament-CMake-Documentation/)
* [rclcpp](https://docs.ros2.org/latest/api/rclcpp/index.html)


## MoveIt 2 튜터리얼 소스 빌드

[MoveIt 2 소스 빌드](https://moveit.ros.org/install-moveit2/source/) 지침을 따라 moveit2를 소스로부터 colcon 워크스페이스를 설정하세요.

moveit2 colcon 워크스페이스에서 command line을 열어주세요:

    cd $COLCON_WS/src

MoveIt 튜터리얼 소스 코드를 다운로드하세요:

    git clone https://github.com/ros-planning/moveit2_tutorials.git
    vcs import < moveit2_tutorials/moveit2_tutorials.repos
    rosdep install -r --from-paths . --ignore-src --rosdistro humble -y

workspace를 설정하고 빌드하세요:

    cd $COLCON_WS
    colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release

## HTML 페이지 로컬로 빌드하기

여러분의 장치에서 로컬로 html 페이지를 생성해서 튜터리얼을 테스트하고자 한다면, ``build_locally`` 스크립트를 사용하여 moveit2_tutorials 패키지의 루트에서 다음 명령어를 사용하여 실행하세요:

    export ROS_DISTRO=humble  # 20.04

    cd $COLCON_WS/src/moveit2_tutorials
    source /opt/ros/$ROS_DISTRO/setup.bash
    ./build_locally.sh

로컬 웹사이트 ``<LOCAL_PACKAGE_PATH>/build/html/index.html``이 자동으로 웹 브라우저에서 열릴 것입니다.

### 옵션 build_locally 설정

 - *noinstall* skip the dependencies install step to speed up the script
 - *loop* automatically rebuild the html if a change is detected

### 포맷 및 스타일

These tutorials use the [reStructuredText](http://www.sphinx-doc.org/en/stable/rest.html) format commonly used in the Sphinx "Python Documentation Generator". This unfortunately differs from the common Markdown format, but its advantage is that it supports embedding code directly from source files for inline code tutorials.

**Code Formatting**

* These tutorials use the same [style guidelines](http://moveit.ros.org/documentation/contributing/code/) as the MoveIt project. When modifying or adding to these tutorials, it is required that code is auto formatted using [clang-format](http://moveit.ros.org/documentation/contributing/code/). To check and apply our style guidelines we use [pre-commit](https://pre-commit.com/).
* Tutorials should exemplify best coding practices. If a contribution wouldn't pass review in the MoveIt project, then it shouldn't pass review in the tutorials.
* Relevant code should be included and explained using the ``.. tutorial-formatter::`` tag.
* Irrelevant code should be excluded from the generated html using the ``BEGIN_TUTORIAL``, ``END_TUTORIAL``, ``BEGIN_SUB_TUTORIAL``, and ``END_SUB_TUTORIAL`` tags.
* Whenever possible, links should be created using the ``extlinks`` dictionary defined in ``conf.py``.
* All demo code should be runnable from within the ``moveit2_tutorials`` package.
* Python code should be run using ``ros2 run``.

**Style**

* Each tutorial should be focused on teaching the user one feature or interface within MoveIt.
* Tutorials should flow from show to tell with videos and demos at the beginning followed by explanations.
* New tutorials should match the formatting, style and flow of existing tutorials whenever possible.

**pre-commit**

pre-commit is a tool that is used in moveit2_tutorials to check and apply style guidelines automatically. To install pre-commit into your system:

    pip3 install pre-commit

Then under moveit2_tutorials directory install the git hooks like this:

    cd $COLCON_WS/src/moveit2_tutorials && pre-commit install

With this pre-commit will automatically run and check a list of styling including clang-format, end of files and trailing whitespaces whenever you run `git commit`. To run pre-commit any time other than `git commit`:

    cd $COLCON_WS/src/moveit2_tutorials && pre-commit run -a

### Including Images and Videos
#### Images
The standard way to include an image in reStructuredText is
```
.. image:: filename.png
   :width: 700px
```

This assumes that `filename.png` is in the same folder as the source `.rst` file. Images linked in this way will automatically be copied to the appropriate folder in the build.

[External Documentation](https://sublime-and-sphinx-guide.readthedocs.io/en/latest/images.html)

Do **not** include animated gifs as the file format leads to very large files. Use a video format like `webm` and see the section on local video below.

#### YouTube and other External Video
You can embed video with raw html, like in this example from the Pick and Place Tutorial.
```
.. raw:: html

    <div style="position: relative; padding-bottom: 5%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="700px" height="400px" src="https://www.youtube.com/embed/QBJPxx_63Bs?rel=0" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
    </div>
```
This includes [Youtube's suggested embed html](https://support.google.com/youtube/answer/171780?hl=en).

#### Local Video
To embed a video that is included in this repository, you also will use raw html, like this example from the Quickstart in RViz tutorial.

```
.. raw:: html

    <video width="700px" controls="true" autoplay="true" loop="true">
        <source src="../../../_static/videos/rviz_joints_nullspace.webm" type="video/webm">
        The joints moving while the end effector stays still
    </video>
```

Note that the video file is in the `_static/videos` folder instead of the same folder.

[External Documentation on &lt;video&gt; tag](https://developer.mozilla.org/en-US/docs/Web/HTML/Element/video)

## License

All content in this repository is open source and released under the [BSD License v3](https://opensource.org/licenses/BSD-3-Clause). Each individual source code file should contain a copy of the license.
