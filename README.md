# MoveIt Tutorials
[Live 튜터리얼은 여기에서](https://moveit.picknik.ai/)

MoveIt 프로젝트에 대한 문서입니다.

## Build Status

이 repository는 현재 Github Actions에 의해 자동으로 빌드됩니다:

- **Rolling** (main): [![CI](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/ci.yaml/badge.svg?branch=main)](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/ci.yaml?query=branch%3Amain) [![Format](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/format.yml/badge.svg?branch=main)](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/format.yml?query=branch%3Amain) [![Deploy](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/deploy.yml/badge.svg?branch=main)](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/deploy.yml?query=branch%3Amain)
- **Humble**: [![CI](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/ci.yaml/badge.svg?branch=humble)](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/ci.yaml?query=branch%3Ahumble) [![Format](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/format.yml/badge.svg?branch=humble)](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/format.yml?query=branch%3Ahumble) [![Deploy](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/deploy.yml/badge.svg?branch=humble)](https://github.com/ros-planning/moveit2_tutorials/actions/workflows/deploy.yml?query=branch%3Ahumble)

## Contributing

여러분이 참여하여 MoveIt의 문서를 개선하는 것을 권장합니다. 이 튜토리얼을 개선하는 데 도움을 주시기 바랍니다. ROS 1의 이전 튜토리얼을 포팅하고 새로운 튜토리얼을 작성하는 방식으로 이 튜터리얼을 개선하는데 도움을 주세요. 아래의 품질 기준을 읽어보시고 [MoveIt 튜터리얼 작성하기](https://moveit.picknik.ai/main/doc/how_to_contribute/how_to_write_tutorials.html) 페이지를 참조하십시오.

이 튜터리얼에서 이슈를 발견했는데 여러분이 수정하기 어렵다면 [GitHub에 이슈를 오픈](https://github.com/ros-planning/moveit2_tutorials/issues/new) 혹은 PR을 제안해 주세요.

## Helping with Porting Tutorials to ROS 2

각 튜터리얼을 ROS 2로 포팅하면서 이슈가 발생합니다. 각 튜터리얼의 맨 위에 ":moveit1:" 태그가 있는데 이 태그는 튜터리얼이 성공적으로 업데이트한 후에 삭제됩니다.

아래에 포팅하는데 도움이 되는 링크가 있습니다.

* [colcon](https://colcon.readthedocs.io/en/released/user/how-to.html)
* [ament](https://index.ros.org/doc/ros2/Tutorials/Ament-CMake-Documentation/)
* [rclcpp](https://docs.ros2.org/latest/api/rclcpp/index.html)


## MoveIt Tutorials Source Build

[MoveIt 2 소스 빌드](https://moveit.ros.org/install-moveit2/source/) 지침을 따라 moveit2의 소스 코드로 colcon 워크스페이스를 설정하세요.

moveit2 colcon 워크스페이스에서 command line을 열어주세요:

    cd $COLCON_WS/src

MoveIt 튜터리얼 소스 코드를 다운로드하세요:

    git clone https://github.com/ros-planning/moveit2_tutorials.git
    vcs import < moveit2_tutorials/moveit2_tutorials.repos
    rosdep install -r --from-paths . --ignore-src --rosdistro rolling -y

workspace를 설정하고 빌드하세요:

    cd $COLCON_WS
    colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release

## Build HTML Pages Locally

여러분의 장치에서 로컬로 html 페이지를 생성해서 튜터리얼을 테스트하고자 한다면, ``build_locally`` 스크립트를 사용하여 moveit2_tutorials 패키지의 root에서 다음 명령어를 실행하세요:

    export ROS_DISTRO=rolling  # 20.04

    cd $COLCON_WS/src/moveit2_tutorials
    source /opt/ros/$ROS_DISTRO/setup.bash
    ./build_locally.sh

로컬 웹사이트 ``<LOCAL_PACKAGE_PATH>/build/html/index.html``가 자동으로 웹 브라우저에서 열릴 것입니다.

### Optional build_locally Settings

 - *noinstall* 의존 설치 단계를 건너뛰어 스크립트 속도를 높입니다.
 - *loop*: 변경 사항이 감지되면 자동으로 HTML을 재빌드합니다.

### Formatting and Style

이 튜토리얼은 Sphinx "Python 문서 생성기"에서 일반적으로 사용되는 [reStructuredText](http://www.sphinx-doc.org/en/stable/rest.html) 형식을 사용합니다. 불행히도 이 형식은 일반적인 Markdown 형식과 다르지만, 소스 파일에 있는 코드를 직접 포함하여 인라인 코드 튜토리얼을 지원한다는 장점이 있습니다.

**Code Formatting**

* 이 튜토리얼은 MoveIt 프로젝트와 동일한 스타일 가이드: [style guidelines](http://moveit.ros.org/documentation/contributing/code/) 사용합니다. 이 튜토리얼을 수정하거나 추가할 때는 코드를 [clang format](http://moveit.ros.org/documentation/contributing/code/) 사용하여 자동 포맷팅해야 합니다. 스타일 가이드를 확인하고 적용하기 위해 [pre-commit](https://pre-commit.com/)을 사용합니다.
* 튜토리얼은 최고의 코딩 실습을 예제로 제시합니다. 만약 MoveIt 프로젝트에서 리뷰를 통과하지 못하는 컨트리뷰션이라면, 튜토리얼에서도 리뷰를 통과하지 못해야 합니다.
* 관련 코드는 ``.. tutorial-formatter::`` 태그를 사용하여 포함 및 설명해야 합니다.
* 관련 없는 코드는 ``BEGIN_TUTORIAL``, ``END_TUTORIAL``, ``BEGIN_SUB_TUTORIAL``, ``END_SUB_TUTORIAL`` 태그를 사용하여, 생성한 HTML에서 제외됩니다.
* 가능한 경우, ``conf.py``에 정의된 ``extlinks`` 사전을 사용하여 링크를 만들어야 합니다.
* 모든 데모 코드는 ``moveit2_tutorials`` 패키지 내에서 실행 가능해야 합니다.
* Python 코드는 ros2 run을 사용하여 실행해야 합니다.

**Style**

* Each tutorial should be focused on teaching the user one feature or interface within MoveIt.
* Tutorials should flow from show to tell with videos and demos at the beginning followed by explanations.
* New tutorials should match the formatting, style, and flow of existing tutorials whenever possible.

**pre-commit**

pre-commit is a tool that is used in moveit2_tutorials to check and apply style guidelines automatically. To install pre-commit into your system:

    pip3 install pre-commit

Then under the moveit2_tutorials directory install the git hooks like this:

    cd $COLCON_WS/src/moveit2_tutorials && pre-commit install

With this pre-commit will automatically run and check a list of styling including clang-format, end of files, and trailing whitespaces whenever you run `git commit`. To run pre-commit any time other than `git commit`:

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

Do **not** include animated gifs as the file format leads to very large files. Use a video format like `webm` and see the section on the local video below.

#### YouTube and other External Video
You can embed video with raw HTML, like in this example from the Pick and Place Tutorial.
```
.. raw:: html

    <div style="position: relative; padding-bottom: 5%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="700px" height="400px" src="https://www.youtube.com/embed/QBJPxx_63Bs?rel=0" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
    </div>
```
This includes [Youtube's suggested embed HTML](https://support.google.com/youtube/answer/171780?hl=en).

#### Local Video
To embed a video that is included in this repository, you also will use raw HTML, like this example from the Quickstart in RViz tutorial.

```
.. raw:: html

    <video width="700px" controls="true" autoplay="true" loop="true">
        <source src="../../../_static/videos/rviz_joints_nullspace.webm" type="video/webm">
        The joints move while the end effector stays still
    </video>
```

Note that the video file is in the `_static/videos` folder instead of the same folder.

[External Documentation on &lt;video&gt; tag](https://developer.mozilla.org/en-US/docs/Web/HTML/Element/video)

## License

이 저장소에 있는 모든 내용은 오픈소스이며, [BSD License v3](https://opensource.org/licenses/BSD-3-Clause) 따라 배포됩니다. 각 소스 코드 파일에는 라이센스 사본이 포함되어야 합니다.
