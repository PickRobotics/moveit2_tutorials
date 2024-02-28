시작하기
===============

여기서 튜토리얼을 가장 잘 실행하기 위한 환경을 설정할 것입니다. Colcon 워크스페이스를 만들고, 모든 최신 MoveIt 소스 코드를 다운로드하고, 모든 것을 소스에서 빌드하여 최신 수정 사항과 개선 사항을 보장합니다.

MoveIt의 모든 소스 코드를 빌드하는 데는 20-30분이 걸릴 수 있습니다. 컴퓨터의 CPU 속도와 사용 가능한 RAM에 따라 다르지만, 성능이 떨어지는 시스템에서 시작하거나 일반적으로 빨리 시작하려는 경우 :doc:`Docker 가이드 </doc/how_to_guides/how_to_setup_docker_containers_in_ubuntu>`를 확인하십시오.

ROS 2와 Colcon 설치하기
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
:ros_documentation:`Install ROS 2 {DISTRO_TITLE}<Installation.html>`.
ROS 2 설치 튜토리얼을 진행할 때 단계를 놓치기 쉽습니다. 다음 단계에서 오류가 발생하면 ROS 2를 올바르게 설치했는지 확인하는 것이 좋은 시작점입니다. 사용자가 자주 잊어버리는 것 중 하나는 ROS 2 설치 자체를 소스하는 것입니다. ::

  source /opt/ros/{DISTRO}/setup.bash

.. note:: Unlike ROS 1 setup scripts, in ROS 2 the setup scripts do not attempt to switch what version of ROS you are using.  This means that if you have previously sourced a different version of ROS, including from within your ``.bashrc`` file, you will run into errors during the building step.  To fix this change what is sourced in your ``.bashrc`` and start a new terminal.

Install `rosdep <http://wiki.ros.org/rosdep>`_ to install system dependencies : ::

  sudo apt install python3-rosdep

Once you have ROS 2 installed, make sure you have the most up to date packages: ::

  sudo rosdep init
  rosdep update
  sudo apt update
  sudo apt dist-upgrade

Install :ros_documentation:`Colcon <Tutorials/Colcon-Tutorial.html#install-colcon>` the ROS 2 build system with `mixin <https://github.com/colcon/colcon-mixin-repository>`_: ::

  sudo apt install python3-colcon-common-extensions
  sudo apt install python3-colcon-mixin
  colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
  colcon mixin update default

Install `vcstool <https://index.ros.org/d/python3-vcstool/>`_ : ::

  sudo apt install python3-vcstool

Create A Colcon Workspace and Download Tutorials
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
For tutorials you will need to have a :ros_documentation:`colcon <Tutorials/Colcon-Tutorial.html#install-colcon>` workspace setup. ::

  mkdir -p ~/ws_moveit2/src

Download Source Code of MoveIt and the Tutorials
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Move into your Colcon workspace and pull the MoveIt tutorials source: ::

  cd ~/ws_moveit2/src
  git clone --branch {DISTRO} https://github.com/ros-planning/moveit2_tutorials

Next we will download the source code for the rest of MoveIt: ::

  vcs import < moveit2_tutorials/moveit2_tutorials.repos

The import command may ask for your GitHub credentials. You can just press Enter until it moves on (ignore the "Authentication failed" error).

Build your Colcon Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following will install from Debian any package dependencies not already in your workspace. This is the step that will install MoveIt and all of its dependencies: ::

  sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

The next command will configure your Colcon workspace: ::

  cd ~/ws_moveit2
  colcon build --mixin release

This build command will likely take a long time (20+ minutes) depending on your computer speed and amount of RAM available (we recommend 32 GB). If you are short on computer memory or generally your build is struggling to complete on your computer, you can append the argument ``--parallel-workers 1`` to the colcon command above.

If everything goes well, you should see the message "finished". If you have problems, try re-checking your `ROS Installation <https://docs.ros.org/en/{DISTRO}/Installation.html>`_.

Setup Your Colcon Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Source the Colcon workspace: ::

  source ~/ws_moveit2/install/setup.bash

Optional: add the previous command to your ``.bashrc``: ::

   echo 'source ~/ws_moveit2/install/setup.bash' >> ~/.bashrc

.. note:: Sourcing the ``setup.bash`` automatically in your ``~/.bashrc`` is
   not required and often skipped by advanced users who use more than one
   Colcon workspace at a time, but we recommend it for simplicity.

Switch to Cyclone DDS
^^^^^^^^^^^^^^^^^^^^^

As of Sep 26, 2022, the default ROS 2 middleware (RMW) implementation has an issue. As a workaround, switch to Cyclone DDS. (Note: this makes all nodes started using this RMW incompatible with any other nodes not using Cyclone DDS.) ::

  sudo apt install ros-{DISTRO}-rmw-cyclonedds-cpp
  # You may want to add this to ~/.bashrc to source it automatically
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

다음 단계
^^^^^^^^^
Nice job! Next we will :doc:`Visualize a robot with the interactive motion planning plugin for RViz </doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial>`
