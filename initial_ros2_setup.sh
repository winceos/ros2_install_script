#!/bin/bash

ENABLE_BUILD=true
ENABLE_ROS2_MASTER=false
ENABLE_SLAM_TOOLBOX=true
ROS_ROOT=""

setup_ros2_environment()
{
    # Update repository and install dependencies
    echo -e "Starting PC setup ..."
    sudo apt update
    sudo apt upgrade -y
    sudo apt install -y ssh net-tools terminator chrony ntpdate curl vim git setserial
    sudo ntpdate ntp.ubuntu.com


    # Set locale
    sudo apt update && sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8


    # Add the ROS2 apt repository
    sudo apt update && sudo apt install -y curl gnupg2 lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


    # Install development tools and ROS2 tools
    sudo apt install -y build-essential cmake git libbullet-dev python3-colcon-common-extensions python3-flake8 python3-pip python3-pytest-cov python3-rosdep python3-setuptools python3-vcstool wget
    python3 -m pip install -U argcomplete flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pytest-repeat pytest-rerunfailures pytest
    # install Fast-RTPS dependencies
    sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
    # install Cyclone DDS dependencies
    sudo apt install --no-install-recommends -y \
  libcunit1-dev
  
    #install Gazebo 
    sudo apt install --no-install-recommends -y \
	libgazebo9-dev \
	gazebo9 \
	gazebo9-common \
	gazebo9-plugin-base

  return_to_root_dir

}

if [ "$ROS2_DISTRO" = "" ]; then
  export ROS2_DISTRO=foxy
  ROS_ROOT=ros2_foxy_ws
fi
if [ "$ROS2_DISTRO" != "foxy" ]; then
  echo "ROS2_DISTRO variable must be set to foxy"
  exit 1
fi

for opt in "$@" ; do
  case $opt in
    --ros2_master)
      ENABLE_ROS2_MASTER=true
      ROS_ROOT=ros2_master_ws
      shift
    ;;
    --download-only)
      ENABLE_BUILD=false
      shift
    ;;
    --no-download-slam_toolbox)
      ENABLE_SLAM_TOOLBOX=false
      shift
    ;;
    *)
      echo "Invalid option: $opt"
      echo "Valid options:"
      echo "--no-ros2       Uses the binary distribution of ROS2 foxy"
      echo "--download-only Skips the build step and only downloads the code"
      echo "--no-download-slam_toolbox Skips the slam_toolbox source code download"
      exit 1
    ;;
  esac
done

set -e
CHECKPOINT_FILES=''

CWD=`pwd`
return_to_root_dir() {
  cd $CWD
}

download_ros2_master() {
  echo "Downloading ROS 2 Release Latest"
  mkdir -p ros2_master_ws/src
  cd ros2_master_ws
  wget https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos
  vcs import src < ros2.repos
  return_to_root_dir
}

download_ros2_foxy() {
  echo "Downloading ROS 2 Foxy Release version"
  mkdir -p ros2_foxy_ws/src
  cd ros2_foxy_ws
  #mv -i ros2.repos ros2.repos.old
  #download release version
  wget https://raw.githubusercontent.com/ros2/ros2/foxy-release/ros2.repos
  #download development branch
  #wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
  #download lastest branch
  #wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
  echo "waiting for downloading ros2.repos"	
  sleep 5
  vcs import src < ros2.repos
  #vcs pull src
  
  #update the repositories that you have already checked out with the following command 
  #vcs custom --args remote update
  return_to_root_dir
  echo "Downloading ROS 2 Foxy Release version finished"
}

fix_ros2_foxy_build_error() {
  # fix broken package.xml in test_pluginlib that crops up if/when rosdep is run again
  #
  #   Error(s) in package '/opt/ros/foxy/build/pluginlib/prefix/share/test_pluginlib/package.xml':
  #   Package 'test_pluginlib' must declare at least one maintainer
  #   The package node must contain at least one "license" tag
  #
  #/home/oem/work/ros2_nav2_ws/build/pluginlib/prefix/share/test_pluginlib/package.xml
  TEST_PLUGINLIB_PACKAGE="$CWD/build/pluginlib/prefix/share/test_pluginlib/package.xml" && \
      sed -i '/<\/description>/a <license>BSD<\/license>' $TEST_PLUGINLIB_PACKAGE && \
      sed -i '/<\/description>/a <maintainer email="michael@openrobotics.org">Michael Carroll<\/maintainer>' $TEST_PLUGINLIB_PACKAGE && \
      cat $TEST_PLUGINLIB_PACKAGE

}

download_nav2_foxy() {

  echo "Downloading the Navigation2 and its dependencies"
  mkdir -p navigation2_ws/src
  cd navigation2_ws
  if [ -f "custom_nav2.repos" ]; then #override default location for testing
    vcs import src < custom_nav2.repos
  else
    cd src
    git clone https://github.com/ros-planning/navigation2.git --branch foxy-devel
  fi
  fix_ros2_foxy_build_error
  
  return_to_root_dir
}

#download_nav2_depends() {
#  #install navigation2 dependencies
#  cd navigation2_ws
#  sudo rosdep init
#  rosdep update
#  rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy
  
#  return_to_root_dir
  
#  echo "Downloading the Navigation2 and its dependencies finished"
#}

generate_nav2_depends()
{
   echo  \
  "repositories:
  BehaviorTree/BehaviorTree.CPP:
    type: git
    url: https://github.com/BehaviorTree/BehaviorTree.CPP.git
    version: master
  ros/angles:
    type: git
    url: https://github.com/ros/angles.git
    version: ros2
  ros-simulation/gazebo_ros_pkgs:
    type: git
    url: https://github.com/ros-simulation/gazebo_ros_pkgs.git
    version: foxy
  ros-perception/image_common:
    type: git
    url: https://github.com/ros-perception/image_common.git
    version: ros2
  ros-perception/vision_opencv:
    type: git
    url: https://github.com/ros-perception/vision_opencv.git
    version: ros2
  ros/bond_core:
    type: git
    url: https://github.com/ros/bond_core.git
    version: foxy-devel
  ompl/ompl:
    type: git
    url: https://github.com/ompl/ompl.git
    version: 1.5.0" > ${CWD}/navigation2_ws/src/navigation2/tools/foxy_nav2_dependencies.repos
}


download_nav2_depends() {
  echo "Downloading the dependencies workspace"
  mkdir -p ros2_nav_dependencies_ws/src
  cd ros2_nav_dependencies_ws
  generate_nav2_depends
  vcs import src < ${CWD}/navigation2_ws/src/navigation2/tools/foxy_nav2_dependencies.repos
  vcs pull src
  return_to_root_dir
  echo "Downloading the dependencies workspace finished!"
}

download_slam_toolbox_foxy() {
  return_to_root_dir
  echo "Downloading slam_toolbox foxy-devel release"
  mkdir -p slam_toolbox_ws/src
  cd slam_toolbox_ws/src
  git clone https://github.com/SteveMacenski/slam_toolbox.git -b foxy-devel
  #git clone -b foxy-devel git@github.com:stevemacenski/slam_toolbox.git
  return_to_root_dir
  echo "Downloading slam_toolbox foxy-devel release finished!"
}

download_turtlebot3_foxy() {
  echo "Downloading Turtlebot3 source code"
  mkdir -p turtlebot3_ws/src
  cd turtlebot3_ws
  wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/foxy-devel/turtlebot3.repos
  vcs import src < turtlebot3.repos
  return_to_root_dir
  echo "Downloading Turtlebot3 source code finished!"
}

checkpoint() {
  local CHECKPOINT_FILE_NAME=.INITIAL_SETUP_$1
  CHECKPOINT_FILES="${CHECKPOINT_FILES} ${CHECKPOINT_FILE_NAME}"
  if [ ! -f ${CHECKPOINT_FILE_NAME} ]; then
    $1
    touch ${CHECKPOINT_FILE_NAME}
  else
    echo "${CHECKPOINT_FILE_NAME} exists. Skipping $1"
  fi
}

download_all() {
  checkpoint setup_ros2_environment
  checkpoint download_ros2_foxy
  if [ "$ENABLE_ROS2_MASTER" = true ]; then
    checkpoint download_ros2_master
  fi
  checkpoint download_nav2_foxy
  checkpoint download_nav2_depends
  checkpoint download_slam_toolbox_foxy
  checkpoint download_turtlebot3_foxy
  return_to_root_dir
}

setup_ros2_env() {

  source $CWD/ros2_foxy_ws/install/setup.bash
}

setup_nav2_env() {

  source $CWD/navigation2_ws/install/setup.bash
  source $CWD/ros2_nav_dependencies_ws/install/setup.bash
}

setup_slam_toolbox_env() {

  source $CWD/slam_toolbox_ws/install/setup.bash
}

setup_turtlebot3_env() {

  source $CWD/turtlebot_ws/install/setup.bash
}

setup_env_all() {
  setup_ros2_env
  setup_nav2_env
  setup_slam_toolbox_env
  setup_turtlebot3_env
  echo "Sourced Ros2, nav2_depends, nav2, slam_toolbox, turtulebot3 environment!"
}

build_all() {

  echo "Start building ros2 foxy source code"
  #fix ros2 foxy build error
  echo ${ROS_ROOT}
  fix_ros2_foxy_build_error
  
  #build ros2 foxy
  checkpoint build_ros2_foxy
  #setup ros2 setup_ros2_enironment.sh
  echo "Setting up ros2 foxy environment"
  setup_ros2_env
  
  #build nav2\slam_toolbox\turtlebot3
  echo "Start building nav2 source code"
  checkpoint build_nav2_depends
  checkpoint build_nav2
  echo "Start building slam_toolbox source code"
  checkpoint build_slam_toolbox
  echo "Start building turtlebot3 source code"
  checkpoint build_turtlebot3
  echo "Start building ros_bridge source code"
  checkpoint build_ros_bridge
    
  return_to_root_dir
}

rosdep_install() {
  rosdep install -y -r -q --from-paths . --ignore-src --rosdistro foxy --skip-keys "catkin"
  #sudo rosdep init
  #rosdep update
  #rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers"
}

build_ros2_foxy() {
  return_to_root_dir
  cd ros2_foxy_ws
  colcon build --symlink-install --packages-skip ros1_bridge
  return_to_root_dir
}

build_ros2_master() {
  return_to_root_dir
  cd ros2_master_ws
  colcon build --symlink-install --packages-skip ros1_bridge
  return_to_root_dir
}

build_nav2_depends() {
  cd $CWD/ros2_nav_dependencies_ws
  export ROSDISTRO_INDEX_URL='https://raw.githubusercontent.com/ros2/rosdistro/ros2/index.yaml'
  setup_ros2_env
  colcon build --symlink-install
  return_to_root_dir
}

build_nav2() {
  return_to_root_dir
  cd $CWD/navigation2_ws
  export ROSDISTRO_INDEX_URL='https://raw.githubusercontent.com/ros2/rosdistro/ros2/index.yaml'
  setup_ros2_env
  . $CWD/ros2_nav_dependencies_ws/install/setup.bash
  colcon build --symlink-install
  return_to_root_dir
}

build_ros_bridge() {

  # Update the ROS1 bridge
  if test "$ENABLE_ROS1" = true && test "$ENABLE_ROS2" = true ; then
    cd $CWD/ros2_foxy_ws
    (setup_ros2_env && setup_nav2_env && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure)
  fi
  return_to_root_dir
}

build_slam_toolbox() {
  return_to_root_dir
  cd slam_toolbox_ws
  (setup_ros2_env && setup_nav2_env && colcon build --symlink-install)
  return_to_root_dir
}

build_turtlebot3() {
  return_to_root_dir
  cd turtlebot3_ws
  (setup_ros2_env && setup_nav2_env && colcon build --symlink-install)
  return_to_root_dir
}

echo "This script will download the ROS 2 foxy release workspace by default, the"
echo "dependencies workspace and the ros_navstack_port workspace to the"
echo "current directory and then build them all. There should be no ROS"
echo "environment variables set at this time."
echo "If you want to download lastest(master) ros2 release workspace, please add --ros2_master" 
echo
echo "The current directory is $CWD"
echo
echo "Are you sure you want to continue? [yN]"
read -r REPLY
echo
if [ "$REPLY" = "y" ]; then
  download_all
  echo "==========Source code downloading finished!======================"
  fix_ros2_foxy_build_error
  rosdep_install
  echo "==========rosdep_install finished!======================"
  echo "==========fix_ros2_foxy_build_error finished!======================"
  #if [ "$ENABLE_BUILD" = true ]; then
  #  $CWD/navigation2_ws/src/navigation2/tools/build_all.sh
  #fi
  if [ "$ENABLE_BUILD" = true ]; then
    echo "==========Start building the source code======================"
    build_all
  fi

  cd ${CWD}
  rm ${CHECKPOINT_FILES}
  echo
  echo "Everything downloaded and built successfully."
  echo "To use the navstack source the setup.bash in the install folder"
  echo
  echo "> source navigation2/install/setup.bash"
  echo
  echo "To build the navstack you can either"
  echo "1. Run 'colcon build --symlink-install' from the navigation2 folder"
  echo "2. or run 'make' from navigation2/build/<project> folder"
fi
