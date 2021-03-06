#!/bin/bash

ENABLE_BUILD=true
ENABLE_ROS2_MASTER=false
ENABLE_SLAM_TOOLBOX=true
ROS_ROOT=""
ROS2_DISTRO="galactic"
ROS2_DISTRO="humble"
NAV2_DISTRO=""



######################################################################################
#         The following section is ros2 source code download and building
######################################################################################


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
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


    # Install development tools and ROS2 tools
    sudo apt install -y build-essential cmake git libbullet-dev python3-colcon-common-extensions python3-flake8 python3-pip python3-pytest-cov python3-rosdep python3-setuptools python3-vcstool wget
    python3 -m pip install -U argcomplete flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pytest-repeat pytest-rerunfailures pytest
    # install Fast-RTPS dependencies
    sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
    # install Cyclone DDS dependencies
    sudo apt install --no-install-recommends -y libcunit1-dev
    
    # install nav2 depends lib boost
    sudo apt-get install libboost-all-dev
    sudo apt-get install libbondcpp-dev
    sudo apt-get install ros-humble-diagnostic-updater
    sudo apt-get install libudev-dev 

  install_gazebo
  install_rviz

  return_to_root_dir

}

install_gazebo() {
    #install Gazebo9
    #sudo apt install --no-install-recommends -y \
    #    libgazebo9-dev \
    #    gazebo9 \
    #    gazebo9-common \
    #    gazebo9-plugin-base
	
    #install Gazebo9
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install gazebo11
    sudo apt-get install libgazebo11-dev
    sudo apt-get install gazebo11-common
    sudo apt-get install gazebo11-plugin-base
}

install_rviz() {

    sudo apt-get -y install rviz
}

if [ "$ROS2_DISTRO" = "" ]; then
  export ROS2_DISTRO=foxy-devel
  ROS_ROOT=ros2_foxy_ws
  NAV2_DISTRO=foxy-devel
elif [ "$ROS2_DISTRO" = "galactic" ]; then
  export ROS2_DISTRO=galactic
  ROS_ROOT=ros2_galactic_ws
  NAV2_DISTRO=galactic
elif [ "$ROS2_DISTRO" = "humble" ]; then
  export ROS2_DISTRO=humble
  ROS_ROOT=ros2_humble_ws
  NAV2_DISTRO=humble
fi

if [ "$ROS2_DISTRO" != "foxy" ] && [ "$ROS2_DISTRO" != "galactic" ] && [ "$ROS2_DISTRO" != "humble" ]; then
  echo "ROS2_DISTRO variable must be set !"
  exit 1
fi

for opt in "$@" ; do
  case $opt in
    --ros2_master)
      ENABLE_ROS2_MASTER=true
      ROS2_DISTRO=master
      ROS_ROOT=ros2_master_ws
      NAV2_DISTRO=master
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
    --ros2_humble)
      ROS2_DISTRO=humble
      ROS_ROOT=ros2_humble_ws
      NAV2_DISTRO=humble
      shift
    ;;
    --ros2_foxy)
      ROS2_DISTRO=foxy-devel
      ROS_ROOT=ros2_foxy_ws
      NAV2_DISTRO=foxy-devel
      shift
    ;;
    --ros2_galactic)
      ROS2_DISTRO=galactic
      ROS_ROOT=ros2_galactic_ws
      NAV2_DISTRO=galactic
      shift
    ;;
    *)
      echo "Invalid option: $opt"
      echo "Valid options:"
      echo "--no-ros2       	Uses the binary distribution of ROS2 foxy"
      echo "--ros2_master   	Download ROS2 master source code "
      echo "--ros2_humble   	Download ROS2 humble source code "
      echo "--ros2_foxy     	Download ROS2 foxy-devel source code "
      echo "--ros2_galactic 	Download ROS2 galactic source code "
      echo "--download-only 	Skips the build step and only downloads the code"
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

download_ros2() {
  echo "Downloading ROS 2 $ROS2_DISTRO version"
  mkdir -p $ROS_ROOT/src
  cd $ROS_ROOT
  #mv -i ros2.repos ros2.repos.old
  #download release version
  wget https://raw.githubusercontent.com/ros2/ros2/$ROS2_DISTRO/ros2.repos
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
  echo "Downloading ROS 2 $ROS2_DISTRO version finished"
}

rosdep_install() {
  #sudo rosdep init
  #rosdep update
  #rosdep install -y -r -q --from-paths . --ignore-src --rosdistro $ROS2_DISTRO --skip-keys "catkin"
  cd $ROS_ROOT
  if [ "$ROS2_DISTRO" = "humble" ]; then
      #sudo rosdep init
      rosdep update
      #rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
      rosdep install --from-paths src --ignore-src --rosdistro $ROS2_DISTRO -y --skip-keys "fastcdr fastrtps rti-connext-dds-6.0.1 urdfdom_headers"
  else # for foxy and earlier version
      #sudo rosdep init
      rosdep update
      rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers"
  fi
  return_to_root_dir
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

###########################################################################################
# fix ros2 humble download and build errors 
# NOTICE: you need to do this manually if you can not download the packages directly.

#1
# git clone https://github.com/ros/console_bridge.git
# tar -c ./console_bridge/ |gzip -n >console_bridge_1.0.2.tar.gz
# md5sum console_bridge_1.0.2.tar.gz
# replace the following 
  # Download and build console_bridge
  #externalproject_add(console_bridge-1.0.2
  #  URL /work/ROS2_WS/ros2_humble_ws/humble_build_fix/console_bridge_1.0.2.tar.gz
  #  URL_MD5 b670ad938693082b4bb48110d721e9db
#2
    #URL https://github.com/OGRECave/ogre/archive/v1.12.1.zip
    #URL /work/ROS2_WS/ros2_humble_ws/humble_build_fix/ogre-1.12.1.zip 

#3  

#ExternalProject_Add(ext-singleproducerconsumer
#  PREFIX singleproducerconsumer
#  DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download
#  URL https://github.com/cameron314/readerwriterqueue/archive/ef7dfbf553288064347d51b8ac335f1ca489032a.zip
#  URL /work/ROS2_WS/ros2_humble_ws/humble_build_fix/readerwriterqueue-ef7dfbf553288064347d51b8ac335f1ca489032a.zip
  
# Concurrent and blocking concurrent queue by moodycamel - header only, don't build, install
#ExternalProject_Add(ext-concurrentqueue
 # PREFIX concurrentqueue
 # DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download
 # #URL https://github.com/cameron314/concurrentqueue/archive/8f65a8734d77c3cc00d74c0532efca872931d3ce.zip
 # URL /work/ROS2_WS/ros2_humble_ws/humble_build_fix/concurrentqueue-8f65a8734d77c3cc00d74c0532efca872931d3ce.zip
 
#4
# zlib-1.2.11.tar.gz


#build error
#--- stderr: nav2_util                               
#CMake Error at CMakeLists.txt:17 (find_package):
#  By not providing "Findbondcpp.cmake" in CMAKE_MODULE_PATH this project has
#  asked CMake to find a package configuration file provided by "bondcpp", but
#  CMake did not find one.
#
#  Could not find a package configuration file provided by "bondcpp" with any
#  of the following names:
#
#    bondcppConfig.cmake
#    bondcpp-config.cmake

# fix can not find libignition-math6.so.6.10.0
# sudo ln -s /usr/lib/x86_64-linux-gnu/libignition-math6.so.6 /usr/lib/x86_64-linux-gnu/libignition-math6.so.6.10.0

# build error can not find gazebo/common/Timer.hh
# add findpackage(gazebo, REQUIRED) to CMakelist.txt of the packages

#build error diagnostic_updater
#  add the following line to the end of ros2.repos
#  ros2/diagnostics:
#    type: git
#    url: https://github.com/ros/diagnostics.git
#    version: ros2-devel
############################################################################################


build_ros2() {
  return_to_root_dir
  cd $ROS_ROOT
  if [ "$ROS2_DISTRO" = "humble" ]; then
  	colcon build --symlink-install
  else
	colcon build --symlink-install --packages-skip ros1_bridge
  fi
  return_to_root_dir
}

build_ros2_master() {
  return_to_root_dir
  cd ros2_master_ws
  colcon build --symlink-install --packages-skip ros1_bridge
  return_to_root_dir
}

build_ros_bridge() {

  # Update the ROS1 bridge
  if test "$ENABLE_ROS1" = true && test "$ENABLE_ROS2" = true ; then
    cd $CWD/$ROS_ROOT
    (setup_ros2_env && setup_nav2_env && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure)
  fi
  return_to_root_dir
}

setup_ros2_env() {

  source $CWD/$ROS_ROOT/install/setup.bash
}


######################################################################################
#       The following section is navigation2 source code download and building
######################################################################################
download_nav2() {

  echo "Downloading the Navigation2 and its dependencies"
  mkdir -p navigation2_ws/src
  cd navigation2_ws
  if [ -f "custom_nav2.repos" ]; then #override default location for testing
    vcs import src < custom_nav2.repos
  else
    cd src
    git clone https://github.com/ros-planning/navigation2.git --branch $NAV2_DISTRO
  fi
  if [ "$ROS2_DISTRO" = "foxy-devel" ]; then
    fix_ros2_foxy_build_error
  fi
  
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
  if [ "$ROS2_DISTRO" = "foxy" ]; then
   echo \
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
	    version: $ROS2_DISTRO
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
	    version: ros2
	  ompl/ompl:
	    type: git
	    url: https://github.com/ompl/ompl.git
	    version: 1.5.0" > $CWD/navigation2_ws/src/navigation2/tools/ros2_nav2_dependencies.repos
   elif [ "$ROS2_DISTRO" = "galactic" ]; then
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
            version: '$ROS2_DISTRO'
          ros-perception/vision_opencv:
            type: git
            url: https://github.com/ros-perception/vision_opencv.git
            version: ros2
          ros/bond_core:
            type: git
            url: https://github.com/ros/bond_core.git
            version: ros2
          ompl/ompl:
            type: git
            url: https://github.com/ompl/ompl.git
            version: 1.5.0" > $CWD/navigation2_ws/src/navigation2/tools/ros2_nav2_dependencies.repos
  fi
}


download_nav2_depends() {
  echo "Downloading the dependencies workspace"
  return_to_root_dir
  
  mkdir -p ros2_nav_depends_ws/src
  cd ros2_nav_depends_ws
  
  if [ "$ROS2_DISTRO" = "humble" ]; then
      vcs import src < ${CWD}/navigation2_ws/src/navigation2/tools/underlay.repos
  else
      checkpoint generate_nav2_depends
      vcs import src < ${CWD}/navigation2_ws/src/navigation2/tools/ros2_nav2_dependencies.repos
  fi
  
  vcs pull src
  return_to_root_dir
  echo "Downloading the dependencies workspace finished!"
}


build_nav2_depends() {
  echo "Building the ros2_nav_depends_ws workspace"
  cd $CWD/ros2_nav_depends_ws
  export ROSDISTRO_INDEX_URL='https://raw.githubusercontent.com/ros2/rosdistro/ros2/index.yaml'
  setup_ros2_env
  colcon build --symlink-install
# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  echo "Build the ros2_nav_depends_ws workspace finished"
  return_to_root_dir
}

build_nav2() {
  echo "Building the navigation2_ws workspace"
  return_to_root_dir
  cd $CWD/navigation2_ws
  export ROSDISTRO_INDEX_URL='https://raw.githubusercontent.com/ros2/rosdistro/ros2/index.yaml'
  setup_ros2_env
  . $CWD/ros2_nav_depends_ws/install/setup.bash
  #rosdep update 
  rosdep install -y -r -q --from-paths src --ignore-src --rosdistro $ROS2_DISTRO
  colcon build --symlink-install
  echo "Build the navigation2_ws workspace finished"
  return_to_root_dir
}

setup_nav2_env() {

  source $CWD/navigation2_ws/install/setup.bash
  source $CWD/ros2_nav_depends_ws/install/setup.bash
}

######################################################################################
#       The following section is slam_toolbox source code download and building
######################################################################################

download_slam_toolbox() {
  return_to_root_dir
  echo "Downloading slam_toolbox $ROS2_DISTRO"
  mkdir -p slam_toolbox_ws/src
  cd slam_toolbox_ws/src
  git clone https://github.com/SteveMacenski/slam_toolbox.git -b $ROS2_DISTRO
  #git clone -b foxy-devel git@github.com:stevemacenski/slam_toolbox.git
  return_to_root_dir
  echo "Downloading slam_toolbox $ROS2_DISTRO finished!"
}

build_slam_toolbox() {
  echo "Building the slam_toolbox_ws workspace"
  return_to_root_dir
  cd slam_toolbox_ws
  (setup_ros2_env && setup_nav2_env && colcon build --symlink-install)
  echo "Build the slam_toolbox_ws workspace finished"
  return_to_root_dir
}

setup_slam_toolbox_env() {

  source $CWD/slam_toolbox_ws/install/setup.bash

}


######################################################################################
#       The following section is turtlebot3 source code download and building
######################################################################################

download_turtlebot3() {
  echo "Downloading Turtlebot3 source code"
  mkdir -p turtlebot3_ws/src
  cd turtlebot3_ws
  wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/$ROS2_DISTRO-devel/turtlebot3.repos
  vcs import src < turtlebot3.repos
  return_to_root_dir
  echo "Downloading Turtlebot3 source code finished!"
}

build_turtlebot3() {
  echo "Building the turtlebot3_ws workspace"
  return_to_root_dir
  cd turtlebot3_ws
  (setup_ros2_env && setup_nav2_env && colcon build --symlink-install)
  echo "Build the turtlebot3_ws workspace finished"
  return_to_root_dir
}

setup_turtlebot3_env() {

  source $CWD/turtlebot_ws/install/setup.bash
  
}

######################################################################################
#    The following setction will download and build ros2, nav2, 
#  slam_toolbox and turtlebot3
#####################################################################################

download_all() {
  checkpoint setup_ros2_environment
  checkpoint download_ros2
  if [ "$ENABLE_ROS2_MASTER" = true ]; then
    checkpoint download_ros2_master
  fi
  checkpoint download_nav2
  checkpoint download_nav2_depends
  checkpoint download_slam_toolbox
  checkpoint download_turtlebot3
  return_to_root_dir
}

build_all() {

  echo "Start building ros2 source code"
  #fix ros2 foxy build error
  echo $ROS_ROOT
    
  #build ros2
  checkpoint build_ros2
  #setup ros2 setup_ros2_enironment.sh
  echo "Setting up ros2 environment"
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

setup_env_all() {
  setup_ros2_env
  setup_nav2_env
  setup_slam_toolbox_env
  setup_turtlebot3_env
  echo "Sourced Ros2, nav2_depends, nav2, slam_toolbox, turtulebot3 environment!"
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

echo "This script will download the ROS2 source code and its dependencies, "
echo " the navigation2, the slam_toolbox, the turtlebot3 to the"
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
  
  rosdep_install
  echo "==========rosdep_install finished!======================"
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

