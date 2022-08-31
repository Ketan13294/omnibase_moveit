#!/bin/sh
export RML_WORKSPACE=$HOME/rml_workspace
mkdir -p $RML_WORKSPACE/src/rml
rosws init -c $RML_WORKSPACE/src/rml /opt/ros/`rosversion -d`
cd $RML_WORKSPACE
cd src/rml
rosws set -y omnibase_moveit https://github.com/Ketan13294/omnibase_moveit.git --git --version=master
rosws set -y omnibase https://github.com/ERC-BPGC/omnibase.git --git --version=master
rosws update omnibase_moveit omnibase
cd $RML_WORKSPACE
catkin init
catkin build
source ./devel/setup.bash
