#!/bin/bash
# Performs a test compilation of the code in a temporary ROS catkin workspace.
set -e

ROS_DISTRO=kinetic

TEST_DIR=/tmp/ros_qr_tracker_test/catkin_ws
[ -d $TEST_DIR ] && rm -Rf $TEST_DIR
mkdir -p $TEST_DIR/src

source /opt/ros/$ROS_DISTRO/setup.bash
export ROS_WORKSPACE=$TEST_DIR

cp --recursive . $TEST_DIR/src/ros_qr_tracker

cd $TEST_DIR

time catkin_make
time catkin_make run_tests
