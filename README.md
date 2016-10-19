ROS QR Tracker
==============

A [QR code](https://en.wikipedia.org/wiki/QR_code) tracking package for [ROS.](http://www.ros.org/)

Installation
------------

Install dependencies:

    sudo apt-get install libffi-dev libffi6 libzbar-dev
    pip install libzbar-cffi

Checkout with git into your Catkin workspace or overlay and then build:

    mkdir -p overlay/src
    cd overlay/src
    catkin_init_workspace
    git clone https://github.com/chrisspen/ros_qr_tracker.git
    cd ..
    catkin_make --pkg ros_qr_tracker

Usage
-----

Run with:

    rosrun ros_qr_tracker qr_tracker.py _topic:=/mycamera/image/compressed

On launch, the node is idle. To make it start tracking QR codes, run:

    rosservice call /qr_tracker/start

If any QR codes are found, they will be broadcast on the topic `/qr_tracker/matches` with you can monitor with:

    rostopic echo /qr_tracker/matches

If you'd like to track a QR code containing specific text, set a target with:

    rosservice call /qr_tracker/set_target "Some text"

To clean the target and track all QR codes, run:

    rosservice call /qr_tracker/clear_target

To test with a webcam run:

    rosrun ros_qr_tracker webcam.py _start:=1

and view this with:

    sudo apt-get install ros-kinetic-rqt-image-view
    rosrun rqt_image_view rqt_image_view image:=/webcam/image _image_transport:=compressed

Then print out a QR code, slow place in front of the camera, and confirm messages are broadcast from `/qr_tracker/matches`.
