ROS QR Tracker
==============

A [QR code](https://en.wikipedia.org/wiki/QR_code) tracking package for [ROS.](http://www.ros.org/)

Usage
-----

Run with:

    rosrun ros_qr_tracker qr_tracker.py _topic:=/mycamera/compressed

On launch, the node is idle. To make it start tracking QR codes, run:

    rosservice call /qr_tracker/start

If any QR codes are found, they will be broadcast on the topic `/qr_tracker/matches` with you can monitor with:

    rostopic echo /qr_tracker/matches

If you'd like to track a QR code containing specific text, set a target with:

    rosservice call /qr_tracker/set_target "Some text"

To clean the target and track all QR codes, run:

    rosservice call /qr_tracker/clear_target

Development
-----------

To build, run:

    time catkin_make --pkg ros_qr_tracker
