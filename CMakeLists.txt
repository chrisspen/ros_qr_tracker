cmake_minimum_required(VERSION 2.8.3)

project(ros_qr_tracker)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs message_generation)

catkin_python_setup()

add_message_files(FILES
    Percept.msg
)

add_service_files(FILES
    AddTarget.srv
    SetTarget.srv
)

generate_messages(   
    DEPENDENCIES  
    std_msgs  
)  

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

#############
## Install ##
#############

install(PROGRAMS
     scripts/create_qr_code.py
     nodes/qr_tracker.py
     nodes/webcam.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add folders to be run by python nosetests
catkin_add_nosetests(tests)
