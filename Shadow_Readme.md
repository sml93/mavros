Instructions on launching the files for navigation for the Shadow Drone:
======

RUN sudo chmod 666 /dev/ttyPixhawk to enable read serial from the Pixhawk 2.1 FCU

RUN roslaunch mavros shadow.launch to establish connection with Pixhawk 2.1 FCU

RUN rosrun mavros shadow.final (trajectory within the room) for navigation, takeoff in stabilise mode and switch to Offboard 

OR 

RUN rosrun mavros shadow.corner (fly forward along the corner of the room and then back but with greater altitude) for navigation, takeoff in stabilise mode and switch to Offboard

Scripts can be found under mavros/mavros/scripts

ADVISED TO RUN ON PYTHON 2.7.12

