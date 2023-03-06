# ZOS v 0.01 Alpha

a.k.a Ze-O-S

the new hope

# TODO

 - [ ] Test OpenGL rendering in Jetson Orin
 - [ ] Add OpenGL Vizualization segments to each main code
 - [ ] The main scripts should automatically detect architectures (arm/amd) ans should build automatically.
 - [ ] The main scripts should pick up indivudual run configurations for each node like (render/no-render) as an argument or from a central json like file
 - [ ] Create two releases will all assests for both arm and amd
 - [ ] Add doxygen style documentation to all the codes
 - [ ] Build doxygen in the repo itself as docs submodule
 - [ ] Have a central build folder instead of individual build in each src.
 - [ ] Try eCAL recording and replaying them as virtual sensor nodes.
 - [ ] Implement virtual sensors subscribers in each code.
 - [ ] Rename all file/functions/variables to remove ros lilke schema and have a standard schema
 - [ ] Clean and re-factor localization code to make it readable. Violates good-coding practices. Uncessary and extreme bifurcations/repetitions.
 - [ ] Refactor vehicleio module.
 - [ ] Implement two nodes for zui-server. (Render View (Three.js, regl) and Debug View (carlaviz, xviz, foxglove c++ websocket custom json))
 - [ ] Is jetson-utils being used? Remove it otherwise
 - [ ] Convert parallel parking to C++ and non-ROS
 - [ ] Implement GPS waypoint collection as part of copy of ekf node to give correct waypoints
 - [ ] Complete globalplanner.hpp to handle picking up of waypoints
 - [ ] Send only binary data over ecal publishers and not strings
 - [ ] Refactor SAL submodule to make it more consistent.
 - [ ] Implement Yolov8 s (Train and incorporate in zOS)
 - [ ] Add failsafe immediate braking in vehicleio. Later, it could be upgraded to deaccelerated braking.
 - [ ] Implement cansend using jetsonio
 - [ ] Do controller track tuning. along with desired speed also output desired acceleration. (throttle and acceleration are not same).
 - [ ] Data-logging in ecal format - one round.
 - [ ] Kalman tuning. (first without rtk, wheel and 1 gps, then without rtx, without wheel + 1 gps, then any other addition if dire need)
 - [ ] Try and configure new IMU
 - [ ] Have a good understanding of ecal sys, monitor and recorder.
 - [ ] Configure node shutdown error checks.
 - [ ] Restrict computes for each algo.
 - [ ] Calibrate homography
 - [ ] Calibrate waypoint correlation
 - [ ] Calibration the motion planning params and top view size of vehicles.
 - [ ] Implement connected compoments to handle person on a bike
 - [ ] Design and implement teleops feature using multiple remote controls over separate computer.
 - [ ] Implement backup camera on separete computer

