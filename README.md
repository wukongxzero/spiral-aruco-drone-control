# spiral-aruco-drone-control

Here is the way to run it on your machine, thankfully ros2 has deleted the entirety of sdf written launch files .. which means consequent pcs have better support but the issue with using ros2 humble was that it needs gazebo garden no gazebo classic and most of the libraries have dependency conflicts, i tried and tested the thing it sucks a lot.

so here are the requirements to run the code:

system: ubuntu 22.04
ros: ros2; foxy.

install ros2 foxy along with px4 by going throught the docs.

install mavsdk from https://mavsdk.mavlink.io/main/en/

create 
mkdir ros2_ws
cd ros2_ws 
and then, mkdir src
run colcon build 

after running colcon build get into the directory src

clone this repository
git clone https://github.com/wukongxzero/spiral-aruco-drone-control

change directory into ros2_ws

click on colcon build --symlink install 
