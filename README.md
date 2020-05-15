These notes are intended as a reminder of how I think things are working on my local machine.



### Launch the SubT Simulator

Currently, I am launching the simulator from a local catkin system setup. Followed installation and setup instructions: https://github.com/osrf/subt/wiki/Catkin%20System%20Setup


#### Terminal 1

```
# Source SubT setup file
source ~/subt_ws/install/setup.bash

ign launch -v 4 tunnel_circuit_practice.ign worldName:=tunnel_circuit_practice_01 robotName1:=X1 robotConfig1:=X1_SENSOR_CONFIG_1
```

#### Terminal 2

The myfirst package is installed locally in a catkin workspace named subt_practice.

```
cd ~/subt_practice

# think this is necessary to access the subt shared object libraries
source ~/subt_ws/install/setup.bash

# source the local installation 
source ./install/setup.bash

# launch the find entrance program

roslaunch myfirst find_entrance.launch robotName:=X1
```
