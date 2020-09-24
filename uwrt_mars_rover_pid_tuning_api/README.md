# PID Tuning API Testing Procedure

## Initial Setup

Make sure the following lines are at the bottom of your .bashrc file:

```bash
#this line should already be there
source /opt/ros/melodic/setup.bash 
source <path to ur catkin ws>/devel/setup.bash
```

Now go to the uwrt_mars_rover_pid_tuning_api.h file in the package's include directory and change the can_interface to vcan0 instead of can0.

## Setting up virtual can

Run the following commands
```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

#check if vcan0 shows up
sudo ip link show vcan0
```

Now test that this was done properly with the following:
```bash
candump vcan0

#now open a new tab of the terminal and run
cangen vcan0 -v
```

This should generate can traffic in the candump window.

## Setting up virtual can

Run the following commands
```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

#check if vcan0 shows up
sudo ip link show vcan0
```

Now test that this was done properly with the following:
```bash
candump vcan0

#now open a new tab of the terminal and run
cangen vcan0 -v
```
This should generate can traffic in the candump window.

## Running the api node

In window separate from your candump window run:

```bash
roscore 
```

In a different window run:
```bash
rosrun uwrt_mars_rover_pid_tuning_api_node node 
```

Now open up a differen terminal and run
```bash
rqt
```

You can now use the rqt service caller to make pid_tuning_api service calls.