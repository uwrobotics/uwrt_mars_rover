# rover_simulation

This package contains the simulation configurations and launch files for the UWRT Mars Rover

## Launch Files

### \<WORLD\>.launch

A number of launch files are located within `launch` to simulate the rover in different Gazebo worlds.

The `world.launch` is a generic world launch file, where the world may be specified through an argument.

The rationale for using a single launch file for worlds (i.e. `empty_world.launch`, `willow_garage.launch`) is due to the fact that spawn settings might want to be permanently configured for each world.

### rover.launch

Responsible for launching the rover `robot_description` and loading rover specific configurations for Gazebo.

## Config Files

### rover_control.yaml

Specifies the `ros_control` controller configurations for each joint

## FAQs

1. Default worlds provided with the ROS installation for melodic can be found at `/usr/share/gazebo-9/worlds` and be used with the `world.launch` file by specifying the `world_name` argument, where `world_name` should be set to `worlds/<WORLD>.world`
