# uwrt_mars_rover

[![Build Status](https://travis-ci.com/uwrobotics/uwrt_mars_rover.svg?branch=master)](https://travis-ci.com/uwrobotics/uwrt_mars_rover)

## Repository Setup

To get started with development, clone this repo (and it's submodules) into the `src` directory of your `catkin` workspace. Use `rosinstall` and `rosdep` to fetch any source or binary dependencies.

```bash
# Ensure rosinstall and rosdep are installed and up to date
sudo apt install python-rosinstall python-rosdep
sudo rosdep init
rosdep update

# Clone repo
cd <catkin workspace location>/src
git clone --recurse-submodules https://github.com/uwrobotics/uwrt_mars_rover.git

# Install all dependencies
rosinstall --catkin . uwrt_mars_rover/uwrt.rosinstall
rosdep install --from-paths . --ignore-src -r -y
```

## Dependencies
For ease of setup on the Jetson, we should always try to use prebuilt binaries of dependent ROS packages, instead of 
including their sources. This means that if a package you write is dependent on a 3rd party package, you should just 
declare that dependency in the `CMakelists.txt` and `package.xml`. `rosdep` will take care of resolving the 
dependencies. If you need to update binary dependencies, run the following commands:
```bash
cd <catkin workspace location>/src
rosdep install --from-paths . --ignore-src -r -y
```

In the event that you cannot use the binaries (ex. we rely on a feature that has not been released), the package source 
code should be cloned outside of our metapackage, so that our CI doesn't run linting/formatting checks on it. To do 
this, declare the source dependency in `uwrt.rosinstall`. If using unreleased features(ie. cloning 3rd party master branch), 
please pin the rosinstall entry to a commit hash, rather than the branch. `rosinstall` will take care of cloning the 
source dependencies declared in `uwrt.rosinstall`. If you need to update source dependencies, run the following commands:
```bash
cd <catkin workspace location>/src
rosinstall --catkin . uwrt_mars_rover/uwrt.rosinstall
```

If you need to declare dependencies that are not ROS packages, typically you can declare them as system dependencies in
a `package.xml`. If it is an unreleased source dependency, (ex. the roboteq c++ driver we wrote), declare it as a git 
submodule in the package that uses it. Source code modules are still subject to the clangformat and clangtidy checks 
because they should only consist of code the team has written. To update submodules, run the following commands:
```bash
cd <catkin workspace location>/src/uwrt_mars_rover/
git submodule update
```

