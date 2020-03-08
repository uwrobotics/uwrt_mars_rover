# uwrt_mars_rover

[![Build Status](https://travis-ci.com/uwrobotics/uwrt_mars_rover.svg?branch=master)](https://travis-ci.com/uwrobotics/uwrt_mars_rover)

## Repository Setup

To get started with development, clone this repo into the `src` directory of your `catkin` workspace. Use `rosinstall` and `rosdep` to fetch any source or binary dependencies.

```bash
# Ensure rosinstall and rosdep are installed and up to date
sudo apt install python-rosinstall python-rosdep
sudo rosdep init
rosdep update

# Clone repo
cd <catkin workspace location>/src
git clone https://github.com/uwrobotics/uwrt_mars_rover.git

# Install upstream source dependencies
rosinstall --catkin . uwrt_mars_rover/upstream_dependencies.rosinstall

# Install metapackage source dependencies
rosinstall --catkin . uwrt_mars_rover/metapackage_dependencies.rosinstall

# Install all binary dependencies
rosdep install --from-paths . --ignore-src -r -y
```

If you're setting up the repository on the Nvidia Jetson, you also need to install the arm64 upstream dependencies:
```bash
# Install upstream source dependencies for arm64
rosinstall --catkin . uwrt_mars_rover/arm64_upstream_dependencies.rosinstall
```

## Updating Dependencies
Commands to update dependencies:
```bash
cd <catkin workspace location>/src
rosinstall --catkin . uwrt_mars_rover/upstream_dependencies.rosinstall uwrt_mars_rover/metapackage_dependencies.rosinstall
rosdep install --from-paths . --ignore-src -r -y
```


## Adding Dependencies
For ease of setup on the Jetson, we should always try to use prebuilt binaries of dependent ROS packages, instead of 
including their sources. This means that if a package you write is dependent on a 3rd party package, you should just 
declare that dependency in the `CMakelists.txt` and `package.xml`. `rosdep` will take care of resolving the 
dependencies. 

In the event that you cannot use the binaries (ex. we rely on a feature that has not been released), the package source 
code should be cloned outside of our metapackage, so that our CI doesn't run linting/formatting checks on it. To do 
this, declare the source dependency in `upstream_dependencies.rosinstall`. If using unreleased features(ie. cloning 3rd party master branch), 
please pin the rosinstall entry to a commit hash, rather than the branch. `rosinstall` will take care of cloning the 
source dependencies declared in `upstream_dependencies.rosinstall`. 

If you need to declare dependencies that are not ROS packages, typically you can declare them as system dependencies in
a `package.xml`. If it is an unreleased source dependency, (ex. the roboteq c++ driver we wrote), declare it in the 
`metapackage_dependencies.rosinstall`. Source code modules are still subject to the clangformat and clangtidy checks 
because they should only consist of code the team has written. 

## Git Workflow
### Issues
Before writing code, assign yourself the issue that pertains to the code you are going to write. If there is no GitHub issue created yet, create one for yourself! Try to keep all discussions related to the task within the issue.

### Branch Naming
To be able to quickly see work done by members, you should name you're branches with the following scheme:
`user/<username>/#<issue number>/<issue name>`

For example:
`user/wmmc88/#2/update-readme-with-git-workflow`

### Pull Requests
To merge code into master, you must open a pull request. Branches must be up to date with the latest master to be able to merge. Choose people you think are familiar with your task to do code reviews of your code. 
After your PR is merged, make sure you close the associated issue and delete the branch. Github will usually delete the branch automatically and you can automatically close the issue by mentioning `closes #<issue number>` in the PR or in one of the commits on the branch.

If you have code that's not ready to merge, but you'd still like people thoughts on it, open a [draft pull request](https://github.blog/2019-02-14-introducing-draft-pull-requests/).

## Running CI Stages Locally
### Strict Build
CI builds with extra cmake flags that can help catch common errors in code. Follow the instructions [here](https://github.com/uwrobotics/dev_tools) to install our `catkin_tools` profiles. It is highly encouraged for you to use this as your default `catkin_tools` profile when developing code. To build using our stricter build flags:
```
catkin clean -y
catkin profile set strict
catkin build 
```

### Clang Tidy
Follow the instructions [here](https://github.com/uwrobotics/dev_tools) to install our `catkin_tools` profiles. You will also need to install `clang-tidy-9`
```
sudo apt install clang-tidy-9
```
To build and invoke clang-tidy:
```
catkin clean -y
catkin profile set clang-tidy
catkin build 
```

### Release Build
The release build profile is a combination of stricter build flags, clang-tidy and `-O3` optimizations. This profile also installs the packages in an isolated install space. This is the profile you should use to build code for execution on the rover.
Assuming you've installed our `catkin_tools` profiles and `clang-tidy-9`, to build in release mode:
```
catkin clean -y
catkin profile set release
catkin build 
```

### Clang Format
You can install clang-format and run it on the files themselves, but the recommended way is to install a clang-format plugin for whatever IDE you're using. They typically automatically find and use the `.clang-format` file in our repo. 

### Catkin Lint
Catkin Lint ensures that the catkin-specific files are configured correctly. This includes the `package.xml`, `CMakeLists.txt` and more. To install `catkin_lint`:
```
sudo apt install python-catkin-lint
```

To run catkin lint:
```
cd <catkin_ws location>/src
catkin_lint --strict -W2 uwrt_mars_rover
```