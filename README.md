# uwrt_mars_rover

![CI](https://github.com/uwrobotics/uwrt_mars_rover/workflows/CI/badge.svg)

**NOTE: Some of the info in this README is out of date and WIP because of our transition to ROS2. Feel free to submit PRs to update.**
## Repository Setup

To get started with development, clone this repo into the `src` directory of your `catkin` workspace. Use `rosinstall`
and `rosdep` to fetch any source or binary dependencies.

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

**Note:** If there have been certain changes to the workspace (ex. if a refactor moved one of the metapackage deps to a
different folder), you may need to delete the existing `.rosinstall` that's found in `<catkin workspace location>/src`
before you run the above commands.

## Adding Dependencies

For ease of setup on the Jetson, we should always try to use prebuilt binaries of dependent ROS packages, instead of
including their sources. This means that if a package you write is dependent on a 3rd party package, you should just
declare that dependency in the `CMakelists.txt` and `package.xml`. `rosdep` will take care of resolving the
dependencies.

In the event that you cannot use the binaries (ex. we rely on a feature that has not been released), the package source
code should be cloned outside of our metapackage, so that our CI doesn't run linting/formatting checks on it. To do
this, declare the source dependency in `upstream_dependencies.rosinstall`. If using unreleased features(ie. cloning 3rd
party master branch), please pin the rosinstall entry to a commit hash, rather than the branch. `rosinstall` will take
care of cloning the source dependencies declared in `upstream_dependencies.rosinstall`.

If you need to declare dependencies that are not ROS packages, typically you can declare them as system dependencies in
a `package.xml`. If it is an unreleased source dependency, (ex. the roboteq c++ driver we wrote), declare it in the
`metapackage_dependencies.rosinstall`. Source code modules are still subject to the clangformat and clangtidy checks
because they should only consist of code the team has written.

## Launching the Rover!

The main entry point of the rover is `rover.launch` in the `uwrt_mars_rover_bringup` package.

To list the required and optional arguements of `rover.launch`:

```
roslaunch uwrt_mars_rover_bringup rover.launch --ros-args
```

Ex. To launch the rover ros stack in drivetrain only mode:

```
roslaunch uwrt_mars_rover_bringup rover.launch control_mode:=drivetrain_only 
```

### Hard Realtime Loop

`rover.launch` makes the assumption that you are running a linux kernel will realtime capabilities (like our NVIDIA
Jetsons where we have enabled the PREEMPT_RT kernel patch). If this assumption is not true(ex. you are running nodes on
your own development machines), you must run `rover.launch` with `use_realtime_kernel:=false`

On machines with a realtime kernel, make sure you have permissions to lock unlimited memory for processes.

To check:

```
ulimit -l
```

If it does not return `unlimited`, you must adjust your memory locking permissions:

1. Open `/etc/security/limits.conf` as sudo in a text editor (ex. gedit, nano, vim)
2. Add the following line:
   ```
   <your username>    -   memlock   -1
   ```
3. Reboot your computer
4. Check that `ulimit -l` returns `unlimited`

## Git Workflow

### Issues

Before writing code, assign yourself the issue that pertains to the code you are going to write. If there is no GitHub
issue created yet, create one for yourself! Try to keep all discussions related to the task within the issue.

### Branch Naming

To be able to quickly see work done by members, you should name you're branches with the following scheme:
`user/<username>/#<issue number>/<issue name>`

For example:
`user/wmmc88/#2/update-readme-with-git-workflow`

### Pull Requests

To merge code into master, you must open a pull request. Branches must be up to date with the latest master to be able
to merge. Choose people you think are familiar with your task to do code reviews of your code. After your PR is merged,
make sure you close the associated issue and delete the branch. Github will usually delete the branch automatically and
you can automatically close the issue by mentioning `closes #<issue number>` in the PR or in one of the commits on the
branch.

If you have code that's not ready to merge, but you'd still like people thoughts on it, open a
[draft pull request](https://github.blog/2019-02-14-introducing-draft-pull-requests/).

## Adding New Ros Packages

When you add new ROS packages:

1. Add the package name to `PACAKGES_TO_TEST` string in the [Github Actions Config](./.github/workflows/ci.yaml)

2. In the package `CMakeLists.txt`, ensure you have all the linters enabled for tests:

   ```CMake
   if (BUILD_TESTING)
   # generate compile_commands.json for clang-tidy
   set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

   # cppcheck
   find_package(ament_cmake_cppcheck REQUIRED)
   ament_cppcheck()

   # clang-format
   find_package(ament_cmake_clang_format REQUIRED)
   ament_clang_format()

   # clang-tidy
   find_package(ament_cmake_clang_tidy REQUIRED)
   ament_clang_tidy(${CMAKE_BINARY_DIR})

   # flake8
   find_package(ament_cmake_flake8 REQUIRED)
   ament_flake8()

   # xmllint
   find_package(ament_cmake_xmllint REQUIRED)
   ament_xmllint()
   endif ()
   ```

3. In the `package.xml`, ensure you have all all the linter dependencies declared alongside any package dependencies:

   ```XML
   <test_depend>ament_cmake_cppcheck</test_depend>
   <test_depend>ament_cmake_clang_format</test_depend>
   <test_depend>ament_cmake_clang_tidy</test_depend>
   <test_depend>ament_cmake_flake8</test_depend>
   <test_depend>ament_cmake_xmllint</test_depend>
   ```

## Running CI Pipeline Locally

The entire Github Actions Workflow can be run locally using [act](https://github.com/nektos/act). This is particularly
useful if you're contributing to improving our CI pipeline or if you just want faster feedback on your code without
needing to push.

### To install `act` on Ubuntu 20.04:
These instructions install the latest version of `act` availble on master. Currently, `act` seems to be in very active
development and a number of bugs in running our workflow was fixed by using the latest builds. But be warned that there 
may be some bugs etc. Be sure to check the active issues on their repo if you run into issues.

1. Install `docker`:

   Follow official instructions: https://docs.docker.com/engine/install/

2. Install latest `Go`(required 1.16+):
   ```
   # From https://github.com/golang/go/wiki/Ubuntu
   sudo add-apt-repository ppa:longsleep/golang-backports
   sudo apt update
   sudo apt install golang-go
   ```
   
3. Install latest `act`:
   ```
   go install github.com/nektos/act@master
   ```

4. Add GOPATH to PATH: 
   ```
   echo "export PATH=${HOME}/go/bin\${PATH:+:\${PATH}}" >> $HOME/.bashrc
   ```
   
5. Create a `.actrc` config file:
   ```
   touch $HOME/.actrc
   echo "-P ubuntu-latest=catthehacker/ubuntu:full-20.04" >> $HOME/.actrc
   echo "-s GITHUB_TOKEN=<A NEWLY GENERATED GITHUB PERSONAL ACCESS TOKEN>" >> $HOME/.actrc
   ```
   More info on generating Github Personal Access Tokens [available here](https://docs.github.com/en/github/authenticating-to-github/keeping-your-account-and-data-secure/creating-a-personal-access-token)
   
   **Warning: This image listed above in the `-P` flag will download a >20GB docker image that is >60GB when
   uncompressed. This is just how big the ubuntu installs running on Github's free runner machines are and its required 
   to use them to get as close as possible to the exact environment our CI pipelines run in. You may try some of the
   images listed [here](https://github.com/nektos/act#runners), but may run into issues that are your responsiblity to 
   resolve.**

### Running Github Workflow Jobs:
```
cd <repo root>

# Run whole workflow
act

# Run a specific job
act -j <job name>

# Run with verbose output
act -v
```
More advanced usage available [here]<https://github.com/nektos/act#commands>

Containers will stay open after you run `act` and you can attach to them to see their workspaces.
```
docker ps # Lists all active containers

docker exec -it <container name> bash # Open a bash prompt for that container

docker kill $(docker ps -q) # Kills all containers
```
