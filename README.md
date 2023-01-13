# uwrt_mars_rover

![CI](https://github.com/uwrobotics/uwrt_mars_rover/workflows/CI/badge.svg)

## Repository Setup

### Using Containerized Development (Windows, non-Ubuntu 20.04)
To get started with development:
1. Create a separate workspace (an empty folder) for your UWRT development: we recommend calling this `uwrt_ws`.
2. Create an `src` directory inside of your workspace (e.g: `uwrt_ws/src`).
3. Clone this repository into the `src` directory of your workspace. `git clone github.com/uwrobotics/uwrt_mars_rover.git` via Git Bash
4. **Navigate inside the folder from the command line** and use Docker to build the `Dockerfile.devcontainer` Dockerfile from your current directory, using the `-t` flag to name your image: `sudo docker build -f Dockerfile.devcontainer -t uwrt-dev .`
5. Start the development container interactive. Before we do this, let's go through what each of the Docker flags we'll be using for `docker run` means:
   - `-v ~/folder1:/folder` shares `~/folder1` to your docker container at `/folder`.
   -  `-it` runs the Docker container interactively, meaning that as soon as the 
   - sharing your UWRT workspace folder to the Docker container via `sudo docker run --name=uwrt-galactic -it -v ~/uwrt_ws/:/uwrt_ws uwrt-dev`. Remember to change the `-v` argument to the correct mapping; it should be an absolute path to your UWRT workspace on your local machine, then a colon, then `/uwrt_ws` on the right side of the colon; this tells Docker which folder to read from for the shared folder, and where to put the shared folder in the container.
7. Now, we need to set up the workspace inside the Docker container
   
### Downloading the Repository and Getting Dependencies
1. Navigate to your ROS2 workspace in the terminal (e.g: `dev_ws` or `uwrt_ws`)
2. Create a `src` directory if you haven't already: `mkdir -p dev_ws/src`.
3. Navigate inside of your `src` directory: `cd src`
4. Clone the repository into your `src` directory: `git clone git@github.com:uwrobotics/uwrt_mars_rover.git`
5. Update your system before continuing: `sudo apt update -y --no-install-recommends && sudo apt dist-upgrade -y`
6. Install `rosdep`, the ROS dependency manager: `sudo apt install -y python3-rosdep`
7. Download the repository's upstream dependencies: `vcs import --input uwrt_mars_rover/common_upstream_dependencies.repos`
8. Navigate back to the root of your workspace, and install all dependencies for your ROS packages: `RUN rosdep install --from-paths src -y --ignore-src`

You can re-navigate to the root of your workspace at any time and rerun #8 to update your ROS packages' dependencies.


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

1. Add the package name to `PACKAGES_TO_TEST` string in the [Github Actions Config](./.github/workflows/ci.yaml)

2. In the package `CMakeLists.txt`, ensure you have all the linters enabled for tests. Omit irrelevant linters:

   ```CMake
   if (BUILD_TESTING)
      # Force generation of compile_commands.json for clang-tidy
      set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

      # clang-format
      find_package(ament_cmake_clang_format REQUIRED)
      ament_clang_format(
      CONFIG_FILE ${CMAKE_SOURCE_DIR}/../../.clang-format
      )

      # clang-tidy
      find_package(ament_cmake_clang_tidy REQUIRED)
      ament_clang_tidy(
      ${CMAKE_BINARY_DIR}
      CONFIG_FILE ${CMAKE_SOURCE_DIR}/../../.clang-tidy
      )
   
      # cppcheck
      find_package(ament_cmake_cppcheck REQUIRED)
      ament_cppcheck()

      # flake8
      find_package(ament_cmake_flake8 REQUIRED)
      ament_flake8(
      CONFIG_FILE ${CMAKE_SOURCE_DIR}/../../.flake8
      )

      # xmllint
      find_package(ament_cmake_xmllint REQUIRED)
      ament_xmllint()
   endif ()
   ```

3. In the `package.xml`, ensure you have all all the linter dependencies declared alongside any package dependencies:

   ```XML
   <test_depend>ament_cmake_clang_format</test_depend>
   <test_depend>ament_cmake_clang_tidy</test_depend>
   <test_depend>ament_cmake_cppcheck</test_depend>
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
