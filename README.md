# ros-esm-dependencies-generator
ROS ESM missing dependencies sources generator

This snap helps you create a .rosinstall file
that lists all the dependencies not present in
your workspace nor in ROS ESM.

More on ESM at: https://ubuntu.com/robotics/ros-esm.

This program is meant to work with ROS 2 and ROS.

## Installation

Build the snap:

```
SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS=1 snapcraft
```

Install the snap

```
snap install ros-esm-dependencies-generator_*.snap --dangerous
```

## Usage

```
usage: ros-esm-dependencies-generator [-h] --rosdistro {noetic,foxy,humble,jazzy}
                                      [--depend-types DEPEND_TYPES [DEPEND_TYPES ...]] -o
                                      OUTPUT_FILE -s SOURCE

Generate a rosinstall with all the recursive build (buid, buildtool, build_export,
buildtool_export), exec (exec, run) or test dependencies

options:
  -h, --help            show this help message and exit
  --rosdistro {noetic,foxy,humble,jazzy}
                        The ROS distro to evaluate.
  --depend-types DEPEND_TYPES [DEPEND_TYPES ...]
                        A list of the depend types to list.
  -o OUTPUT_FILE, --output-file OUTPUT_FILE
                        The .rosinstall file to write.
  -s SOURCE, --source SOURCE
                        Source directory to look for packages.
```

### Example

In a ROS 2 ESM foxy environment with a workspace located in `ros-ws/src`:

```
ros-esm-dependencies-generator --rosdistro foxy -o dependencies.rosinstall --source ros-ws/src/
```

We can now pull, build and sources our dependencies:

```
mkdir -p deps-ws/src
vcs import --shallow deps-ws/src < ~/deps.rosinstall
cd deps-ws
rosdep install --ignore-src --from-paths src --default-yes
colcon build
. install/setup.bash
```

We can now build our workspace:
```
cd ros-ws
rosdep install --ignore-src --from-paths src --default-yes
colcon build
```
