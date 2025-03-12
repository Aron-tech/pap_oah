# `pap_oah_random` package

ROS 2 C++ package. [![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)

## Packages and build

It is assumed that the workspace is `~/ros2_ws/`.

### Clone the packages

```bash
cd ~/ros2_ws/src
```

```bash
git clone https://github.com/Aron-tech/pap_oah_random
```

### Build ROS 2 packages

```bash
cd ~/ros2_ws/
```

```bash
colcon build --packages-select pap_oah_random --symlink-install
```

### Source the setup file

```bash
source install/setup.bash
```

### Start turtlesim

```bash
ros2 run turtlesim turtlesim_node
```

### On a different terminal, run the houses node

```bash
ros2 run pap_oah_random houses [number_of_houses 1-3]
```

#### For example:
```bash
ros2 run pap_oah_random houses 2
```

