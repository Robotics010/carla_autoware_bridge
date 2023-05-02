This tutorial helps to setup and launch autoware simulation with CARLA simulator.

Warning! This is **Work in Progress** tutorial. Reports and improvement suggestions are very welcome.

## Requirements

* Ubuntu 20.04
* ROS2 Galactic

## Step 1. CARLA installation 

Install [CARLA server 0.9.12](https://carla.readthedocs.io/en/0.9.12/start_quickstart/#carla-installation) and [carla client 0.9.12](https://carla.readthedocs.io/en/0.9.12/start_quickstart/#carla-0912).

Currently `carla-ros-bridge` has the latest 0.9.12 tag, so it is recommended to use exact the same CARLA simulator and client.

Check the CARLA with [Running CARLA steps](https://carla.readthedocs.io/en/0.9.12/start_quickstart/#running-carla) if needed.

## Step 2. Autoware installation

Install [Autoware Universe](https://autowarefoundation.github.io/autoware-documentation/galactic/installation/autoware/source-installation/) from `galactic` branch.

## Step 3. carla-ros-bridge installation

Clone my fork of [`carla-ros-bridge`](https://github.com/Robotics010/ros-bridge) to your workspace. This fork has several changes:

* change imu frame_id to `tamagawa/imu_link`
* change lidar frame_id to `velodyne_top`
* add ring field to lidar messages

And build it with the following commands:

```
source /opt/ros/galactic/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Then download and prepare the map for CARLA town 1. For that clone [autoware-contents](https://bitbucket.org/carla-simulator/autoware-contents/src/master/), copy `maps/point_cloud_maps/Town01.pcd` and `maps/vector_maps/lanelet2/Town01.osm` to `~/autoware_map/carla-town-1/` and rename to `pointcloud_map.pcd` and `lanelet2_map.osm` accordingly.

## Step 4. carla-autoware-bridge installation

Clone [`carla-autoware-bridge`](https://github.com/Robotics010/carla_autoware_bridge) and [`carla_tesla_model3_description`](https://github.com/Robotics010/carla_tesla_model3_description.git) packages to your workspace and build it with the following commands:

```
source /opt/ros/galactic/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Step 5. Launch ad hoc simulation

Here is a list of launch commands to launch Autoware Universe ad hoc simulation with CARLA.

### Launch CARLA server:
```
cd to/carla/folder
./CarlaUE4.sh
```

where `to/carla/folder` is a folder path to your CARLA folder installed.

You might find these arguments useful while executing CARLA:
* `-prefernvidia` - use NVIDIA GPU for hardware acceleration
* `-carla-rpc-port=3000` - use other than 2000 default port for RPC service's port
* `-quality-level=Low` - use low quality level mode for a minimal video memory consumption

### Launch carla_ros_bridge and carla_autoware_bridge

In order to launch `carla_ros_bridge` and `carla_autoware_bridge`:
```
source /opt/ros/galactic/setup.bash
source ~/autoware/install/setup.bash
ros2 launch carla_autoware_bridge carla_ros_bridge.launch.py
```

where `~/autoware` is a path to your workspace with cloned `carla_ros_bridge` and `carla_autoware_bridge`.

Here you can add `port:=3000` argument to switch to a different CARLA port for it's RPC port.

### Spawn ego vehicle

You need to spawn a vehicle with it's sensors:

```
source /opt/ros/galactic/setup.bash
source ~/autoware/install/setup.bash
ros2 launch carla_autoware_bridge carla_autoware_ego_vehicle.launch.py
```

where `~/autoware` is a path to your workspace with cloned `carla_ros_bridge` and `carla_autoware_bridge`.

### (Optional) Launch manual control window

You can find it useful to launch and manual control window. It shows your ego vehicle from a third view in the CARLA world. And you can switch it to manual control from keyboard if it is necessary.

```
source /opt/ros/galactic/setup.bash
source ~/autoware/install/setup.bash
ros2 launch carla_manual_control carla_manual_control.launch.py
```

where `~/autoware` is a path to your workspace with cloned `carla_ros_bridge`.

### Launch Autoware Universe

And finally launch Autoware software stack with the following commands:

```
source /opt/ros/galactic/setup.bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch e2e_simulator.launch.xml map_path:=$HOME/autoware_map/carla-town-1 vehicle_model:=carla_tesla_model3 sensor_model:=sample_sensor_kit
```

where `~/autoware` is a path to your workspace with cloned Autoware Universe installed.

### Troubleshooting

Go to [Troubleshooting](troubleshooting.md) section in order to fix some known problems.
