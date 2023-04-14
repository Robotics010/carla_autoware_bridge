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

Currently some problems might occur if you are using galactic docker version (such as [galactic or foxy rviz error](https://github.com/ros2/rviz/issues/753)). If so, use source installation instead.

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

Clone [`carla-autoware-bridge`](https://github.com/Robotics010/carla_autoware_bridge) to your workspace and build it with the following commands:

```
source /opt/ros/galactic/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Step 5. Temporary workarounds

Since `carla_autoware_bridge` is still a work in progress solution, then not all features are working out of box or working with a certain performance.

Currently you need to do the following as well to get the autoware stack functioning:

1. Disable planning `surround_obstacle_checker`
   * because of lidar sensor placement and that lidar crop filters are not adjusted finely for the vehicle, so it results in points from th vehicle recognized as an obstacle points
1. Move out `shift_decider`, `vehicle_cmd_gate` and `operation_mode_transition_manager` nodes from `control_container`, but only if you have a problem starting them

### Disable `surround_obstacle_checker`

Go to `src/universe/autoware.universe/launch/tier4_planning_launch/launch/scenario_planning/lane_driving.launch.xml` and add `use_surround_obstacle_check` launch argument as following:

```
    <!-- motion planning module -->
    <group>
      <push-ros-namespace namespace="motion_planning"/>
      <include file="$(find-pkg-share tier4_planning_launch)/launch/scenario_planning/lane_driving/motion_planning/motion_planning.launch.py">
        <arg name="vehicle_info_param_file" value="$(var vehicle_info_param_file)"/>
        <arg name="tier4_planning_launch_param_path" value="$(var tier4_planning_launch_param_path)"/>
        <arg name="use_multithread" value="true"/>
        <arg name="use_surround_obstacle_check" value="false"/>
      </include>
    </group>
```

### Move some nodes from `control_container`

This workaround is only needed if you have a problem starting `shift_decider`, `vehicle_cmd_gate` and `operation_mode_transition_manager`.

Go to `src/universe/autoware.universe/launch/tier4_control_launch/launch/control.launch.py` and move out `shift_decider`, `vehicle_cmd_gate` and `operation_mode_transition_manager` nodes from `control_container` by making the following changes:

```
    shift_decider_loader = LoadComposableNodes(
        composable_node_descriptions=[shift_decider_component],
        target_container="/control/control_container",
    )

    vehicle_cmd_gate_loader = LoadComposableNodes(
        composable_node_descriptions=[vehicle_cmd_gate_component],
        target_container="/control/control_container",
    )

    operation_mode_transition_manager_loader = LoadComposableNodes(
        composable_node_descriptions=[operation_mode_transition_manager_component],
        target_container="/control/control_container",
    )

    container = ComposableNodeContainer(
        name="control_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            controller_component,
            lane_departure_component,
            # shift_decider_component,
            # vehicle_cmd_gate_component,
            # operation_mode_transition_manager_component,
        ],
    )

    group = GroupAction(
        [
            PushRosNamespace("control"),
            container,
            external_cmd_selector_loader,
            external_cmd_converter_loader,
            obstacle_collision_checker_loader,
            shift_decider_loader,
            vehicle_cmd_gate_loader,
            operation_mode_transition_manager_loader,
        ]
    )
```

## Step 6. Launch ad hoc simulation

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

And finally launch autoware software stack with the following commands:

```
source /opt/ros/galactic/setup.bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch e2e_simulator.launch.xml map_path:=$HOME/autoware_map/carla-town-1 vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

where `~/autoware` is a path to your workspace with cloned Autoware Universe installed.