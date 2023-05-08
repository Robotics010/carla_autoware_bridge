# Troubleshooting

This section describes how resolve some of the errors while configuring and launching Autoware Universe with CARLA.

## Invalid version: '0.23ubuntu1' (package: distro-info)

If you get `Invalid version: '0.23ubuntu1' (package: distro-info)` exception while building your ros2 packages, you can fix it by installing `setuptools==58.3.0` with the following command:
```
python3 -m pip install --upgrade --user setuptools==58.3.0
```

More info is [here](https://stackoverflow.com/questions/75272737/error-invalid-version-0-23ubuntu1-package-distro-info).

## carla_ros_bridge time-out Error

If you get `Error: time-out of 2000ms while waiting for the simulator, make sure the simulator is ready and connected to localhost:2000`, then you need just increase timeout value.

## Control does not work because of missing nodes

If you encounter a problem with control not functioning it could be because of missing control node such as `shift_decider`, `vehicle_cmd_gate` and `operation_mode_transition_manager`. If so you can try moving them out from the `control_container` as a workaround. This workaround help you out only if you have a problem starting `shift_decider`, `vehicle_cmd_gate` and `operation_mode_transition_manager`.

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

## Launching rviz inside galactic docker

Currently some problems might occur if you are using galactic docker version (such as [galactic or foxy rviz error](https://github.com/ros2/rviz/issues/753)). If so, you can use source installation instead.
