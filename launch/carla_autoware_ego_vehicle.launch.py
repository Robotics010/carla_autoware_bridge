import os

import launch
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='objects_definition_file',
            default_value=get_package_share_directory(
                'carla_autoware_spawn') + '/config/objects.json'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_spawn_objects'), 'carla_example_ego_vehicle.launch.py')
            ),
            launch_arguments={
                'objects_definition_file': launch.substitutions.LaunchConfiguration('objects_definition_file'),
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
