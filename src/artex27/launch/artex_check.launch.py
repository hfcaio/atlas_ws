# Pre-voo automatizado: le missao, geofence e parametros do FCU e reporta
# divergencias face ao que este pacote espera.
#
#   ros2 launch artex27 artex_check.launch.py
#   ros2 launch artex27 artex_check.launch.py manual_land:=true
#   ros2 launch artex27 artex_check.launch.py check_params:=false
#
# Nao escreve nada no FCU.
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Lanca o no `artex_check` com o `config/artex27.yaml` do pacote."""
    config = os.path.join(
        get_package_share_directory('artex27'), 'config', 'artex27.yaml')

    check = Node(
        package='artex27',
        executable='artex_check',
        output='screen',
        parameters=[config, {
            'vehicle': LaunchConfiguration('vehicle'),
            'manual_land': LaunchConfiguration('manual_land'),
            'check_params': LaunchConfiguration('check_params'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('vehicle', default_value='mavros',
                              description='prefixo MAVROS do veiculo'),
        DeclareLaunchArgument('manual_land', default_value='false',
                              description='qual das duas variantes se espera no FCU'),
        DeclareLaunchArgument('check_params', default_value='true',
                              description='comparar tambem os parametros do FCU'),
        check,
    ])
