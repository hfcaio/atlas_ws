# Carrega a missao ARTEX 2000 m e a geofence no FCU.
#
# Assume que o MAVROS JA esta' no ar (ligado ao aviao ou ao SITL), por exemplo
# via `ros2 launch atlas_bringup sim.launch.py` ou `ap-sim plane`.
#
#   ros2 launch artex27 artex_mission.launch.py
#   ros2 launch artex27 artex_mission.launch.py manual_land:=true
#   ros2 launch artex27 artex_mission.launch.py approach_brg:=317.2   # QFU real
#   ros2 launch artex27 artex_mission.launch.py vehicle:=icarus
#
# NAO arma e NAO descola: a descolagem da missao ARTEX e' manual.
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Lanca o no `artex_mission` com o `config/artex27.yaml` do pacote."""
    config = os.path.join(
        get_package_share_directory('artex27'), 'config', 'artex27.yaml')

    mission = Node(
        package='artex27',
        executable='artex_mission',
        output='screen',
        parameters=[config, {
            'vehicle': LaunchConfiguration('vehicle'),
            'manual_land': LaunchConfiguration('manual_land'),
            'approach_brg': LaunchConfiguration('approach_brg'),
            'push_fence': LaunchConfiguration('push_fence'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('vehicle', default_value='mavros',
                              description='prefixo MAVROS do veiculo'),
        DeclareLaunchArgument('manual_land', default_value='false',
                              description='variante com LOITER_UNLIM para pouso manual'),
        DeclareLaunchArgument('approach_brg', default_value='-1.0',
                              description='rumo de aproximacao [graus]; -1 = geometrico (137,2)'),
        DeclareLaunchArgument('push_fence', default_value='true',
                              description='carregar tambem a geofence de inclusao'),
        mission,
    ])
