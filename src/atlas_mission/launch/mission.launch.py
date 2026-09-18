# Roda uma missao em SITL, assumindo que a simulacao (Gazebo+SITL+MAVROS) JA esta no ar
# (ex. via `ap-sim copter` ou `ros2 launch atlas_bringup sim.launch.py`).
#
#   ros2 launch atlas_mission mission.launch.py                 # missao icarus + alvo falso
#   ros2 launch atlas_mission mission.launch.py target_lat:=-35.3605
#
# Publica o alvo falso (fake_target) e, apos um curto atraso, arranca a missao.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = LaunchConfiguration("mission_pkg")
    exe = LaunchConfiguration("mission_exe")

    fake_target = Node(
        package="atlas_mission",
        executable="fake_target",
        output="screen",
        parameters=[{
            "latitude": LaunchConfiguration("target_lat"),
            "longitude": LaunchConfiguration("target_lon"),
            "altitude": LaunchConfiguration("target_alt"),
        }],
    )

    mission = Node(
        package=pkg,
        executable=exe,
        output="screen",
        parameters=[{
            "auto_confirm": LaunchConfiguration("auto_confirm"),
        }],
    )
    # da' tempo do fake_target comecar a publicar antes da missao subscrever
    mission_timer = TimerAction(period=3.0, actions=[mission])

    return LaunchDescription([
        DeclareLaunchArgument("mission_pkg", default_value="icarus_tests"),
        DeclareLaunchArgument("mission_exe", default_value="mission"),
        DeclareLaunchArgument("target_lat", default_value="-35.3605622"),
        DeclareLaunchArgument("target_lon", default_value="149.1652375"),
        DeclareLaunchArgument("target_alt", default_value="603.0"),
        DeclareLaunchArgument("auto_confirm", default_value="true"),
        fake_target,
        mission_timer,
    ])
