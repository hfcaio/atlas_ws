# Tudo num comando: sobe Gazebo+SITL+MAVROS (atlas_bringup) e, depois de conectar,
# publica o alvo falso e roda a missao.
#
#   ros2 launch atlas_mission mission_sim.launch.py               # copter + missao icarus
#   ros2 launch atlas_mission mission_sim.launch.py vehicle:=copter gui:=true
#
# Para SITL rapido use SPEEDUP no ambiente. Se quiser ver o MAVProxy/map, use
# `ap-sim` + `ros2 launch atlas_mission mission.launch.py` em vez deste.
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vehicle = LaunchConfiguration("vehicle")
    gui = LaunchConfiguration("gui")
    start_delay = LaunchConfiguration("mission_start_delay")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("atlas_bringup"), "launch", "sim.launch.py"]),
        ]),
        launch_arguments={"vehicle": vehicle, "gui": gui, "mavros": "true"}.items(),
    )

    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("atlas_mission"), "launch", "mission.launch.py"]),
        ]),
        launch_arguments={
            "target_lat": LaunchConfiguration("target_lat"),
            "target_lon": LaunchConfiguration("target_lon"),
            "target_alt": LaunchConfiguration("target_alt"),
        }.items(),
    )
    # espera Gazebo+SITL+MAVROS conectarem antes de arrancar a missao
    mission_timer = TimerAction(period=start_delay, actions=[mission])

    return LaunchDescription([
        DeclareLaunchArgument("vehicle", default_value="copter"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("mission_start_delay", default_value="30.0",
                              description="segundos ate a missao arrancar (esperar MAVROS/GPS)"),
        DeclareLaunchArgument("target_lat", default_value="-35.3605622"),
        DeclareLaunchArgument("target_lon", default_value="149.1652375"),
        DeclareLaunchArgument("target_alt", default_value="603.0"),
        sim,
        mission_timer,
    ])
