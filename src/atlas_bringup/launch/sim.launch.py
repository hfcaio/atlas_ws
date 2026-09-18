# Sobe a simulacao ATLAS: Gazebo + ArduPilot SITL + MAVROS.
#
#   ros2 launch atlas_bringup sim.launch.py               # copter, com GUI e MAVROS
#   ros2 launch atlas_bringup sim.launch.py vehicle:=vtol
#   ros2 launch atlas_bringup sim.launch.py gui:=false mavros:=false
#
# E' um wrapper dos scripts do ambiente (ap-gazebo / ap-sitl / ap-mavros), que ja
# tratam mundo, frame do SITL, MAVProxy (+ terreno) e portas. Para uso interativo
# com 3 paineis tmux, prefira o script `ap-sim <vehicle>`.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    vehicle = LaunchConfiguration("vehicle")
    gui = LaunchConfiguration("gui")
    mavros = LaunchConfiguration("mavros")
    wait_gazebo = LaunchConfiguration("wait_gazebo")
    mavros_delay = LaunchConfiguration("mavros_delay")

    # HEADLESS=1 no ap-gazebo quando gui:=false
    gz_headless = PythonExpression(["'0' if '", gui, "' == 'true' else '1'"])

    # Gazebo (mundo conforme o veiculo). GUI controlado por HEADLESS.
    gazebo = ExecuteProcess(
        cmd=["ap-gazebo", vehicle],
        additional_env={"HEADLESS": gz_headless},
        output="screen",
    )

    # SITL: MAVProxy headless (sem --map/--console) mas com --out para o MAVROS.
    # WAIT_GAZEBO da' tempo do Gazebo carregar o modelo antes do SITL ligar ao plugin.
    sitl = ExecuteProcess(
        cmd=["ap-sitl", vehicle],
        additional_env={"HEADLESS": "1", "WAIT_GAZEBO": "0"},
        output="screen",
    )
    sitl_timer = TimerAction(period=wait_gazebo, actions=[sitl])

    # MAVROS por cima (fcu_url default udp://:14550@ do ap-mavros).
    mavros_proc = ExecuteProcess(
        cmd=["ap-mavros"],
        output="screen",
        condition=IfCondition(mavros),
    )
    mavros_timer = TimerAction(period=mavros_delay, actions=[mavros_proc])

    return LaunchDescription([
        DeclareLaunchArgument("vehicle", default_value="copter",
                              description="copter | plane | vtol"),
        DeclareLaunchArgument("gui", default_value="true",
                              description="Gazebo com GUI (false = headless)"),
        DeclareLaunchArgument("mavros", default_value="true",
                              description="subir MAVROS por cima do SITL"),
        DeclareLaunchArgument("wait_gazebo", default_value="8.0",
                              description="segundos antes de arrancar o SITL"),
        DeclareLaunchArgument("mavros_delay", default_value="14.0",
                              description="segundos antes de arrancar o MAVROS"),
        gazebo,
        sitl_timer,
        mavros_timer,
    ])
