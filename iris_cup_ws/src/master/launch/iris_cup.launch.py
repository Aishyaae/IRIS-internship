import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Cari path otomatis ke folder 'base_station' di root workspace
    ws_dir = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '../../../../..')
    )
    base_station_dir = os.path.join(ws_dir, 'base_station')

    # 2. Jalankan HTTP Server untuk Base Station (port 8000)
    base_station_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000'],
        cwd=base_station_dir,
        output='screen',
    )

    # 3. Jalankan ROS Bridge WebSocket (port 9090)
    rosbridge_dir = get_package_share_directory('rosbridge_server')
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml'
            )
        )
    )

    # 4. Daftarkan semua Node C++ (pastikan nama executable sesuai CMakeLists.txt)
    vision_node = Node(
        package='vision',
        executable='vision_node',
        name='vision_node',
        output='screen',
    )

    kinematics_node = Node(
        package='master',
        executable='kinematika_node',
        name='kinematika_node',
        output='screen',
    )

    udp_sender_node = Node(
        package='communication',
        executable='udp_sender_node',
        name='udp_sender_node',
        output='screen',
    )

    udp_receiver_node = Node(
        package='communication',
        executable='udp_receiver_node',
        name='udp_receiver_node',
        output='screen',
    )

    # 5. Eksekusi semua bersamaan
    return LaunchDescription([
        base_station_server,
        rosbridge_launch,
        vision_node,
        kinematics_node,
        udp_sender_node,
        udp_receiver_node,
    ])