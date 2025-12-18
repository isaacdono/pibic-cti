import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'simulation'
    urdf_file_name = 'r2d2.urdf.xml'
    rviz_file_name = 'r2d2.rviz'

    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, urdf_file_name)
    rviz_config_path = os.path.join(pkg_share, rviz_file_name)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 1. Robot State Publisher (Publica TFs estáticos e das juntas)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    # 2. Gazebo Sim
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    # 3. Spawna o robô
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'r2d2', '-topic', 'robot_description', '-z', '0.5'],
        output='screen'
    )

    # 4. Ponte ROS-Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Relógio
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Comandos de velocidade
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Laser Scan
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Câmera
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # Odometria (TF)
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Estados das Juntas (Para as rodas)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        output='screen'
    )

    # 5. RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # CORREÇÃO DEFINITIVA DE FRAME (GAMBIARRA PADRÃO)
    # Diz ao ROS que o frame estranho do Gazebo é, na verdade, o nosso laser_link
    # Args: x y z roll pitch yaw parent_frame child_frame
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'laser_link', 'r2d2/base_footprint/gpu_lidar'],
        output='screen'
    )

    # Faça o mesmo para a câmera se necessário
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'r2d2/base_footprint/camera'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        bridge,
        robot_state_publisher_node,
        spawn_entity,
        rviz,
        lidar_tf,
        camera_tf
    ])