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

    # Lê o URDF para passar como string no parâmetro robot_description
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # CORREÇÃO: Passando a string do URDF para o parâmetro correto
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

    # Inicia o Gazebo Sim (Mundo Vazio)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'], 
        output='screen'
    )

    # Pontes (Bridges) - Adicionei CMD_VEL, SCAN e CAMERA
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        output='screen'
    )

    # Ponte para Joint States (conecta Gazebo -> ROS 2)
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/empty/model/r2d2/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'],
        output='screen'
    )

    # Joint State Publisher (usa os dados da ponte para publicar TFs das rodas)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/joint_states', '/world/empty/model/r2d2/joint_state')]
    )

    # Spawna o robô
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'r2d2', '-topic', 'robot_description', '-z', '0.5'],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        bridge,
        robot_state_publisher_node,
        joint_state_bridge,
        joint_state_publisher_node,
        spawn_entity,
        rviz
    ])