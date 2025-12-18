import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'simulation'
    
    # Arquivos
    world_file_name = 'tugbot_warehouse.sdf'
    urdf_file_name = 'tugbot.urdf.xml' # Certifique-se de ter criado este arquivo (Dummy URDF)
    
    pkg_share = get_package_share_directory(package_name)
    world_path = os.path.join(pkg_share, world_file_name)
    urdf_path = os.path.join(pkg_share, urdf_file_name)

    # Lê o URDF para o Robot State Publisher
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # # 1. Gazebo
    # gazebo = ExecuteProcess(
    #     cmd=['gz', 'sim', '-r', world_path],
    #     output='screen'
    # )

    # 2. Ponte (Bridge) - REVERTIDA PARA O TÓPICO ESPECÍFICO
    # Isso garante que pegamos a movimentação do robô
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/tugbot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/tugbot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/tugbot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            
            # CORREÇÃO AQUI: Usando o caminho completo do sensor OMNI
            '/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            
            # Câmera Frontal (RGB)
            '/world/world_demo/model/tugbot/link/camera_front/sensor/color/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # Profundidade (Depth)
            '/world/world_demo/model/tugbot/link/camera_front/sensor/depth/depth_image@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        remappings=[
            ('/model/tugbot/cmd_vel', '/cmd_vel'),
            ('/model/tugbot/odometry', '/odom'),
            ('/model/tugbot/tf', '/tf'),
            # Redireciona o tópico longo do Gazebo para o '/scan' padrão do ROS
            ('/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan', '/scan'),
            ('/world/world_demo/model/tugbot/link/camera_front/sensor/color/image', '/camera/color/image_raw'),
            ('/world/world_demo/model/tugbot/link/camera_front/sensor/depth/depth_image', '/camera/depth/image_raw')
        ],
        output='screen'
    )

    # 3. Robot State Publisher (Visual)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc
        }]
    )

    # 4. TF STITCHING (A Costura)
    
    # ELO 1: Conecta o 'odom' padrão do ROS ao início da odometria do Tugbot
    tf_odom_connect = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'tugbot/odom'],
        output='screen'
    )

    # ELO 2: Conecta o corpo físico do Tugbot (simulação) ao corpo visual (URDF)
    # Isso faz a caixa laranja "grudar" na simulação
    tf_body_connect = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'tugbot/base_link'],
        output='screen'
    )
    
    # Conecta o frame do sensor Omni ao robô
    tf_laser_connect = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # Argumentos: x y z yaw pitch roll parent child
        # O child frame DEVE bater com o que o Gazebo manda.
        # Geralmente é "tugbot/scan_omni/scan_omni" ou "tugbot/scan_omni"
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'tugbot/scan_omni/scan_omni'],
        output='screen'
    )

    tf_camera_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # Ajuste a posição (x, y, z) conforme o modelo real
        arguments=['0.4', '0', '0.3', '0', '0', '0', 'base_link', 'tugbot/camera_front/color'],
        output='screen'
    )

    # 5. RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'r2d2.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        # gazebo,
        bridge,
        robot_state_publisher_node,
        tf_odom_connect,
        tf_body_connect,
        tf_laser_connect,
        tf_camera_fix,
        rviz
    ])