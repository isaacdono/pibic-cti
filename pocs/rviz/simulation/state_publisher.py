from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')

        # --- Parâmetros (Declarados primeiro) ---
        self.declare_parameter('enable_spinning', True)

        # --- Publishers e QOS ---
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        # --- Estado do Robô (agora são atributos da classe) ---
        self.degree = pi / 180.0
        self.tilt = 0.
        self.tinc = self.degree
        self.swivel = 0.
        self.angle = 0.
        self.height = 0.
        self.hinc = 0.005

        # --- Mensagens (reutilizáveis) ---
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'axis'
        self.joint_state = JointState()

        # --- O Timer (Substitui o while loop) ---
        # Chama a função 'timer_callback' 30 vezes por segundo (1/30)
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

    def timer_callback(self):
        """
        Esta função é chamada 30x por segundo pelo timer.
        O rclpy.spin() no main() cuida dos parâmetros.
        """
        
        # Lê o parâmetro
        is_spinning = self.get_parameter('enable_spinning').value

        # update joint_state
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = ['swivel', 'tilt', 'periscope']
        self.joint_state.position = [self.swivel, self.tilt, self.height]

        # update transform
        self.odom_trans.header.stamp = now.to_msg()
        self.odom_trans.transform.translation.x = cos(self.angle)*2
        self.odom_trans.transform.translation.y = sin(self.angle)*2
        self.odom_trans.transform.translation.z = 0.7
        self.odom_trans.transform.rotation = \
            euler_to_quaternion(0, 0, self.angle + pi/2) # roll,pitch,yaw

        # send the joint state and transform
        self.joint_pub.publish(self.joint_state)
        self.broadcaster.sendTransform(self.odom_trans)

        # Create new robot state
        self.tilt += self.tinc
        if self.tilt < -0.5 or self.tilt > 0.0:
            self.tinc *= -1
        self.height += self.hinc
        if self.height > 0.2 or self.height < 0.0:
            self.hinc *= -1
        
        # Só gira se o parâmetro for True
        if is_spinning:
            self.swivel += self.degree
            self.angle += self.degree/4

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    # --- O Padrão Correto ---
    rclpy.init(args=args)
    
    node = StatePublisher()
    
    try:
        # rclpy.spin() mantém o nó vivo e responsivo
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()