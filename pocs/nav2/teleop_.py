import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Configurações de Velocidade
MAX_LIN_VEL = 0.5  # Metros por segundo
MAX_ANG_VEL = 1.0  # Radianos por segundo

# Mapeamento de Teclas
# Tecla: (linear_x, angular_z)
MOVE_BINDINGS = {
    'w': (1, 0),   # Frente
    's': (-1, 0),  # Trás
    'a': (0, 1),   # Esquerda (Giro positivo)
    'd': (0, -1),  # Direita (Giro negativo)
}

# Teclas de Parada
STOP_BINDINGS = {' ', 'k'} # Espaço ou 'k' param o robô

msg = """
---------------------------
Controle do Robô (WASD)
---------------------------
   w
 a s d

Espaço ou 'k': Parar
CTRL-C: Sair
---------------------------
"""

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_wasd')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Nó de Teleoperação Iniciado")

def getKey(settings):
    """Lê uma única tecla do terminal sem esperar 'Enter'."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    # Salva as configurações originais do terminal
    settings = termios.tcgetattr(sys.stdin)

    target_linear_vel = 0.0
    target_angular_vel = 0.0
    
    try:
        print(msg)
        while True:
            key = getKey(settings)
            
            # Se uma tecla de movimento for pressionada
            if key in MOVE_BINDINGS.keys():
                lin_dir = MOVE_BINDINGS[key][0]
                ang_dir = MOVE_BINDINGS[key][1]
                
                target_linear_vel = lin_dir * MAX_LIN_VEL
                target_angular_vel = ang_dir * MAX_ANG_VEL
                
                print(f"Comando: Lin={target_linear_vel} | Ang={target_angular_vel}")

            # Se tecla de parada
            elif key in STOP_BINDINGS:
                target_linear_vel = 0.0
                target_angular_vel = 0.0
                print("PARANDO ROBÔ")

            # Ctrl+C para sair
            elif key == '\x03': 
                break

            # Publica a mensagem Twist
            twist = Twist()
            twist.linear.x = float(target_linear_vel)
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(target_angular_vel)
            
            node.publisher_.publish(twist)

    except Exception as e:
        print(e)

    finally:
        # Envia comando de parada antes de morrer
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.publisher_.publish(twist)

        # Restaura o terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()