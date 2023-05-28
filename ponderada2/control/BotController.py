from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

from Pose import Pose
from Rotation import Rotation
from MissionControl import MissionControl

# Máxima diferença de translação aceita para considerar que o robô chegou no ponto
MAX_DIFF = 0.1

# Define a classe BotController, que representa o nó de controle do robô
class BotController(Node):
    # Inicializa o nó com período de controle de 0.05s e uma fila vazia
    def __init__(self, control_period=0.05, mission_control=MissionControl()):
        super().__init__("bot_controller")

        self.initiated = False # Indica se a pose inicial já foi recebida
        
        self.setpoint = Pose() # Define o ponto de destino como 0
        self.current_pose = Pose() # Define a pose inicial como 0

        self.relative_vector_angle = Rotation() # Ângulo em relação ao eixo x do vetor relativo entre a pose atual e o destino; esse ângulo é somado com a rotação atual do robô para definir a rotação total necessária para o robô virar para o destino

        self.setpoint_rotation = Rotation() # Define a rotação de destino como 0
        self.setpoint_translation = 0.0 # Define a translação de destino, no eixo x do robô, como 0
        self.current_rotation = Rotation() # Define a rotação inicial como 0

        self.queue = mission_control # Define a fila de pontos a serem seguidos

        self.movement_origin = Pose() # Define a origem do movimento no eixo x do robô como 0

        # Cria um timer para controlar o robô
        self.control_timer = self.create_timer(
            timer_period_sec=control_period, 
            callback=self.control_callback
        )

        # Cria um assinante para receber a pose do robô
        self.subscription = self.create_subscription(
            msg_type=Odometry,
            topic="odom",
            callback=self.pose_callback,
            qos_profile=10
        )

        # Cria um publicador para enviar comandos de movimento para o robô
        self.publisher = self.create_publisher(
            msg_type=Twist, 
            topic="cmd_vel", 
            qos_profile=10
        )

    # Função de callback para controlar o movimento do robô
    def control_callback(self): 

        # Se a pose inicial ainda não foi recebida, não faz nada
        if not self.initiated:
            self.get_logger().info("Aguardando pose...")
            return
        
        # Cria mensagem de movimento
        msg = Twist()

        # Se o robô ainda não rotacionou o suficiente, rotaciona
        if not self.setpoint_rotation.rotated:

            # Checa se o robô já rotacionou o suficiente
            if self.current_rotation == self.setpoint_rotation:
                msg.angular.z = 0.0
                self.get_logger().info(f"O robô rodou o suficiente em {self.current_rotation}")
                self.setpoint_rotation.rotated = True # Indica que o robô já rotacionou o suficiente
            else:
                # Calcula o ângulo restante para rotacionar
                offset = self.setpoint_rotation.theta - self.current_rotation.theta
                # Se o módulo do ângulo restante for maior que 0.05, rotaciona
                if abs(offset) > 0.05:
                    msg.angular.z = 0.5 if offset > 0 else -0.5
        
        # Se o robô já rotacionou o suficiente, translada
        else:
            # Checa se o robô já transladou o suficiente
            if self.current_pose == self.setpoint:
                msg.linear.x = 0.0
                self.get_logger().info(f"O robô chegou ao destino")
                self.publisher.publish(msg) # Publica mensagem de movimento para garantir que o robô pare
                self.update_setpoint() 
            
            else:
                # Corrige o ângulo caso necessário
                offset = self.setpoint_rotation.theta - self.current_rotation.theta
                if abs(offset) > 0.05:
                    msg.angular.z = 0.5 if offset < 0 else -0.5
                else:
                    msg.angular.z = 0.0

                # Calcula a translação restante
                # Calcula o vetor relativo entre o robô e o destino
                self.relative_vector = Pose(
                    x=self.setpoint.x - self.current_pose.x, 
                    y=self.setpoint.y - self.current_pose.y
                )
    
                # Se o módulo da translação restante for maior que 0.1, translada
                if abs(self.desired - self.current) > 0.1:
                    msg.linear.x = 0.5 if self.desired - self.current else -0.5
                else:
                    msg.linear.x = 0.0
                    self.get_logger().info(f"O robô chegou ao destino")
                    self.publisher.publish(msg)
                    self.update_setpoint()

        self.publisher.publish(msg)

    def update_setpoint(self):
        try:
            self.setpoint = self.queue.dequeue() # Atualiza o ponto de destino

            self.get_logger().info(f"O robô chegou em {self.current_pose}, \
                                   andando para {self.setpoint}")
                       
            self.movement_origin = self.current_pose # Atualiza a origem do movimento

            # Se o setpoint for o ponto (0,0), atualiza o ângulo relativo para 0
            if self.setpoint == Pose(0.0,0.0):
                self.relative_vector_angle = Rotation(theta=0.0) 
            else:
                # Calcula o ângulo relativo entre o robô e o destino
                self.relative_vector_angle= Rotation(
                    theta=math.atan2(
                        self.setpoint.y - self.current_pose.y, 
                        self.setpoint.x - self.current_pose.x
                        )
                    )

            # Calcula o vetor relativo entre o robô e o destino
            self.relative_vector = Pose(
                x=self.setpoint.x - self.current_pose.x, 
                y=self.setpoint.y - self.current_pose.y
            )

            # Calcula a translação total desejada (módulo do vetor relativo)
            self.desired = math.sqrt(
                self.relative_vector.x**2 + self.relative_vector.y**2
            )

            # Calcula a rotação total desejada segundo o quadrante no qual o destino se encontra em relação ao robô
            if self.relative_vector.x >= 0 and self.relative_vector.y >=0:
                self.setpoint_rotation = Rotation(
                    theta=abs(self.relative_vector_angle.theta)
                    )

            elif self.relative_vector.x >=0 and self.relative_vector.y <= 0:
                self.setpoint_rotation = Rotation(
                    theta=-abs(self.relative_vector_angle.theta)
                    )

            elif self.relative_vector.x <=0 and self.relative_vector.y <= 0:
                self.setpoint_rotation = Rotation(
                    theta=-abs(self.relative_vector_angle.theta)
                    )
            else:
                self.setpoint_rotation = Rotation(
                    theta=abs(self.relative_vector_angle.theta)
                    )
       
       # Se a fila estiver vazia, indica que a jornada acabou
        except IndexError:
            self.get_logger().info(f"Rota Completa!")
            exit()

    # Callback para receber pose atual
    def pose_callback(self, msg):

            # Decompõe a mensagem de pose em x, y e theta
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            ang = msg.pose.pose.orientation
            _, _, theta = euler_from_quaternion([ang.x, ang.y, ang.z, ang.w])

            # Atualiza a pose e rotação atuais
            self.current_pose = Pose(x=x, y=y, theta=theta)
            self.current_rotation = Rotation(theta=self.current_pose.theta)

            # Calcula o quanto o robô andou desde o início do movimento
            self.current = math.sqrt(
                (self.current_pose.x - self.movement_origin.x)**2 + (self.current_pose.y - self.movement_origin.y)**2
                )

            # Se o robô estiver parado, atualiza o ponto de destino
            if not self.initiated:
                self.initiated = True
                self.update_setpoint()