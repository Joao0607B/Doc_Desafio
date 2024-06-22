import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

# Inicializa o nó ROS
rospy.init_node('send_waypoint')

# Cria um publicador para o tópico /edrn/mavros/setpoint_position/local
waypoint_pub = rospy.Publisher('/edrn/mavros/setpoint_position/local', PoseStamped, queue_size=10)

# Função para enviar o waypoint
def send_waypoint():
    # Cria a mensagem PoseStamped
    waypoint = PoseStamped()
    
    # Define o cabeçalho
    waypoint.header.stamp = rospy.Time.now()
    waypoint.header.frame_id = "map"  # ou "world", dependendo do seu sistema
    
    # Define a posição (x, y, z) em metros
    waypoint.pose.position.x = 10.0 # Exemplo: 1 metro no eixo x
    waypoint.pose.position.y = 2.0  # Exemplo: 2 metros no eixo y
    waypoint.pose.position.z = 5.0  # Exemplo: 5 metros no eixo z

    # Define a orientação (roll, pitch, yaw) em radianos
    waypoint.pose.orientation.x = 0.0
    waypoint.pose.orientation.y = 0.0
    waypoint.pose.orientation.z = 0.0
    waypoint.pose.orientation.w = 1.0  # Quaternion representando a orientação

    # Publica a mensagem
    waypoint_pub.publish(waypoint)
    rospy.loginfo("Waypoint enviado: x=%.2f, y=%.2f, z=%.2f", waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z)

# Espera um pouco para garantir que o nó está completamente inicializado
rospy.sleep(1)

# Chama a função para enviar o waypoint
send_waypoint()

# Mantém o nó rodando
rospy.spin()
