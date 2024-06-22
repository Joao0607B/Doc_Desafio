import rospy
from geometry_msgs.msg import PoseStamped

# Inicializa o nó ROS
rospy.init_node('current_position_listener')

# Função de callback que processa a mensagem PoseStamped
def pose_callback(msg):
    position = msg.pose.position
    orientation = msg.pose.orientation

    # Exibe a posição e orientação do drone
    rospy.loginfo("Posição atual do drone (relativa à base de decolagem): x=%.2f, y=%.2f, z=%.2f", position.x, position.y, position.z)
    rospy.loginfo("Orientação atual do drone: x=%.2f, y=%.2f, z=%.2f, w=%.2f", orientation.x, orientation.y, orientation.z, orientation.w)

# Inscreve-se no tópico /edrn/mavros/local_position/pose
pose_sub = rospy.Subscriber('/edrn/mavros/local_position/pose', PoseStamped, pose_callback)

# Mantém o nó rodando
rospy.spin()
