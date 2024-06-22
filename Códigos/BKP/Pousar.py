#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandTOL, CommandTOLRequest
from mavros_msgs.msg import State

current_state = State()

def state_cb(state):
    global current_state
    current_state = state

def land_drone():
    # Inicializa o nó ROS
    rospy.init_node('land_drone_client_node', anonymous=True)

    # Subscriber para o tópico de estado
    rospy.Subscriber('/edrn/mavros/state', State, state_cb)

    # Espera o serviço /edrn/mavros/cmd/land estar disponível
    rospy.wait_for_service('/edrn/mavros/cmd/land')
    
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("Esperando conexão com FCU...")
        rate.sleep()
    
    rospy.loginfo("Conectado ao FCU")
    
    try:
        # Cria um cliente de serviço para /edrn/mavros/cmd/land
        land_service = rospy.ServiceProxy('/edrn/mavros/cmd/land', CommandTOL)
        
        # Cria a requisição CommandTOL
        request = CommandTOLRequest()

        # Chama o serviço com a requisição
        response = land_service(request)
        
        # Verifica a resposta do serviço
        if response.success:
            rospy.loginfo("Aterrissagem iniciada.")
        else:
            rospy.logwarn("Falha ao iniciar a aterrissagem.")
            
    except rospy.ServiceException as e:
        rospy.logerr(f"Falha ao chamar o serviço de aterrissagem: {e}")

    # Mantém o nó rodando para monitorar o estado
    while not rospy.is_shutdown():
        rospy.loginfo(f"Estado atual do drone: Modo: {current_state.mode}, Armado: {current_state.armed}")
        rate.sleep()

if __name__ == "__main__":
    land_drone()
