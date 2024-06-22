#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.msg import State

current_state = State()

def state_cb(state):
    global current_state
    current_state = state

def arm_drone(arm):
    # Inicializa o nó ROS
    rospy.init_node('arm_drone_client_node', anonymous=True)

    # Subscriber para o tópico de estado
    rospy.Subscriber('/edrn/mavros/state', State, state_cb)

    # Espera o serviço /edrn/mavros/cmd/arming estar disponível
    rospy.wait_for_service('/edrn/mavros/cmd/arming')
    
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("Esperando conexão com FCU...")
        rate.sleep()
    
    rospy.loginfo("Conectado ao FCU")
    
    try:
        # Cria um cliente de serviço para /edrn/mavros/cmd/arming
        arm_service = rospy.ServiceProxy('/edrn/mavros/cmd/arming', CommandBool)
        
        # Cria a requisição CommandBool
        request = CommandBoolRequest()
        request.value = arm

        # Chama o serviço com a requisição
        response = arm_service(request)
        
        # Verifica a resposta do serviço
        if response.success:
            if arm:
                rospy.loginfo("Drone armado com sucesso!")
            else:
                rospy.loginfo("Drone desarmado com sucesso!")
        else:
            if arm:
                rospy.logwarn("Falha ao armar o drone.")
            else:
                rospy.logwarn("Falha ao desarmar o drone.")
            
    except rospy.ServiceException as e:
        rospy.logerr(f"Falha ao chamar o serviço arming: {e}")

    # Mantém o nó rodando para monitorar o estado
    while not rospy.is_shutdown():
        rospy.loginfo(f"Estado atual do drone: Armado: {current_state.armed}, Conectado: {current_state.connected}, Modo: {current_state.mode}")
        # Verifica se o drone foi desarmado e tenta rearmar
        if arm and not current_state.armed:
            rospy.logwarn("Drone desarmado, tentando armar novamente...")
            try:
                response = arm_service(request)
                if response.success:
                    rospy.loginfo("Drone rearmado com sucesso!")
                else:
                    rospy.logwarn("Falha ao rearmar o drone.")
            except rospy.ServiceException as e:
                rospy.logerr(f"Falha ao chamar o serviço arming: {e}")
        rate.sleep()

if __name__ == "__main__":
    # Defina True para armar o drone, False para desarmar
    arm = True
    arm_drone(arm)