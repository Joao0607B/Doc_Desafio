#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandTOL, CommandTOLRequest, SetMode, SetModeRequest
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.msg import State

current_state = State()

def state_cb(state):
    global current_state
    current_state = state

def arm_and_takeoff_drone(target_altitude):
    # Inicializa o nó ROS
    rospy.init_node('takeoff_drone_client_node', anonymous=True)

    # Subscriber para o tópico de estado
    rospy.Subscriber('/edrn/mavros/state', State, state_cb)

    # Espera os serviços necessários estarem disponíveis
    rospy.wait_for_service('/edrn/mavros/cmd/arming')
    rospy.wait_for_service('/edrn/mavros/set_mode')
    rospy.wait_for_service('/edrn/mavros/cmd/takeoff')
    
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("Esperando conexão com FCU...")
        rate.sleep()
    
    rospy.loginfo("Conectado ao FCU")

    # Arma o drone
    try:
        arm_service = rospy.ServiceProxy('/edrn/mavros/cmd/arming', CommandBool)
        arm_request = CommandBoolRequest()
        arm_request.value = True
        arm_response = arm_service(arm_request)
        if arm_response.success:
            rospy.loginfo("Drone armado com sucesso!")
        else:
            rospy.logwarn("Falha ao armar o drone.")
            return
    except rospy.ServiceException as e:
        rospy.logerr(f"Falha ao chamar o serviço arming: {e}")
        return
    
    # Muda para o modo GUIDED
    try:
        set_mode_service = rospy.ServiceProxy('/edrn/mavros/set_mode', SetMode)
        set_mode_request = SetModeRequest()
        set_mode_request.custom_mode = 'GUIDED'
        set_mode_response = set_mode_service(set_mode_request)
        if set_mode_response.mode_sent:
            rospy.loginfo("Modo GUIDED configurado com sucesso!")
        else:
            rospy.logwarn("Falha ao configurar o modo GUIDED.")
            return
    except rospy.ServiceException as e:
        rospy.logerr(f"Falha ao chamar o serviço set_mode: {e}")
        return

    # Inicia a decolagem
    try:
        takeoff_service = rospy.ServiceProxy('/edrn/mavros/cmd/takeoff', CommandTOL)
        takeoff_request = CommandTOLRequest()
        takeoff_request.altitude = target_altitude
        takeoff_response = takeoff_service(takeoff_request)
        if takeoff_response.success:
            rospy.loginfo(f"Decolagem iniciada para {target_altitude} metros.")
        else:
            rospy.logwarn("Falha ao iniciar a decolagem.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Falha ao chamar o serviço de decolagem: {e}")

    # Mantém o nó rodando para monitorar o estado
    while not rospy.is_shutdown():
        rospy.loginfo(f"Estado atual do drone: Modo: {current_state.mode}, Armado: {current_state.armed}")
        rate.sleep()

if __name__ == "__main__":
    # Defina a altitude alvo para a decolagem em metros
    target_altitude = 10.0  # Altitude desejada em metros
    arm_and_takeoff_drone(target_altitude)
