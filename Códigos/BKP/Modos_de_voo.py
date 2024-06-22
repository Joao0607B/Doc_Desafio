#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import SetModeRequest  # Importe corretamente a mensagem SetModeRequest
from mavros_msgs.msg import State

current_state = State()

def state_cb(state):
    global current_state
    current_state = state

def set_flight_mode(mode):
    rospy.wait_for_service('/edrn/mavros/set_mode')
    try:
        set_mode_service = rospy.ServiceProxy('/edrn/mavros/set_mode', SetMode)
        
        # Cria a mensagem SetModeRequest corretamente
        mode_req = SetModeRequest()
        mode_req.base_mode = 0  # Normalmente não precisa ser alterado
        mode_req.custom_mode = mode

        # Chama o serviço com a requisição correta
        response = set_mode_service(mode_req)

        if response.mode_sent:
            rospy.loginfo(f'Modo de voo alterado para {mode}.')
        else:
            rospy.logwarn(f'Falha ao alterar o modo de voo para {mode}.')
    except rospy.ServiceException as e:
        rospy.logerr(f'Falha ao chamar o serviço de alteração de modo: {e}')

    # Mantém o nó rodando para monitorar o estado
    while not rospy.is_shutdown():
        rospy.loginfo(f'Estado atual do drone: Modo: {current_state.mode}, Armado: {current_state.armed}')
        rospy.sleep(1.0)

def main():
    rospy.init_node('set_mode_node', anonymous=True)
    rospy.Subscriber('/edrn/mavros/state', State, state_cb)
    
    # Aguarda a conexão com o FCU
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("Esperando conexão com FCU...")
        rate.sleep()
    
    rospy.loginfo("Conectado ao FCU")

    # Defina o modo desejado
    flight_mode = 'GUIDED'  # Substitua pelo modo desejado

    # Altera o modo de operação do drone
    set_flight_mode(flight_mode)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

