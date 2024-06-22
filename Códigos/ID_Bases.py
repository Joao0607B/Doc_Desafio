import rospy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

current_state = State()
landing_spots = []
image_counter = 0
image_received = False
cv_image = None
drone_height = 0  # Inicializa a altura do drone
camera_position = [0, 0, 0]  # Inicializa a posição da câmera no drone
drone_position = [0, 0, 0]  # Inicializa a posição do drone

def state_cb(msg):
    global current_state
    current_state = msg

def set_mode_client(base_mode, custom_mode):
    rospy.wait_for_service('/edrn/mavros/set_mode')
    try:
        set_mode = rospy.ServiceProxy('/edrn/mavros/set_mode', SetMode)
        response = set_mode(base_mode, custom_mode)
        rospy.loginfo("SetMode response: {0}".format(response))
        return response.mode_sent
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {0}".format(e))
        return False

def arm_drone():
    rospy.wait_for_service('/edrn/mavros/cmd/arming')
    try:
        arm_service = rospy.ServiceProxy('/edrn/mavros/cmd/arming', CommandBool)
        while not rospy.is_shutdown():
            response = arm_service(True)
            if response.success:
                rospy.loginfo("Drone armado com sucesso!")
                return True
            else:
                rospy.logwarn("Falha ao armar o drone. Tentando novamente...")
            rospy.sleep(1)  # Aguarda 1 segundo antes de tentar novamente
    except:
        return False

def takeoff_drone(altitude):
    global drone_height, camera_position, drone_position
    rospy.wait_for_service('/edrn/mavros/cmd/takeoff')
    try:
        takeoff_service = rospy.ServiceProxy('/edrn/mavros/cmd/takeoff', CommandTOL)
        response = takeoff_service(altitude=altitude)
        if response.success:
            rospy.loginfo("Drone decolou com sucesso!")
            drone_height = altitude  # Atualiza a altura do drone após a decolagem
            camera_position = [0, 0, drone_height]  # Atualiza a posição da câmera para refletir a altura do drone
            drone_position[2] = altitude  # Atualiza a posição do drone
        else:
            rospy.logwarn("Falha ao decolar o drone.")
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Erro ao chamar o serviço de decolagem: {0}".format(e))
        return False

def land_drone():
    rospy.wait_for_service('/edrn/mavros/cmd/land')
    try:
        land_service = rospy.ServiceProxy('/edrn/mavros/cmd/land', CommandTOL)
        response = land_service(altitude=0)
        if response.success:
            rospy.loginfo("Drone pousou com sucesso!")
        else:
            rospy.logwarn("Falha ao pousar o drone.")
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Erro ao chamar o serviço de pouso: {0}".format(e))
        return False

def set_waypoint(position):
    global drone_position
    waypoint = PoseStamped()
    waypoint.pose.position.x = position[0]
    waypoint.pose.position.y = position[1]
    waypoint.pose.position.z = position[2]
    local_pos_pub.publish(waypoint)
    drone_position = position  # Atualiza a posição do drone

def convert_image_to_world_coordinates(x, y, drone_height):
    # Parâmetros da câmera
    focal_length = 1.0  # Suponha uma distância focal (ajuste conforme necessário)
    sensor_height = 1.0  # Altura do sensor da câmera (ajuste conforme necessário)
    
    # Conversão de pixels para metros (ajuste conforme necessário)
    pixel_to_meter = drone_height / focal_length

    # Ajuste baseado no centro da imagem
    x_centered = (x - 320) * pixel_to_meter
    y_centered = (y - 240) * pixel_to_meter

    # Coordenadas no mundo real
    x_world = x_centered + camera_position[0]
    y_world = y_centered + camera_position[1]
    z_world = 0  # Assumimos que o ponto de pouso está no chão

    return Point(x_world, y_world, z_world)

def image_callback(msg):
    global cv_image, image_received
    # Converte a imagem ROS para uma imagem OpenCV
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    image_received = True

def process_image():
    global cv_image, image_counter, drone_height, drone_position
    if cv_image is not None:
        # Converte a imagem para escala de HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Definindo limites para a máscara na escala de HSV
        lower_limit_ver = np.array([0, 50, 20])
        upper_limit_ver = np.array([80, 255, 255])
        # Aplicando a máscara
        mask_ver = cv2.inRange(hsv, lower_limit_ver, upper_limit_ver)

        # Usa a detecção de círculos para encontrar os pontos de pouso
        circles = cv2.HoughCircles(mask_ver, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30, param1=50, param2=30, minRadius=10, maxRadius=40)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                # Desenha o círculo e o centro na imagem
                cv_image = cv2.circle(cv_image, (x, y), r, (0, 255, 0), 4)
                cv_image = cv2.rectangle(cv_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                rospy.loginfo(f"Landing spot detected at: x={x}, y={y}, radius={r}")

                # Converte as coordenadas da imagem para coordenadas no mundo real
                world_position = convert_image_to_world_coordinates(x, y, drone_height)

                # Soma as coordenadas do drone para obter as coordenadas absolutas
                absolute_position = Point(
                    world_position.x + drone_position[0],
                    world_position.y + drone_position[1],
                    world_position.z
                )

                # Armazena a posição do ponto de pouso
                landing_spots.append(absolute_position)
                landing_spot_pub.publish(absolute_position)
                rospy.loginfo(f"Base de pouso detectada na coordenada: {absolute_position}")

        # Salva a imagem com a posição do drone
        image_path = os.path.join(os.path.expanduser("~"), "avant_ws/src/cbr24_simulation/scripts", f"image_{image_counter}.png")
        cv2.imwrite(image_path, cv_image)
        rospy.loginfo(f"Imagem salva: {image_path} com a posição do drone: {drone_position}")
        image_counter += 1

if _name_ == '_main_':
    rospy.init_node("drone_mission_node", anonymous=True)
    state_sub = rospy.Subscriber("/edrn/mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher("/edrn/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    image_sub = rospy.Subscriber('/edrn/camera/color/image_raw', Image, image_callback)
    bridge = CvBridge()
    landing_spot_pub = rospy.Publisher('/landing_spot_positions', Point, queue_size=10)

    rate = rospy.Rate(10)

    # Espera pela conexão com o FCU
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("Aguardando conexão com o FCU...")
        rate.sleep()

    rospy.loginfo("Conexão com o FCU estabelecida.")

    # Define o modo GUIDED
    if current_state.mode != "GUIDED":
        if set_mode_client(0, "GUIDED"):
            rospy.loginfo("Modo GUIDED definido com sucesso")
        else:
            rospy.logwarn("Falha ao definir o modo GUIDED")

    # Arma e decola o drone
    if arm_drone() and takeoff_drone(3):  # Ajusta a altura de decolagem conforme necessário
        base_positions = [(0, 5.5, 2.2), (12, 0, 1.2)]
        flight_coordinates = flight_coordinates = [
            (0, 5.5, 2.2), (8, 0, 5),
            (8, 1, 5), (0, 1, 5),
            (0, 2, 5), (8, 2, 5),
            (8, 3, 5), (0, 3, 5),
            (0, 4, 5), (8, 4, 5),
            (8, 5, 5), (0, 5, 5),
            (0, 6, 5), (8, 6, 5),
            (8, 7, 5), (0, 7, 5),
            (0, 8, 5), (8, 8, 5)
            ]

        for base_position in base_positions:
            rospy.loginfo(f"Indo para a base: {base_position}")
            set_waypoint(base_position)
            rospy.sleep(10)  # Aguarda 10 segundos antes de prosseguir

        for flight_coordinate in flight_coordinates:
            rospy.loginfo(f"Indo para a coordenada de voo: {flight_coordinate}")
            set_waypoint(flight_coordinate)
            rospy.sleep(10)  # Aguarda 10 segundos antes de processar a imagem
            process_image()

        # Pousa o drone
        if not land_drone():
            rospy.logwarn("Falha ao pousar o drone")

    rospy.spin()