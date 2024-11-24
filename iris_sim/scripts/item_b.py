#!/usr/bin/python3
# -*- coding:utf-8 -*-

# ===========================
# === CONFIGURANDO O NODE ===
# ===========================

import rospy # Biblioteca do ROS para Python
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State
from mavros_msgs.msg import ExtendedState
from mavros_msgs.msg import BatteryStatus

# Inicializa o node
rospy.init_node("takeoff_land")

# ================================
# === PUBLISHERS E SUBSCRIBERS ===
# ================================

current_state = State()
current_pose = PoseStamped()
goal_pose = PoseStamped()
extended_state = ExtendedState()
battery_status = BatteryStatus()

rate = rospy.Rate(20)

def multiple_rate_sleep(n):
    for i in range(n):
        rate.sleep()

# Funções de callback
def state_callback(msg):
    global current_state
    current_state = msg

def pose_callback(msg):
    global current_pose
    current_pose = msg

def extended_state_callback(msg):
    global extended_state
    extended_state = msg

def battery_callback(msg):
    global battery_status
    battery_status = msg

# Objetos de Service, Publisher e Subscriber
arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
local_position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
state_sub = rospy.Subscriber("/mavros/state", State, state_callback)
pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_callback)
extended_state_sub = rospy.Subscriber("/mavros/extended_state", ExtendedState, extended_state_callback)
battery_sub = rospy.Subscriber("/mavros/battery", BatteryStatus, battery_callback)

# =============================
# === PREPARACAO PARA O VOO ===
# =============================

# Espera a conexao ser iniciada
rospy.loginfo("Esperando conexao com FCU")
while not rospy.is_shutdown() and not current_state.connected:
    rate.sleep()

# Publica algumas mensagens antes de trocar o modo de voo
for i in range(100):
    local_position_pub.publish(goal_pose)
    rate.sleep()

# Coloca no modo Offboard
last_request = rospy.Time.now()
if (current_state.mode != "OFFBOARD"):
    result = set_mode_srv(0, "OFFBOARD")
    rospy.loginfo("Alterando para modo Offboard")
    while not rospy.is_shutdown() and current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(1.0)):
        result = set_mode_srv(0, "OFFBOARD")
    rospy.loginfo("Drone em modo Offboard")
else:
    rospy.loginfo("Drone já está em modo Offboard")

# Arma o drone
if (not current_state.armed):
    result = arm(True)
    rospy.loginfo("Armando o drone")
    while not rospy.is_shutdown() and not current_state.armed:
        result = arm(True)
    rospy.loginfo("Drone armado")
else:
    rospy.loginfo("Drone ja armado")

# =============================
# === MOVIMENTACAO DO DRONE ===
# =============================

TOL = 0.1

# Subir
rospy.loginfo("Subindo")
goal_pose.pose.position.z = 5
while not rospy.is_shutdown() and abs(goal_pose.pose.position.z - current_pose.pose.position.z) > TOL:
    local_position_pub.publish(goal_pose)
    rate.sleep()

# Espera 5 segundos após subir
rospy.loginfo("Esperando")
t0 = rospy.Time.now()
while not rospy.is_shutdown() and rospy.Time.now() - t0 < rospy.Duration(5):
    local_position_pub.publish(goal_pose)
    rate.sleep()

# Define as coordenadas para visitar
coordinates = [
    [5, 5, 5],  # (Xa, Ya, Za)
    [10, 5, 5], # (Xb, Yb, Zb)
    [10, 10, 5] # (Xc, Yc, Zc)
]

# Função para mover para as coordenadas e esperar 3 minutos
def move_and_wait(x, y, z):
    rospy.loginfo(f"Movendo para ({x}, {y}, {z})")
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = z

    # Move para a nova posição
    while not rospy.is_shutdown() and (
        abs(goal_pose.pose.position.x - current_pose.pose.position.x) > TOL or
        abs(goal_pose.pose.position.y - current_pose.pose.position.y) > TOL or
        abs(goal_pose.pose.position.z - current_pose.pose.position.z) > TOL):
        local_position_pub.publish(goal_pose)
        rate.sleep()

    rospy.loginfo(f"Chegou em ({x}, {y}, {z}), esperando 3 minutos")
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and rospy.Time.now() - t0 < rospy.Duration(180): # 3 minutos
        local_position_pub.publish(goal_pose)
        check_battery()
        rate.sleep()

# Função para verificar a bateria e retornar à base se necessário
def check_battery():
    if battery_status.percentage <= 0.1:  # 90% de carga restante
        rospy.loginfo("Bateria baixa! Retornando à base para carregamento.")
        move_and_wait(0, 0, 5)  # Retorna à coordenada de origem

# Visitando as coordenadas
for coord in coordinates:
    move_and_wait(coord[0], coord[1], coord[2])

# Retorna à coordenada de origem (0, 0, 5)
move_and_wait(0, 0, 5)

# Coloca no modo Land
if (current_state.mode != "AUTO.LAND"):
    result = set_mode_srv(0, "AUTO.LAND")
    rospy.loginfo("Alterando para modo Land")
    while not rospy.is_shutdown() and current_state.mode != "AUTO.LAND":
        result = set_mode_srv(0, "AUTO.LAND")
    rospy.loginfo("Drone em modo Land")
else:
    rospy.loginfo("Drone ja esta em modo Land")

# Espera Pousar
land_state_on_ground = 1
while not rospy.is_shutdown() and extended_state.landed_state != land_state_on_ground:
    if(extended_state.landed_state == 1):
        rospy.loginfo("Pousado no solo. Altura = " + str(current_pose.pose.position.z) + " m")
    elif(extended_state.landed_state == 2):
        rospy.loginfo("Voando. Altura = " + str(current_pose.pose.position.z) + " m")
    elif(extended_state.landed_state == 3):
        rospy.loginfo("Decolando. Altura = " + str(current_pose.pose.position.z) + " m")
    elif(extended_state.landed_state == 4):
        rospy.loginfo("Pousando. Altura = " + str(current_pose.pose.position.z) + " m") 
    multiple_rate_sleep(10)

# Finaliza a simulação
while not rospy.is_shutdown():
    print("Pressione Ctrl+C para encerrar a simulacao")
    multiple_rate_sleep(100)
