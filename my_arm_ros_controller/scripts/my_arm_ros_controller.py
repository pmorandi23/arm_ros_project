#!/usr/bin/env python3
import rospy
# import keyboard
from pynput.keyboard import Key, Listener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
#from turtlesim.msg import Pose # msg for topics
#from turtlesim.srv import SetPen # srv for services 
#from geometry_msgs.msg import Twist # esto lo tengo que agregar en el package.xml para que compile

# Variables globales
SIMULATION_TIME = 1
# POSITION_ARM_W = [0.0,-1.572,-1.0,-1.0,1.0]
POSITION_ARM_W = [0.0,0.0,0.0,0.0,0.0]
POSITION_ARM_A = [0.0,0.0,-1.572,-1.572,0.0]
POSITION_ARM_S = [1.0,1.0,0.5,-2.0,-1.0]
POSITION_ARM_D = [1.572,0.0,0.0,0.0,1.572]
POSITION_HAND_Q = [0.001, 0.001]
POSITION_HAND_E = [0.01, -0.01]

# Brazo
# Joint1 ---> de -180° a 180° (-3,142rad a 3,142rad)
# Joint2 ---> de -90° a 90° (-1,57rad a 1,57rad)
# Joint3 ---> de -180° a 57,29° (-3,142rad a 1rad)
# Joint4 ---> de -180° a 0° (-3,14rad a 0rad)
# Joint5 ---> de  0° a 180°

# Mano
# Joint6 ----> de 0.0010m a 0.03m 
# Joint7 ----> de -0.03m a 0.0010m

# Ejemplos de state:
# HAND_ACTUAL_POSITION:
# (-0.00028681349312585083, -0.00035403529289333154)
# ARM_ACTUAL_POSITION:
# (1.8640231520095085e-05, -0.000438911951536447, 0.0001305284259149886, 0.000130157593694058, 0.004946943518867819)


def state_arm_callback(state: JointTrajectoryControllerState):
    #print("ARM_ACTUAL_POS_JOINT_1:\n", state.actual.positions[0])
    #print("ARM_ACTUAL_POS_JOINT_2:\n", state.actual.positions[1])
    #print("ARM_ACTUAL_POS_JOINT_3:\n", state.actual.positions[2])
    #print("ARM_ACTUAL_POS_JOINT_4:\n", state.actual.positions[3])
    #print("ARM_ACTUAL_POS_JOINT_5:\n", state.actual.positions[4])
    return
    

def state_hand_callback(state: JointTrajectoryControllerState):
    #print("HAND_ACTUAL_POS_JOINT_6:\n", state.actual.positions[0])
    #print("HAND_ACTUAL_POS_JOINT_7:\n", state.actual.positions[1])
    return 

    
def set_trayectory_hand (position):
    cmd_joint_tray = JointTrajectory ()
    cmd_joint_tray_point = JointTrajectoryPoint ()
    # Pinza 1 
    cmd_joint_tray.joint_names.append ('joint_6')
    # Pinza 2
    cmd_joint_tray.joint_names.append ('joint_7')
    # Punto objetivo en el espacio
    cmd_joint_tray_point.time_from_start.secs = SIMULATION_TIME
    cmd_joint_tray_point.positions = position
    cmd_joint_tray.points.append(cmd_joint_tray_point)
    rospy.loginfo("Executing trayectory...")
    pub_hand.publish(cmd_joint_tray)

def set_trayectory_arm (position):
    cmd_joint_tray = JointTrajectory ()
    cmd_joint_tray_point = JointTrajectoryPoint ()
    # Joints involucradas 
    cmd_joint_tray.joint_names.append ('joint_1')
    cmd_joint_tray.joint_names.append ('joint_2')
    cmd_joint_tray.joint_names.append ('joint_3')
    cmd_joint_tray.joint_names.append ('joint_4')
    cmd_joint_tray.joint_names.append ('joint_5')
    # Punto objetivo en el espacio
    cmd_joint_tray_point.time_from_start.secs = SIMULATION_TIME
    cmd_joint_tray_point.positions = position
    cmd_joint_tray.points.append(cmd_joint_tray_point)
    rospy.loginfo("Executing trayectory...")
    pub_arm.publish(cmd_joint_tray)

def on_press(key):
    # print ("tecla: ",key)
    if key == Key.esc:
        rospy.logwarn("Closing arm control...")
    elif format(key.char) == 'w':
        set_trayectory_arm(POSITION_ARM_W)
    elif format(key.char) == 's':
        set_trayectory_arm(POSITION_ARM_S)
    elif format(key.char) == 'a':
        set_trayectory_arm(POSITION_ARM_A)
    elif format(key.char) == 'd':
        set_trayectory_arm(POSITION_ARM_D)
    elif format(key.char) == 'q':
        set_trayectory_hand(POSITION_HAND_Q)
    elif format(key.char) == 'e':
        set_trayectory_hand(POSITION_HAND_E)
    else:
        rospy.logwarn("Please, enter a valid key.")

def on_release(key):
    if key == Key.esc:
        # Stop listener
        return False
    
if __name__ == '__main__':
    rospy.init_node("arm_ros_controller")
    pub_hand = rospy.Publisher("/hand_ee_controller/command",JointTrajectory, queue_size=10)
    pub_arm = rospy.Publisher("/robot_arm_controller/command",JointTrajectory, queue_size=10)
    sub_arm = rospy.Subscriber("/robot_arm_controller/state", JointTrajectoryControllerState, callback=state_arm_callback)
    sub_hand = rospy.Subscriber("/hand_ee_controller/state", JointTrajectoryControllerState, callback=state_hand_callback)

    rospy.loginfo("/arm_ros_controller node has been started. ")
    print("--------- ARM ROS CONTROLLER ------------")
    print("-----Authors: Sebastian Gavegno y Pablo Morandi --------")
    print("-------- Róbotica - UTN FRBA - 2023 -----------")
    print("Please, use keys ( w, a, s, d) to move the robot...")
    # Inicializo comando para controlar brazo
    # init_cmd()
    # Inicializo timer
    # Collect events until released
    with Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    






