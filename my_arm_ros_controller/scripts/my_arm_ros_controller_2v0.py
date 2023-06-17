#!/usr/bin/env python3
import rospy
# import keyboard
from pynput.keyboard import Key, Listener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
#from turtlesim.msg import Pose # msg for topics
#from turtlesim.srv import SetPen # srv for services 
#from geometry_msgs.msg import Twist # esto lo tengo que agregar en el package.xml para que compile

# POSITIONS
POS_1 = 1
POS_2 = 2
POS_3 = 3
POS_4 = 4
POS_5 = 5
POS_6 = 6
POS_7 = 7

##### GOAL POSITIONS #####
# ARM
POSITION_ARM_1 = [0.0, 0.0, 0.0, 0.0, 0.0]
POSITION_ARM_1_5 = [0.0, 1.0, 0.2, -2.8, 0.0]
POSITION_ARM_2 = [0.0, 1.57, -0.2905, -2.8, 0.0]
POSITION_ARM_3 = [0.0, 1.57, -1.572, -2.9766, 0.0]
POSITION_ARM_4 = [3.142, 1.57, -1.572, -2.9766, 0.0]
POSITION_ARM_5 = [3.142, 1.57, -0.2905, -2.8, 0.0]
POSITION_ARM_5_5 = [3.142, 1.0, 0.2, -2.8, 0.0]
POSITION_ARM_6 = [3.142, 1.57, -1.572, -2.9436, 0.0]
POSITION_ARM_7 = [0.0, -0.5233, 0.9368, -0.6504, 0.0]
# HAND
POSITION_HAND_CLOSED = [0.001, -0.001]
POSITION_HAND_MID = [0.020, -0.020]
POSITION_HAND_OPEN = [0.029, -0.029]

##### AUX POSITIONS ####(para consultar si llego al objetivo)
# ARM
POSITION_ARM_1_AUX = [0.01, 0.01, 0.01, 0.01, 0.01]
POSITION_ARM_2_AUX = [0.01, 1.56, -0.28, -2.79, 0.01]
POSITION_ARM_3_AUX = [0.01, 1.56, -1.56, -2.96, 0.00]
POSITION_ARM_4_AUX = [3.13, 1.56, -1.56, -2.96, 0.00]
POSITION_ARM_5_AUX = [3.13, 1.56, -0.30, -2.95, 0.00]
POSITION_ARM_6_AUX = [3.13, 1.56, -1.56, -2.95, 0.00]
POSITION_ARM_7_AUX = [0.01, -0.51, 0.92, -0.66, 0.01]
# HAND
POSITION_HAND_CLOSED_AUX = [0.001, -0.001]
POSITION_HAND_MID_AUX = [0.02, -0.02]
POSITION_HAND_OPEN_AUX = [0.029, -0.029]

# Variables globales
start_trayectory = False
actual_pos = 0
SIMULATION_TIME = 1

# Brazo
# Joint1 ---> de -180° a 180° (-3,142rad a 3,142rad)
# Joint2 ---> de -90° a 90° (-1,57rad a 1,57rad)
# Joint3 ---> de -180° a 57,29° (-3,142rad a 1rad)
# Joint4 ---> de -180° a 0° (-3,14rad a 0rad)
# Joint5 ---> de  0° a 180°

# Mano
# Joint6 ----> de 0.0010m a 0.03m 
# Joint7 ----> de -0.03m a 0.0010m



def state_arm_callback(state: JointTrajectoryControllerState):

    global start_trayectory
    global actual_pos
    if (start_trayectory):
        list_actual_pos = list(state.actual.positions)
        #print ("TYPE ARM_ACTUAL_POS:\n", type(state.actual.positions))
        #print ("LIST ACTUAL POS:\n",list(state.actual.positions))
        #print ("TYPE ARM_AUX_POS:\n", type(POSITION_ARM_1_AUX))
        #print ("LIST ARM_AUX_POS:\n",POSITION_ARM_1_AUX)
        # Si ya estoy en la POS_1
        # POS_1 --> POS_2
        if (list_actual_pos[0] < POSITION_ARM_1_AUX[0] and
            list_actual_pos[1] < POSITION_ARM_1_AUX[1] and
            list_actual_pos[2] < POSITION_ARM_1_AUX[2] and
            list_actual_pos[3] < POSITION_ARM_1_AUX[3] and
            list_actual_pos[4] < POSITION_ARM_1_AUX[4] and 
            actual_pos != POS_1):
            actual_pos = POS_1
            rospy.loginfo("POS_1 reached.")
            rospy.loginfo("Going to POS_2...")
            set_trayectory_arm(POSITION_ARM_2)
        # POS_2 --> POS_3
        elif (list_actual_pos[0] < POSITION_ARM_2_AUX[0] and
              list_actual_pos[1] > POSITION_ARM_2_AUX[1] and
              list_actual_pos[2] < POSITION_ARM_2_AUX[2] and
              list_actual_pos[3] < POSITION_ARM_2_AUX[3] and
              list_actual_pos[4] < POSITION_ARM_2_AUX[4] and 
            actual_pos == POS_1):
            actual_pos = POS_2
            rospy.loginfo("POS_2 reached.")
            rospy.loginfo("Going to POS_3...")
            set_trayectory_arm(POSITION_ARM_3)
        # POS_3 --> POS_4
        elif (list_actual_pos[0] < POSITION_ARM_3_AUX[0] and
              list_actual_pos[1] > POSITION_ARM_3_AUX[1] and
              list_actual_pos[2] < POSITION_ARM_3_AUX[2] and
              list_actual_pos[3] < POSITION_ARM_3_AUX[3] and
              list_actual_pos[4] > POSITION_ARM_3_AUX[4] and 
            actual_pos == POS_2):
            actual_pos = POS_3
            rospy.loginfo("POS_3 reached.")
            rospy.loginfo("Going to POS_4...")
            set_trayectory_arm(POSITION_ARM_4)
        # POS_4 --> POS_5
        elif (list_actual_pos[0] > POSITION_ARM_4_AUX[0] and
              list_actual_pos[1] > POSITION_ARM_4_AUX[1] and
              list_actual_pos[2] < POSITION_ARM_4_AUX[2] and
              list_actual_pos[3] < POSITION_ARM_4_AUX[3] and
              list_actual_pos[4] > POSITION_ARM_4_AUX[4] and 
            actual_pos == POS_3):
            actual_pos = POS_4
            rospy.loginfo("POS_4 reached.")
            rospy.loginfo("Going to POS_5...")
            set_trayectory_arm(POSITION_ARM_5)
        # POS_5 --> POS_6
        elif (list_actual_pos[0] > POSITION_ARM_5_AUX[0] and
              list_actual_pos[1] > POSITION_ARM_5_AUX[1] and
              list_actual_pos[2] > POSITION_ARM_5_AUX[2] and
              list_actual_pos[3] > POSITION_ARM_5_AUX[3] and
              list_actual_pos[4] > POSITION_ARM_5_AUX[4] and 
            actual_pos == POS_4):
            actual_pos = POS_5
            rospy.loginfo("POS_5 reached.")
            rospy.loginfo("Going to POS_6...")
            set_trayectory_arm(POSITION_ARM_6)
        # POS_6 --> POS_7
        elif (list_actual_pos[0] > POSITION_ARM_6_AUX[0] and
              list_actual_pos[1] > POSITION_ARM_6_AUX[1] and
              list_actual_pos[2] < POSITION_ARM_6_AUX[2] and
              list_actual_pos[3] > POSITION_ARM_6_AUX[3] and
              list_actual_pos[4] > POSITION_ARM_6_AUX[4] and 
            actual_pos == POS_5):
            actual_pos = POS_6
            rospy.loginfo("POS_6 reached.")
            rospy.loginfo("Going to POS_7...")
            set_trayectory_arm(POSITION_ARM_7)
        # POS_7
        elif (list_actual_pos[0] < POSITION_ARM_7_AUX[0] and
              list_actual_pos[1] < POSITION_ARM_7_AUX[1] and
              list_actual_pos[2] > POSITION_ARM_7_AUX[2] and
              list_actual_pos[3] > POSITION_ARM_7_AUX[3] and
              list_actual_pos[4] < POSITION_ARM_7_AUX[4] and 
            actual_pos == POS_6):
            actual_pos = POS_7
            rospy.loginfo("POS_7 reached.")
            rospy.loginfo("Auto trajectory complete successfully.")
            start_trayectory = False
            #set_trayectory_arm(POSITION_ARM_7)
            
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
    #rospy.loginfo("Executing trayectory...")
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
    #rospy.loginfo("Executing trayectory...")
    pub_arm.publish(cmd_joint_tray)

def on_press(key):
    if (type(key) == Key):
        if key == Key.esc:
            rospy.logwarn("Closing arm control...")
        elif key == Key.space:
            rospy.loginfo("Starting automatic trayectory...")
            rospy.loginfo("Going to POS_1...")
            set_trayectory_arm(POSITION_ARM_1)
            global start_trayectory
            start_trayectory = True


    else:
        if format(key.char) == '1':
            rospy.loginfo("Starting trayectory 1...")
            set_trayectory_arm(POSITION_ARM_1)
        elif format(key.char) == 'a':
            rospy.loginfo("Starting trayectory 2,5...")
            set_trayectory_arm(POSITION_ARM_1_5)
        elif format(key.char) == 'b':
            rospy.loginfo("Starting trayectory 5,5...")
            set_trayectory_arm(POSITION_ARM_5_5)
        elif format(key.char) == '2':
            rospy.loginfo("Starting trayectory 2...")
            set_trayectory_arm(POSITION_ARM_2)
        elif format(key.char) == '3':
            rospy.loginfo("Starting trayectory 3...")
            set_trayectory_arm(POSITION_ARM_3)
        elif format(key.char) == '4':
            rospy.loginfo("Starting trayectory 4...")
            set_trayectory_arm(POSITION_ARM_4)
        elif format(key.char) == '5':
            rospy.loginfo("Starting trayectory 5...")
            set_trayectory_arm(POSITION_ARM_5)
        elif format(key.char) == '6':
            rospy.loginfo("Starting trayectory 6...")
            set_trayectory_arm(POSITION_ARM_6)  
        elif format(key.char) == '7':
            rospy.loginfo("Starting trayectory 7...")
            set_trayectory_arm(POSITION_ARM_7)        
        elif format(key.char) == 'q':
            rospy.loginfo("Starting trayectory q (hand closing)...")
            set_trayectory_hand(POSITION_HAND_CLOSED)
        elif format(key.char) == 'w':
            rospy.loginfo("Starting trayectory w (hand half open)...")
            set_trayectory_hand(POSITION_HAND_MID)
        elif format(key.char) == 'e':
            rospy.loginfo("Starting trayectory e (hand opening)...")
            set_trayectory_hand(POSITION_HAND_OPEN)
        else:
            rospy.logwarn("Please, enter a valid key. For closing, press Esc")
        
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
    #gazebo_contact = rospy.Subscriber("/contacts",ContactsState, callback=collision_callback)
    
    rospy.loginfo("/arm_ros_controller node has been started. ")
    print("--------- ARM ROS CONTROLLER ------------")
    print("-----Authors: Sebastian Gavegno y Pablo Morandi --------")
    print("-------- Róbotica - UTN FRBA - 2023 -----------")
    print("-------------------------------------------------")
    print("Please, press key Space to start arm robot automatic trayectory")
    print("-------------------------------------------------")
    print("Or set manual trayectory with keys: 1, 2, 3, 4, 5, 6 ,7 q, w, e")

    # Inicializo comando para controlar brazo
    # init_cmd()
    # Inicializo timer
    # Collect events until released
    with Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    






