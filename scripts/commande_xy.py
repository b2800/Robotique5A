#!/usr/bin/env python

# *************************
#     MINEURE ROBOTIQUE 5A
#          ESIEA - ONERA
# *************************

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
import numpy as np 
import time

threshold = 15

def isCloseEnough(value, target):
    return np.abs(target-value) < threshold

def isAngleCloseEnough(value, target):
    return isCloseEnough(value, target) or isCloseEnough(value+360, target) or isCloseEnough(value-360, target)

# variables globales
commande = Twist()

# initialisation du noeud
rospy.init_node('commande_z', anonymous=True)

# declaration d'un publisher sur le topic de commande
pubCommande = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


RotationWP = [0, 90, 180, 360] #en degres

timer = 4   #in s
nextWP = 0
isWaiting = False
startTime = time.time()
altitudeRef = 0.8

cmd_x = 0
cmd_y = 0

def GoToNextWP():
    global nextWP
    global isWaiting
    if nextWP < len(RotationWP)-1:
        nextWP += 1
        isWaiting = False

# fonction de lecture de la mesure et d'envoi de la commande
def getAltitude(data):
    global nextWP
    global isWaiting
    global timer
    global startTime
    global cmd_x
    global cmd_y
    
    # lecture donnee altitude (en m)
    altitude = data.altd / 1000.0
    tag_xc = 0
    tag_yc = 0
    tag_orientation = -1

    k_xy = 0.25
    k_angle = 4
    
    cmd_angular_z = 0
    
    angle_target = RotationWP[nextWP]
    
    print("angle target : ", angle_target)
    
    if data.tags_count >= 1:
        tag_xc = data.tags_xc[0]
        tag_yc = data.tags_yc[0]
        tag_orientation = data.tags_orientation[0]
        
        cmd_x = -((tag_yc-500.0)/500.0)*k_xy #Forward/backward
        cmd_y = -((tag_xc-500.0)/500.0)*k_xy #Left/right
        
        # Calul de l'angle
        cmd_angular_z = tag_orientation - angle_target
        if cmd_angular_z > 180:
            cmd_angular_z = 360 - cmd_angular_z
        if cmd_angular_z > 360:
            cmd_angular_z -= 360.0
        elif cmd_angular_z < 0:
            cmd_angular_z += 360.0
        cmd_angular_z /= 360.0
        cmd_angular_z *= k_angle
        
        if isAngleCloseEnough(tag_orientation, angle_target):
            if isWaiting:
                if time.time() - startTime >= timer:
                    GoToNextWP()
                    isWaiting = False
            else:
                startTime = time.time()
                isWaiting = True
    
    # commande.linear.z = (altitudeRef - altitude)*1.5 #0.75

    commande.linear.x = cmd_x
    rospy.loginfo("X correction : %f", cmd_x)

    commande.linear.y = cmd_y
    rospy.loginfo("Y correction : %f", cmd_y )
    
    commande.angular.z = cmd_angular_z
    rospy.loginfo("Angular correction : %f", cmd_angular_z)    
    
    # publication de la commande
    pubCommande.publish(commande)    

# declaration d'un subscriber : appelle getAltitude a chq arrivee d'une donnee sur le topic Navdata
rospy.Subscriber("/ardrone/navdata", Navdata, getAltitude)


# fonction main executee en boucle 
if __name__ == '__main__':
    rospy.spin()


