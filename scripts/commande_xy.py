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

def isCloseEnough(value, target):
    return np.abs(target-value) < 0.10

# variables globales
commande = Twist()

# initialisation du noeud
rospy.init_node('commande_z', anonymous=True)

# declaration d'un publisher sur le topic de commande
pubCommande = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


WP = [0.3, 0.6, 1, 1.4] #en m
# WP = [1.0, 1.0]
timer = 5   #in s
nextWP = 0
isWaiting = False
startTime = time.time()
altitudeRef = 0.8

def GoToNextWP():
    global nextWP
    global isWaiting
    if nextWP < len(WP)-1:
        nextWP += 1
        isWaiting = False

# fonction de lecture de la mesure et d'envoi de la commande
def getAltitude(data):
    global nextWP
    global isWaiting
    global timer
    global startTime
    
    # lecture donnee altitude (en m)
    altitude = data.altd / 1000.0
    tag_xc = 0
    tag_yx = 0
    tag_width = -1
    tag_height = -1
    tag_orientation = -1
    tag_distance = -1
    cmd_x = 0
    cmd_y = 0
    k_xy = 0.25
    
    if data.tags_count >= 1:
        tag_xc = data.tags_xc[0]
        tag_yc = data.tags_yc[0]
        tag_width = data.tags_width[0]
        tag_height = data.tags_height[0]
        tag_distance = data.tags_distance[0]
        tag_orientation = data.tags_orientation[0]
        
        cmd_x = -((tag_yc-500.0)/500.0)*k_xy
        cmd_y = -((tag_xc-500.0)/500.0)*k_xy
        
    # affichage sur la console (pour info)
    rospy.loginfo("altitude (m) = %f \n", altitude)

    # calcul de la commande
    # *********** A MODIFIER EN TP *************
    if isCloseEnough(altitude, altitudeRef):
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
    
    # ****************************************

    # pblication de la commande
    pubCommande.publish(commande)    


# declaration d'un subscriber : appelle getAltitude a chq arrivee d'une donnee sur le topic Navdata
rospy.Subscriber("/ardrone/navdata", Navdata, getAltitude)



# fonction main executee en boucle 
if __name__ == '__main__':
    rospy.spin()


