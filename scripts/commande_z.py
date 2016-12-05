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
    # affichage sur la console (pour info)
    rospy.loginfo(rospy.get_caller_id() + "altitude (m) = %f", altitude)

    # calcul de la commande
    # *********** A MODIFIER EN TP *************
    altitudeRef = WP[nextWP]
    print("Current waypoint : %f", altitudeRef)
    if isCloseEnough(altitude, altitudeRef):
        if isWaiting:
            if time.time() - startTime >= timer:
                GoToNextWP()
                isWaiting = False
        else:
            startTime = time.time()
            isWaiting = True
    
    commande.linear.z = (altitudeRef - altitude)*1.5 #0.75
    
    # ****************************************

    # pblication de la commande
    pubCommande.publish(commande)    


# declaration d'un subscriber : appelle getAltitude a chq arrivee d'une donnee sur le topic Navdata
rospy.Subscriber("/ardrone/navdata", Navdata, getAltitude)



# fonction main executee en boucle 
if __name__ == '__main__':
    rospy.spin()


