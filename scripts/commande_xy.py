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


def isCloseEnough(value, target, threshold):
    return np.abs(target-value) < threshold

def isAngleCloseEnough(value, target, threshold):
    return isCloseEnough(value, target, threshold) or isCloseEnough(value+360, target, threshold) or isCloseEnough(value-360, target, threshold)


# initialisation du noeud
rospy.init_node('commande_z', anonymous=True)

# declaration d'un publisher sur le topic de commande
pubCommande = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


class Drone:
    def __init__(self):
        # Configuration 
        self.rotation_threshold = 15    # in degrees
        self.altitude_threshold = 0.1   # in meters 
        self.center_threshold = 150
        self.rotationWP = [0, 90, 180, 360] #in degrees
        self.nextWaypointIndex = 0
        self.altitudeRef = 0.8 # in meters
        self.k_z = 1.5
        self.k_xy = 0.85
        self.k_angle = 3
        self.enable_Z = False
        self.enable_XY = True
        self.enable_angular_Z = True
        
        # Timer related
        self.timer = 4   # in seconds
        self.isWaiting = False
        self.startTime = time.time()

        # Commands related
        self.cmd_x = 0
        self.cmd_y = 0
        self.cmd_z = 0
        self.cmd_angular_z = 0
        self.lastKnownTagDirection = [0, 0]
        self.middle_value = 400.0 # should be half the range of tag_xc

    def FlyOverTag(self, data):
        self.StabilizeZ(data)
        if data.tags_count >= 1:
            self.StabilizeXY(data)
            self.AlignWithTarget(data)
        else:
            self.FlyToLastKnownTagPosition(data)
            
        self.PublishCommands()
    
    def StabilizeZ(self, data):
        if not self.enable_Z:
            return
        altitude = data.altd / 1000.0
        self.cmd_z = (self.altitudeRef - altitude)*self.k_z

    def StabilizeXY(self, data):
        if not self.enable_XY:
            return
        tag_xc = data.tags_xc[0]
        tag_yc = data.tags_yc[0]
        print(tag_xc, tag_yc)
        
        self.cmd_x = -((tag_yc-self.middle_value)/self.middle_value)*self.k_xy #Forward/backward
        self.cmd_y = -((tag_xc-self.middle_value)/self.middle_value)*self.k_xy #Left/right
        print(self.cmd_x, self.cmd_y)
        norm = np.linalg.norm([self.cmd_x, self.cmd_y])
        self.lastKnownTagDirection[0] = self.cmd_x / norm
        self.lastKnownTagDirection[1] = self.cmd_y / norm
        
    def AlignWithTarget(self, data):
        # Early exit if too far from tag
        if not self.enable_angular_Z or not self.TagIsCentered(data):
            return
            
        print("Centered enough")
        angle_target = self.rotationWP[self.nextWaypointIndex]
        tag_orientation = data.tags_orientation[0]
        print("angle target : ", angle_target)
        print("real angle : ", tag_orientation)

        # Calul de l'angle
        self.cmd_angular_z = tag_orientation - angle_target
        if self.cmd_angular_z > 180:
            self.cmd_angular_z = 360 - self.cmd_angular_z
        if self.cmd_angular_z > 360:
            self.cmd_angular_z -= 360.0
        elif self.cmd_angular_z < 0:
            self.cmd_angular_z += 360.0
        self.cmd_angular_z /= 360.0
        self.cmd_angular_z *= self.k_angle
        

        if isAngleCloseEnough(tag_orientation, angle_target, self.rotation_threshold):
            if self.isWaiting:
                if time.time() - self.startTime >= self.timer:
                    self.GoToNextWP()
                    self.isWaiting = False
            else:
                self.startTime = time.time()
                self.isWaiting = True
              
    def TagIsCentered(self, data):
        if data.tags_count == 0:
            return
        tag_xc = data.tags_xc[0]
        tag_yc = data.tags_yc[0]

        return isCloseEnough(tag_xc, self.middle_value, self.center_threshold) and isCloseEnough(tag_yc, self.middle_value, self.center_threshold)        
        
    def FlyToLastKnownTagPosition(self, data):
        return
        self.cmd_x = self.lastKnownTagDirection[0] * self.k_xy
        self.cmd_y = self.lastKnownTagDirection[1] * self.k_xy


    def PublishCommands(self):
        commande = Twist()

        commande.linear.x = self.cmd_x
        rospy.loginfo("X correction : %f", self.cmd_x)
    
        commande.linear.y = self.cmd_y
        rospy.loginfo("Y correction : %f", self.cmd_y )
        
        commande.linear.z = self.cmd_z
        
        commande.angular.z = self.cmd_angular_z
        rospy.loginfo("Angular correction : %f", self.cmd_angular_z)    
        
        pubCommande.publish(commande)    
        
    def GoToNextWP():
        if self.nextWP < len(self.rotationWP)-1:
            self.nextWP += 1
            self.isWaiting = False 
    
    
d = Drone()
rospy.Subscriber("/ardrone/navdata", Navdata, d.FlyOverTag)


# fonction main executee en boucle 
if __name__ == '__main__':
    rospy.spin()


