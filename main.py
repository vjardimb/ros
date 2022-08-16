#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Every Python ROS Node will have this declaration at the top. 
# The first line makes sure our script is executed as a Python script. 

# Necessary when writing a ROS node
import rospy
import sys, termios, tty

# Pour le traitement d'image
import cv_bridge
import cv2

# Pour les listes et les calculs...
import numpy as np
from numpy import inf

# Les msg sorti de la camera du robot
from sensor_msgs.msg import Image
# Les msg sort du sensor lazer du robot
from sensor_msgs.msg import LaserScan
# Envoyer les msg de movement du robot
from geometry_msgs.msg import Twist
# Savoir la position actuelle du robot
from sensor_msgs.msg import Imu
# Savoir la position actuelle du robot
from nav_msgs.msg import Odometry


from tf.transformations import euler_from_quaternion, quaternion_from_euler



class Trajet:
    def __init__(self):
        self.cvBridge = cv_bridge.CvBridge()

        # Le subscriber de /scan  le lazer
        self.lazer_subs = rospy.Subscriber('/scan', LaserScan, self.laser_callback)        
        # Le subscriber de /camera/image  la camera 
        self.camera_subs = rospy.Subscriber('/camera/image', Image, self.image_callback)

        #self.pos_sub = rospy.Subscriber('/imu', Imu, self.get_rotation)
        self.pos_sub = rospy.Subscriber('/odom', Odometry, self.get_rotation)        
        # Publiser pour envoyer les commande de movement du robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        
        self.detect = False #Detection de ligne rouge
        self.front = False # True if obstacle devant
        self.left = False
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.i = 0 # nombre de ligne rouges passees
        
        # Fonction pour challenge 1
        # suivre trajet en evitant les obstacles
    def image_callback(self, msg): # Camera subs
        # detection du rouge toujours
        # Mais le reste seulement au moment de suivre les lignes
        
        # rouge
        red_low = np.array([160,50,50], dtype=np.uint8)
        red_high = np.array([180,255,255], dtype=np.uint8)
            
        # Transform the image to openCV format
        cvImage = self.cvBridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Change color format from BGR to HSV
        hsv = cv2.cvtColor(cvImage, cv2.COLOR_BGR2HSV)
        # Transform the image to openCV format
        cvImage = self.cvBridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Change color format from BGR to HSV
        hsv = cv2.cvtColor(cvImage, cv2.COLOR_BGR2HSV)
            
        # Detect the object based on HSV Range Values
        # red line
        mask_red = cv2.inRange(hsv, red_low, red_high)
            
        # Savoir les dimensions de la photo sorti de la camera
        h, w, d = cvImage.shape   # h=240 # w=320 # d=3
        limit_top = int(3*h/4)
        limit_bot = int(3*h/4 + 60)
        limit_left = int(w/2 - 120)
        limit_right = int(w/2 + 120)
        # Mettre les limites pour la camera du robot
        # Pour qu'il ne prend compte que des lignes proche de lui
        mask_red[0:limit_top, 0:w] = 0
        mask_red[limit_bot:h, 0:w] = 0
        mask_red[0:h, 0:limit_left] = 0
        mask_red[0:h, limit_right:w] = 0
        
        M_r = cv2.moments(mask_red)    

        if M_r["m00"] > 0: #red
        # Calculate x coordinate of center
            cX_r = int( M_r["m10"] / M_r["m00"] )
        else:
            cX_r = 0 # valeure minimale a gauche    
            
        # detection ligne rouge
        # Si il a detecte une ligne ensuite il ne voit plus donc il a passe par une
        if ( (cX_r > 100) and (cX_r < 200) ):
            self.detect = True
        else :
            if (self.detect):
                self.detect = False
                if (self.i >= 5) :# On a termine le trajet
                   self.i = 0#reprendre des le debut
                else:
                    self.i+=1     
                print("lignes rouge passees =", self.i)
                
        ## Pour suivre les lignes en evitant les obstacles
        # On est dans challenge 1 ou le trajet entre challenge 2 et 3
        if ( (self.i!=3)  and (self.i!=5) ):
            # Limites des couleurs
            # jaune
            yellow_low = np.array([22, 93, 0], dtype=np.uint8)
            yellow_high = np.array([45, 255, 255], dtype=np.uint8)
            # blanc
            white_low = np.array([0,0,240], dtype=np.uint8)
            white_high = np.array([255,15,255], dtype=np.uint8)     
            
            # Detect the object based on HSV Range Values
            # yellow line
            mask_yellow = cv2.inRange(hsv, yellow_low, yellow_high)
            # white line
            mask_white = cv2.inRange(hsv, white_low, white_high)
            
            # Mettre les limites pour la camera du robot
            # Pour qu'il ne prend compte que des lignes proche de lui
            mask_yellow[0:limit_top, 0:w] = 0
            mask_yellow[limit_bot:h, 0:w] = 0
            mask_yellow[0:h, 0:limit_left] = 0
            mask_yellow[0:h, limit_right:w] = 0
            
            mask_white[0:limit_top, 0:w] = 0
            mask_white[limit_bot:h, 0:w] = 0
            mask_white[0:h, 0:int(w/2)] = 0
            mask_white[0:h, limit_right:w] = 0
            
            # Compute the mask moments
            M_y = cv2.moments(mask_yellow)
            M_w = cv2.moments(mask_white)
                
            if M_y["m00"] > 0: #yellow
            # Calculate x,y coordinate of center
                cX_y = int( M_y["m10"] / M_y["m00"] )
            else:
                cX_y = 10
                
            if M_w["m00"] > 0: #white
            # Calculate x coordinate of center
                cX_w = int( M_w["m10"] / M_w["m00"] )
            else:
                cX_w = 300 # valeure maximale a droite
                
            if (self.front == False): # Si il n'y a pas d'obstacles
                #if  ( cX_y < cX_w ):
                self.twist.linear.x = 0.2 # Commence a bouger
                if (cX_w < 220): # QUE LE BLANC TOURNE a gauche
                    self.twist.angular.z = np.min( [(230-cX_w)/10 , 3]) # Tourne a gauche
                elif cX_y > 80 : # Detection Jauce tourne a droite
                    self.twist.angular.z = -np.min([(cX_y-80)/10 , 3.5]) # Tourne a droite
                else : 
                    self.twist.angular.z = 0 # Tout droit
                #else :
                 #   self.twist.angular.z = 0.2
                  #  self.twist.linear.x = 0
            else : # Si il y a un obstacle   
                if ( cX_y > 20):
                    self.twist.angular.z = -0.4
                    self.twist.linear.x = 0     
                else :
                    self.twist.angular.z = 0.4
                    self.twist.linear.x = 0
                    
            self.cmd_vel_pub.publish(self.twist)
                    
            # show the image
     #       cv2.imshow("Yellow_line", mask_yellow)
     #       cv2.imshow("White_line", mask_white)
        #cv2.imshow("red_line", mask_red)
        cv2.waitKey(3)
        
        # Fonction appelee par le subscriber du laser
    def laser_callback(self,msg):
        if (self.i == 3): #If on est dans le 
            self.wall_func(msg) # Appelle la fonction de challenge 2
        elif (self.i == 5):
            self.third_problem(msg) # Dernier challenge, la boite
        else:
            self.obstacle_evit(msg) # le reste du trajet
    
    def obstacle_evit(self, msg):    
        ran = msg.ranges
        ran = np.array(ran)
        
        l = ran[55:135]
        f = ran[325:]    # front
        f = np.append(f,ran[:35])
        
        n_f = (f < 0.36).sum()
        

        
        if(n_f > 16):
            self.front = True
        else:
            self.front = False
        
        if (self.i < 10):
            n_l = (l < 0.2).sum()
            if (n_l > 10):
                self.left = True
            else:
                self.left = False
     #       print("left = ", n_l)
    #

    def get_rotation (self,msg):
        ''' 
        Responsible for getting robots orientations. 
        The orientation is first in a quartenion base, the its transormed to eulers base and updated
        Needed in 2nd and 3rd challenges.
        '''

        # If we are in challenge 2 or 3, then:
        if ( (self.i == 3) or (self.i== 5) ): 
            # gets orientation from msg input
            orientation_q = msg.pose.pose.orientation

            # Prepares inputs to euler_from_quaternion function
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

            # Converts quartenions base to euler's base, and updates the class' attributes
            (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def wall_func(self,msg):
        ''' 
        Responsible for going through the second challenge, characterized by a sharp U-shaped curve. 
        This curve is defined by two walls between which the robot will pass without ever touching them. 
        The algorithm is summarized in three main parts, one for when the robot is heading towards the curve, 
        another during the curve and finally leaving the curve and heading towards the exit. 
        In the first, the robot is oriented to walk parallel to the wall, with constant speed, in the second, 
        the robot perceives a difference between what it sees on its right and left side and then begins to turn
        with a constant z angular speed, and finally the robot repeats the first instruction but in the opposite direction, 
        leaving the "U".
        '''

        # takes the ranges attribute from the input instance, which will contain the distances found 
        # for all 360 degrees around the robot. It will be a list with 360 elements, each one beign a 
        # float representing its distance on a certain angle.
        ranges = msg.ranges

        # Gets right corner distances
        right_corner = ranges[0:90]

        # Gets left corner distances
        left_corner = ranges[270:360]

        # Gets the average of distances of both corners
        right_corner_dist = np.average(right_corner)
        lef_corner_dist = np.average(left_corner)
    
        # Sets default linear speed
        self.twist.linear.x = 0.25
    
        # Get difference from what it sees on its right and left side
        dif = right_corner_dist - lef_corner_dist
        
        # If the difference is big enough, start to turn
        if dif < -0.18:
            self.twist.linear.x = 0.15
            self.twist.angular.z = -0.55

        # Aligns the robot to the right direction, depending if its going in or out the "U"
        elif -0.78 > self.yaw > -2.35:
            self.twist.angular.z = -2*(self.yaw + 1.57)
        elif 0.78 < self.yaw < 2.35:
            self.twist.angular.z = -2*(self.yaw - 1.57)

        # Turns of angular velocity
        else:
            self.twist.angular.z = 0


    
        self.cmd_vel_pub.publish(self.twist)
    
    def find_biggest_sublist(self,big_list, size):
        '''
        Function responsible for finding the sublist whose sum of values is the largest within the main list. 
        Returns the position that this sublist is in the large list. Example: [12, 0 , 2, 3, 35, 45, 10, 6], 
        the largest 3-element sublist found would be [ 35, 45, 10], with position = 4. 
        It will be used in the third problem. The main list represents the range of distances seen by the robot, 
        the small list represents the space taken by the robot when walking in this direction.
        '''
        # Intializes current biggest sum
        biggest_sum = 0
        # Intializes current position of list with biggest sum
        biggest_sum_pos = 0
        # Intializes current list with biggest sum
        biggest_list = [0 for i in range(size)]

        # Loops overs big_list and finds the position of list with biggest sum
        for i,item in enumerate(big_list[:-size+1]):
            small_list = big_list[i:i+size]
            addition = sum(small_list)
            if addition > biggest_sum and max(small_list)-min(small_list)<1.5:
                biggest_sum = addition
                biggest_sum_pos = i
                biggest_list = small_list

        # Post a new message in the topic to which we are subscribed containing the new angular and linear velocities
        return biggest_sum_pos
    
    def third_problem(self,msg):
        '''
        Responsible for passing through the third part of the route. 
        The third part is a closed white box containing cylindrical obstacles inside. 
        With the car positioned at the entrance of the box, it starts moving forward. 
        The algorithm then finds, looking at a 90 degree range in front of the robot, 
        the direction where there is more room for it to go. if that direction is to your left, 
        a positive angular velocity is triggered so that the car aligns with that direction. 
        The same if this direction is to your right, this time with a negative angular velocity.
        '''

        # takes the ranges attribute from the input instance, which will contain the distances found 
        # for all 360 degrees around the robot. It will be a list with 360 elements, each one beign a 
        # float representing its distance on a certain angle.
        ranges = msg.ranges

        # Selects the bandwidth of distances that the robot will look in front of. 
        # In this case we will use 90 degrees only. If we took large values of width, 
        # the robot could get confused and leave the same place it entered, 
        # since that would be another direction with a lot of depth.
        big_list_size = 90
        
        # Selects the width of the distance band that the robot should use as a margin 
        # of error so that it does not hit obstacles close to the chosen direction.
        small_list_size = 25

        # Filters the initial list of distances using the parameters above
        front_list = ranges[-int(big_list_size/2):]+ranges[:int(big_list_size/2)]
    
        # Converts infinities to 100, since distances above a certain limit are considered infinity, 
        # which is not convenient here. 
        front_list = [100  if np.isinf(value) else value for value in front_list]
    
        # Gets the value of the angle whose depth is the greatest. 
        # 0 degrees = front of robot, pi/2 = left of robot, pi = behind robot and 3pi/2 = right of robot.
        biggest_sum_pos = self.find_biggest_sublist(front_list, small_list_size)

        # Corrects the position with the error margin to avoid hitting obstacles
        big_dist_list_center = biggest_sum_pos + round(small_list_size/2)

        # Figures out the difference between the position above and the front of the robot.
        ang_dif = big_dist_list_center - big_list_size/2

        # If the robot sees a sufficient number of infinities, it means that it is looking at the exit. 
        # It should then align with the exit direction.
        if len([value for value in front_list if value == 100]) > 2*small_list_size:
            self.twist.angular.z = -2*(self.yaw)
        else:
            # If the chosen direction is to the right, we activate the negative angular velocity, 
            # proportionaly to its difference from the front direction
            if ang_dif < 0:
                self.twist.angular.z = max(0.07*ang_dif,-1.5)

            # If the chosen direction is to the left, we activate the positive angular velocity, 
            # proportionaly to its difference from the front direction.
            else:
                self.twist.angular.z = min(0.07*ang_dif,1.5)
    
        # Sets a default linear speed
        self.twist.linear.x = 0.15
        # Post a new message in the topic to which we are subscribed containing the new angular and linear velocities
        self.cmd_vel_pub.publish(self.twist)


if __name__ == '__main__':

    try:    
        # Starts a new node
        rospy.init_node('projet2022', anonymous=True)
        
        follower = Trajet()
        
        # And then ... wait for the node to be terminated
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
