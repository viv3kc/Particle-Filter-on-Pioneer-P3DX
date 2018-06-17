#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import random
import time

turn_right = False
already_turning = False
sequence = []
index = 0

# Author: Tjark Siewers
# I have taken some code from the robotics 2017 

def callback( sensor_data ):
	global velocity_publisher
	global turn_right
	global already_turning
	global sequence
	global index
	#sensor_data (LaserScan data type) has the laser scanner data
	#vel_msg (Twist data type) created to control the base robot
        
    	vel_msg = Twist()
    
    	forward_index1 = len(sensor_data.ranges) / 3;
    	forward_index2 = 2 * len(sensor_data.ranges) / 3;
    	
    	range_scanned = min(sensor_data.ranges[forward_index1 : forward_index2])

	print "Minimum range value from 60 to 120 degrees angel: " + str(range_scanned)

    	#Since we are moving just in x-axis
    	if (range_scanned < 1.00): 
    		if already_turning == False:
    			already_turning = True
    			if (index < len(sequence)):
    				turn_right = sequence[index]
    			else:
    				index = 0
    				turn_right = sequence[index]
    			
    			index = index + 1
    				
    		
    				
    		vel_msg.linear.x = 0
    		vel_msg.linear.y = 0
    		vel_msg.linear.z = 0
    		vel_msg.angular.x = 0
    		vel_msg.angular.y = 0
    		if turn_right == True:
    			vel_msg.angular.z = -math.pi / 2
    			print "Turning right"
    		else:
    			vel_msg.angular.z = math.pi / 2
    			print "Turning left"
    			
    		velocity_publisher.publish(vel_msg)
    		
    	else:
    		# Move forward
    		print("Moving forward");
    		vel_msg.linear.x = 0.8
    		vel_msg.linear.y = 0
    		vel_msg.linear.z = 0
    		vel_msg.angular.x = 0
    		vel_msg.angular.y = 0
    		vel_msg.angular.z = 0
    		already_turning = False
    		velocity_publisher.publish(vel_msg)
    
        

if __name__ == '__main__':
	# Starts a new node
	
    global velocity_publisher
    global sequence
    print "Starting mover"
    sequence_file = open("/data/private/robot/robotics/src/sequence_mover/src/sequence.txt", "r");
    for line in sequence_file:
        print line
        if ("R" in line):
        	sequence.append(True)
        if ("L" in line):
        	sequence.append(False)
    
    print "Sequence is :" + str(sequence)
    sequence_file.close()
    rospy.init_node('mover', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('base_scan', LaserScan, callback)
    print "Waiting for Caller"
    
    
    rospy.spin()
