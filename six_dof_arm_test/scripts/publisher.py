#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose

def publisher():

	pub = rospy.Publisher('pose', Pose, queue_size=10)
	rospy.init_node('position_publisher', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	#while not rospy.is_shutdown():
		
	pos=Pose()
	pos.position.x=0.025969
	pos.position.y=-0.0066206
	pos.position.z=0.5839
	pos.orientation.w=1
	pos.orientation.x=-0.00092018
	pos.orientation.y=6.5196e-05
	pos.orientation.z=0.0002036
	
	pub.publish(pos)   #publishes the position to topic Pose
	rospy.loginfo(pos) #prints the position
	rate.sleep() # ensures freq=10 hz

if __name__ == '__main__':
	try:
		publisher()

	except rospy.ROSInterruptException:
		pass
