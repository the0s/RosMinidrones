#!/usr/bin/env python
# license removed for brevity
import rospy
import traceback
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from termcolor import colored
from kios.msg import MamboState, DroneInput
import re

def fastInput(inTup):
	temp = DroneInput()
	temp.roll=inTup(0)
	temp.pitch=inTup(1)
	temp.yaw=inTup(2)
	temp.vertical=inTup(3)
	temp.duration=inTup(4)

#fl = open("/home/the0s/catkin_ws/src/kios/test.txt",'w')

mamboStates=[None,None,None]
mambos = ["e014cd613dd1", "e014a8473dbe", "e01486a13dc0"]

def pubAll(pubs, cmd):
	for k in pubs:
		k.publish(cmd)

def pubdiff(pubs, cmd):
	for k,j in zip(pubs,cmd):
		k.publish(j)		

def rosCallback(data):
	for n,ids in enumerate(mambos):
		if ids == data.header.frame_id[6:]:
			mamboStates[n] = data.flying_state 
	#if mambos[0] == data.header.frame_id[6:]:
	#	fl.write("%s, %.4f, %.4f, %.4f,%.4f, %.4f\n" %(data.flying_state, data.velocity.x, data.velocity.y, data.velocity.z, data.altitude, data.header.stamp.to_sec()))

def checkAll(state):
	status = True
	for n,_ in enumerate(mambos):
		if state != mamboStates[n]:
			status &= False	
	return status

def hook():
	print("Shutting down")
	rospy.signal_shutdown("Shutdown hook")

def is_exit():
	if rospy.is_shutdown():
		sys.exit()	

def checkHover(wait):
	wait.sleep()
	print("waiting all to hover...")
	while(not checkAll('hovering') and not rospy.is_shutdown()):
		wait.sleep()
	is_exit()

'''
Main Function
'''
if __name__ == '__main__':
	try:
		rospy.init_node('drone_test', anonymous=True)
		rospy.on_shutdown(hook)
		mamboAddr = rospy.get_param('~droneId', "e0:14:cd:61:3d:d1")
		mamboAddrSub = re.sub(':', '', mamboAddr)
		mamboName = "drone_" + mamboAddrSub		
		publisher=rospy.Publisher("/"+ mamboName + "/input", DroneInput, queue_size=10)
		
		pubs = []
		subs = []
		for k in mambos:
			pubs.append(rospy.Publisher("/drone_"+ k + "/sub", String, queue_size=10))
			subs.append(rospy.Subscriber("/drone_"+ k + "/pub", MamboState, rosCallback))
		
		wait = rospy.Rate(1.0/3)
		rate = rospy.Rate(1.0/5)
		once = True	
		while not rospy.is_shutdown():
			if once:
				# temp = DroneInput()
				# temp.roll=0
				# temp.pitch=0
				# temp.yaw=0
				# temp.vertical=100
				# temp.duration=1
				# publisher.publish(temp)
				# print("/"+ mamboName + "/input")
				# p1.publish('takeoff')
				# p2.publish('takeoff')
				print("waiting landed state...")
				while(not checkAll('landed') and not rospy.is_shutdown()):
					wait.sleep()
				is_exit()
				print('Takeoff')						
				pubAll(pubs,'takeoff')

				checkHover(wait)
				print('flip')
				pubAll(pubs,'flipleft')

				checkHover(wait)
				print('flip')
				pubAll(pubs,'flipright')
				wait.sleep()

				checkHover(wait)
				print('flip')
				pubAll(pubs,'flipfront')
				wait.sleep()	

				checkHover(wait)
				print('indivudual moves')
				pubdiff(pubs,['flipfront', 'flipright','flipleft'])
				wait.sleep()	

				checkHover(wait)
				print('indivudual moves')
				pubdiff(pubs,['flipback', 'flipleft','flipright'])
				wait.sleep()					

				checkHover(wait)
				print('Land')	
				pubAll(pubs,'land')

				once=False
			rate.sleep()
	except rospy.ROSInterruptException: 
		pass
	except SystemExit:
		pass

 