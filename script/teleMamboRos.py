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
import sys, select, termios, tty

# SET the bluetooth ids of the drones to control!
mambos = ["e014cd613dd1", "e014a8473dbe", "e01486a13dc0"]

mamboStates=[None,None,None]

def fastInput(inTup):
	temp = DroneInput()
	temp.roll=inTup[0]
	temp.pitch=inTup[1]
	temp.yaw=inTup[2]
	temp.vertical=inTup[3]
	temp.duration=inTup[4]
	return temp

def pubAll(pubs, cmd,swarm=-1):
	if swarm == -1:
		for k in pubs:
			k.publish(cmd)
	else:
		pubs[swarm].publish(cmd)

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

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def controls():
	print("Control Using:\n Throttle: w s \n yaw: a d \n pitch: i k \n roll: j l\n Change aggresiveness: - + \n Select drone: 0 \n Special moves: 2-5, Land:6, Takeoff: 1")
 

'''
Main Function
'''
if __name__ == '__main__':
	try:
		settings = termios.tcgetattr(sys.stdin)
		rospy.init_node('drone_test', anonymous=True)
		rospy.on_shutdown(hook)
		
		pubs = []
		pubInput =[]		
		subs = []
		for k in mambos:
			pubs.append(rospy.Publisher("/drone_"+ k + "/sub", String, queue_size=10))
			pubInput.append(rospy.Publisher("/drone_"+ k + "/input", DroneInput, queue_size=10))
			subs.append(rospy.Subscriber("/drone_"+ k + "/pub", MamboState, rosCallback))
		
		duration = 0.1
		rate = rospy.Rate(1/duration)
		speed=50
		swarm=-1
		controls()
		while not rospy.is_shutdown():
			key = getKey()
			print('Key: '+key)
			if key == '\x03':
				sys.exit()
			elif key =='1':
				print('Takeoff')						
				pubAll(pubs,'takeoff',swarm)
			elif key =='2':
				print('flip')
				pubAll(pubs,'flipleft',swarm)	
			elif key =='3':
				print('flip')
				pubAll(pubs,'flipright',swarm)	
			elif key =='4':
				print('flip')
				pubAll(pubs,'flipback',swarm)	
			elif key =='5':
				print('flip')
				pubAll(pubs,'flipfront',swarm)															
			elif key =='6':
				print('Land')	
				pubAll(pubs,'land',swarm)
			elif key == '+':
				speed+=1
				print('Speed: %i'%speed)
			elif key == '-':
				speed-=1
				print('Speed: %i'%speed)
			elif key =='w':
				temp=fastInput((0,0,0,speed,duration))
				pubAll(pubInput,temp,swarm)
			elif key =='s':
				temp=fastInput((0,0,0,-speed,duration))
				pubAll(pubInput,temp,swarm)			
			elif key =='a':
				temp=fastInput((-speed,0,0,0,duration))
				pubAll(pubInput,temp,swarm)
			elif key =='d':
				temp=fastInput((speed,0,0,0,duration))
				pubAll(pubInput,temp,swarm)
			elif key =='i':
				temp=fastInput((0,speed,0,0,duration))
				pubAll(pubInput,temp,swarm)
			elif key =='k':
				temp=fastInput((0,-speed,0,0,duration))
				pubAll(pubInput,temp,swarm)
			elif key =='j':
				temp=fastInput((0,0,speed,0,duration))
				pubAll(pubInput,temp,swarm)
			elif key =='l':
				temp=fastInput((0,0,-speed,0,duration))
				pubAll(pubInput,temp,swarm)
			elif key =='0':
				value=raw_input('Select Drone (-1 for all):')
				try:
					swarm = int(value)
				except:
					swarm= -1
				print("Control %i"%swarm)
			rate.sleep()
	except rospy.ROSInterruptException: 
		pass
	except SystemExit:
		pass

 