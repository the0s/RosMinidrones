#!/usr/bin/env python
# license removed for brevity
import rospy
import traceback
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__))+"/pyparrot")
from std_msgs.msg import String,Header
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from Mambo import Mambo
from termcolor import colored
from kios.msg import MamboState, DroneInput
import re
import tf
import numpy as np
from multiprocessing import Process


# rotate vector v1 by quaternion q1 
def qv_mult(q1, v1):
    q2 = list(v1)
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2), 
        tf.transformations.quaternion_conjugate(q1)
    )[:3]

class DroneNode(object):
	def __init__(self, mamboAddr ="e0:14:cd:61:3d:d1",initalPos=np.array([0.0,0.0,0.0])):    
		try:
			# initialise variables
			self.reqVarInit()
			# get drone unique name
			mamboAddrSub = re.sub(':', '', mamboAddr)
			self.mamboName = "drone_" + mamboAddrSub
			self.position=initalPos
			# setting up drone's bluetooth and connect
			print colored("* Setting Mambo: %s ..." %mamboAddr,'magenta')	
			self.mambo = Mambo(mamboAddr, use_wifi=False)
			print colored("* Trying to connect to %s ..." %mamboAddr,'magenta')
			success = self.mambo.connect(num_retries=3)
			print colored("Connected to %s : %s" % (mamboAddr,success),'green')
			# check connection!
			self.mambo.smart_sleep(2)
			self.mambo.ask_for_state_update()
			self.mambo.smart_sleep(2)	
			print colored("** Ready **", 'green')					
		except:
			print traceback.format_exc()
			rospy.loginfo(traceback.format_exc())	
			sys.exit()				

	def reqVarInit(self):
		self.subscriber = None
		self.subscriber2 = None
		self.publisher = None
		self.publisher_range = None
		self.publisher_odom = None
		self.droneSem = False
		self.droneSemTime = rospy.get_rostime()
		self.moveDroneData=None
		self.inputDroneData=None	
		self.tf_broadcaster = tf.TransformBroadcaster()
		self.timeCurrent=rospy.get_rostime()
		self.timeLast = rospy.get_rostime()

	def __del__(self):
		self.mambo.disconnect()
		print colored("** Disconnect Drone **", 'red')

	def update(self):		
		#print("update")
		# Get new state and publish
		dataMambo = self.mambo.sensors
		#print(dataMambo)
		self.publishState(dataMambo)

		# check if new data to move drone
		if self.moveDroneData != None:
			print('moveDrone')
			self.moveDrone(self.moveDroneData)
			self.moveDroneData = None

		# check if new input data to move drone
		if self.inputDroneData != None:
			print('inputMoveDrone')
			self.inputMoveDrone(self.inputDroneData)
			self.inputDroneData = None


	def publishState(self,dataMambo):
		temp = MamboState()
		temp.header.stamp = rospy.get_rostime()
		temp.header.frame_id = self.mamboName
		temp.battery = dataMambo.battery
		temp.flying_state = dataMambo.flying_state
		temp.velocity.x = dataMambo.speed_x
		temp.velocity.y = dataMambo.speed_y
		temp.velocity.z = dataMambo.speed_z
		temp.velocity_ts = dataMambo.speed_ts
		temp.altitude = dataMambo.altitude
		temp.altitude_ts = dataMambo.altitude_ts
		temp.quaternion.w = dataMambo.quaternion_w
		temp.quaternion.x = dataMambo.quaternion_x
		temp.quaternion.y = dataMambo.quaternion_y
		temp.quaternion.z = dataMambo.quaternion_z
		temp.quaternion_ts = dataMambo.quaternion_ts
		# create position estimate
		self.timeCurrent = temp.header.stamp
		self.approximatePos(temp)
		#
		self.publishRos(temp)
		self.timeLast=self.timeCurrent


	def approximatePos(self,state):
		dt = (self.timeCurrent - self.timeLast).to_sec()
		deltas = qv_mult([state.quaternion.x,state.quaternion.y,state.quaternion.z,state.quaternion.w],\
					[state.velocity.x*dt,state.velocity.y*dt,-state.velocity.z*dt])
		self.position += deltas		

	'''
	Ros Callback
	'''
	def rosCallback(self, data):
		try:
			print(data)
			self.moveDroneData = data.data
		except:
			print traceback.format_exc()
			rospy.loginfo(traceback.format_exc())

	'''
	Ros Input Callback
	'''
	def callbackInput(self, data):
		try:
			print(data)
			self.inputDroneData = data
		except:
			print traceback.format_exc()
			rospy.loginfo(traceback.format_exc())		


	def inputMoveDrone(self,data):
		self.mambo.fly_direct(roll=data.roll, \
							pitch=data.pitch, \
							yaw=data.yaw, \
							vertical_movement=data.vertical, \
							duration=data.duration\
							)						
	
	def moveDrone(self,dat):
		d = rospy.Duration.from_sec(5)
		print(rospy.get_rostime() - self.droneSemTime > d)
		
		if (not self.droneSem) or (rospy.get_rostime() - self.droneSemTime > d): # get semaphore when executive commands
			self.droneSem = True
			self.droneSemTime = rospy.get_rostime()			
		else:
			print colored("Previous Command Still running",'red')
			return not self.droneSem
		
		print('Going for %s'%dat)
		if (dat == "takeoff"):
			self.mambo.safe_takeoff(5)
		elif self.mambo.sensors.flying_state != "flying" or self.mambo.sensors.flying_state != "hovering":	
			if dat in ['flipleft','flipright','flipfront','flipback']:				
				print(dat[4:])
				self.mambo.flip(direction=dat[4:])
			elif(dat =="land"):
				self.mambo.safe_land(5)
			elif(dat == "trick"):
				print("Flying direct: going around in a circle (yes you can mix roll, pitch, yaw in one command!)")
				self.mambo.fly_direct(roll=25, pitch=0, yaw=50, vertical_movement=0, duration=1)
		print("Finished %s" % dat)
		self.droneSem = False
		return not self.droneSem

	'''
	Publishers
	'''
	def publishRos(self, data):
		try:		
			# publish mambo state as it is
			self.publisher.publish(data)				
			# publish laser scan (altitude?)
			self.publishRange(data)
			# publish odometry
			self.publishOdom(data)
		except:
			rospy.loginfo(traceback.format_exc())

	def publishRange(self,data):
		range_sensor = Range()
		range_sensor.header.stamp = data.header.stamp
		range_sensor.header.frame_id = data.header.frame_id + "_range"
		range_sensor.radiation_type = range_sensor.ULTRASOUND
		range_sensor.field_of_view = 1.39
		range_sensor.min_range = 0.02
		range_sensor.max_range = 4.0
		range_sensor.range = data.altitude
		self.tf_broadcaster.sendTransform((0.0, 0.0, 0.0),
						(0.0, 0.707, 0.0, 0.707),
						data.header.stamp,
						range_sensor.header.frame_id,
						data.header.frame_id)		
		self.publisher_range.publish(range_sensor)

	def publishOdom(self,data):	
		odom = Odometry() # IMPROVE TODO
		odom.header.stamp = data.header.stamp
		odom.header.frame_id = data.header.frame_id + "_odom"
		odom.child_frame_id = data.header.frame_id		
		odom.pose.pose.position.x = self.position[0]
		odom.pose.pose.position.y = self.position[1]
		odom.pose.pose.position.z = self.position[2]
		odom.pose.pose.orientation = data.quaternion
		odom.twist.twist.linear = data.velocity
		# first, we'll publish the transform over tf
		self.tf_broadcaster.sendTransform(
		    (odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z),
		    (data.quaternion.x,data.quaternion.y,data.quaternion.z,data.quaternion.w),
		    odom.header.stamp,
		   	odom.child_frame_id,
		    odom.header.frame_id
		)			
		#publish odom
		self.publisher_odom.publish(odom)

	'''
	Setup listeners/subscribers
	Main loop!
	'''	
	def listener(self):
		self.subscriber=rospy.Subscriber("/"+ self.mamboName + "/input", DroneInput, self.callbackInput)
		self.subscriber2=rospy.Subscriber("/"+ self.mamboName + "/sub", String, self.rosCallback)
		self.publisher=rospy.Publisher("/"+ self.mamboName + "/pub", MamboState, queue_size=10)
		self.publisher_range=rospy.Publisher("/"+ self.mamboName + "/range", Range, queue_size=10)
		self.publisher_odom =rospy.Publisher("/"+ self.mamboName + "/odom", Odometry, queue_size=10)

		rateHz = 10
		rate = rospy.Rate(rateHz)  
						
		while not rospy.is_shutdown(): 			
			self.update()			
			self.mambo.ask_for_state_update()	
			self.mambo.smart_sleep(1/rateHz)
			#rate.sleep()

'''
Main Function
'''
if __name__ == '__main__':
	try:
		rospy.init_node('drone_node', anonymous=True)
		private_param = rospy.get_param('~droneId', "e0:14:cd:61:3d:d1")
		delay = rospy.get_param('~delay', 0)
		if delay != 0:
			print colored("A delay of %i seconds is set for drone %s" %(delay,private_param),'red')
			rospy.Rate(1.0/delay).sleep()
		droneNode = DroneNode(mamboAddr = private_param)
		droneNode.listener()
	except rospy.ROSInterruptException: 
		pass

 