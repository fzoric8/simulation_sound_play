#!/usr/bin/env python2

import time
import sys
import rospy
import rosbag
import numpy as np
from math import sqrt, pow
from geometry_msgs.msg import Pose, Point, Twist
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from nav_msgs.msg import Odometry, Path

class GenerateSound():

	def __init__(self):

		# 2 possible control modes --> a) reactive and b) interactive
		self.playback_mode = "reactive" #"interactive"

		self.rate = rospy.Rate(1)

		# Subscribers
		self.cmd_sub = rospy.Subscriber('/uav/pose_ref', Pose, self.cmd_cb, queue_size=1)
		self.odom_sub = rospy.Subscriber('/uav/odometry', Odometry, self.odom_cb, queue_size=1)
		self.cmd_reciv = False
		self.odom_reciv = False

		# Publishers
		self.sound_pub = rospy.Publisher('/robotsound', SoundRequest, queue_size = 1)
		self.pose_ref_pub = rospy.Publisher('/uav/pose_ref', Pose, queue_size=1)

		# Read rosbag
		if self.playback_mode == "interactive": 
			self.inbag_filename = "/home/developer/catkin_ws/src/simulation_sound_play/scripts/2021-05-09-09-49-56.bag"
			self.poses = []
			self.read_rosbag()

	def cmd_cb(self, data):
		self.cmd_reciv = True
		self.cmd_pose = Pose()
		self.cmd_pose = data
		
	def odom_cb(self, data):
		self.odom_reciv = True
		self.current_pose = Pose()
		self.current_pose.position = data.pose.pose.position
		self.current_pose.orientation = data.pose.pose.orientation

	def read_rosbag(self):
		for (topic, msg, t) in rosbag.Bag(self.inbag_filename, 'r').read_messages():
			if(topic == '/path'):
				for p_ in msg.poses:
					self.poses.append(p_)

	def check_distance(self, pose1, pose2):
		self.distance = sqrt(pow(pose1.position.x - pose2.position.x, 2) + pow(pose1.position.y - pose2.position.y, 2) + pow(pose1.position.z - pose2.position.z, 2))
		return self.distance	

	def play_sound(self, sound_cmd):
		
		soundMsg = SoundRequest()
		soundMsg.sound = -3
		soundMsg.volume = 1.0 
		soundMsg.arg2 = 'voice_kal_diphone'
		
		if (sound_cmd != ""):
			# int8 PLAY_ONCE=1; int8 PLAY_START=2
			soundMsg.command = 1
			soundMsg.arg = str(sound_cmd)
			self.sound_pub.publish(soundMsg)

		else:
			# int8 PLAY_STOP=0
			soundMsg.command = 0
			soundMsg.arg = str(sound_cmd)

	def compare_difference(self):

		diff_x = self.current_pose.position.x - self.cmd_pose.position.x
		diff_y = self.current_pose.position.y - self.cmd_pose.position.y 
		diff_z = self.current_pose.position.z - self.cmd_pose.position.z

		rospy.logdebug("diff_x: {}\t diff_y: {}\t diff_z: {}\t".format(diff_x, diff_y, diff_z))

		index = np.argmax([abs(diff_x), abs(diff_y), abs(diff_z)])

		# To be percieved as pose change
		min_val = 0.02

		if index == 0 and (abs(diff_x) > min_val): 
			if diff_x > 0: 
				sound = "go backward"
			else:
				sound = "go forward"
		elif index == 1 and (abs(diff_y) > min_val):
			if diff_y > 0: 
				sound = "go left"
			else:
				sound = "go right"
		elif index == 2 and (abs(diff_z) > min_val):
			if diff_z > 0:
				sound = "go down"
			else:
				sound = "go up"
		else:
			sound = ""

		rospy.logdebug("Sound to reproduce is: {}".format(sound))

		return sound

	def run(self):
		while not rospy.is_shutdown():
			rospy.loginfo("Running sound playback!")
			
			if(self.playback_mode == "reactive"):
				if (self.odom_reciv and self.cmd_reciv):
					sound_cmd = self.compare_difference()
					self.play_sound(sound_cmd)
					self.rate.sleep()
				else:
					rospy.loginfo("Didn't recieved cmd or odom msg!")

			if (self.playback_mode == "interactive"):
				if self.odom_reciv:
					for x in self.poses:
						pose_cmd = Pose()
						pose_cmd.position = x.pose.position
						pose_cmd.orientation = x.pose.orientation
						self.pose_ref_pub.publish(pose_cmd)

						while self.check_distance(self.current_pose, x.pose) > 0.05:							
							#sound_cmd = self.compare_difference()
							#self.play_sound(sound_cmd)

							rospy.loginfo("Waiting UAV to reach cmd state!")

				# use bag for now


				
if __name__ == '__main__':
	rospy.init_node('generate_ref_sound', anonymous = True, log_level=rospy.DEBUG)
	time.sleep(5)
	try:
		gS = GenerateSound()
		gS.run()
	except rospy.ROSInterruptException:pass
	
