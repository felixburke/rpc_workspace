#!/usr/bin/env python
import roslib
roslib.load_manifest("rpc_game_client")
import rospy
import socket
import os
import sys
from threading import Thread
from rpc_game_client.msg import Alive, Score, GameState
from rpc_game_client.srv import ClientScore, PlayerScore
from geometry_msgs.msg import Twist

class GameClient:
	def __init__(self):
		rospy.on_shutdown(self.exit)
		self.hostname = socket.gethostname()
		try:
			self.mytag = int(os.environ['RPC_TAG_ID'])
		except KeyError, e:
			self.mytag = 0
		rospy.init_node("game_client_%s" % self.hostname)
		self.alive_pub = rospy.Publisher('/rpc_game/alive', Alive, queue_size=0)
		rospy.Subscriber('/rpc_game/gamestate', GameState, self.game_state)
		# Send Alive message every 5 seconds
		alive_thread = Thread(target=self.alive)
		alive_thread.start()
		# ServiceMiddleLayer
		self.score_service = rospy.Service('/rpc_score', PlayerScore, self.rpc_score)
		# Block 10 seconds between submission
		self.interval = rospy.Duration(10)
		self.last_score = rospy.Time(0)
		# Blocking Message/State & Blocker
		self.block_msg = Twist()
		self.block_msg.linear.x = 0
		self.block_msg.linear.y = 0
		self.block_msg.linear.z = 0
		self.block_msg.angular.x = 0
		self.block_msg.angular.y = 0
		self.block_msg.angular.z = 0
		self.blocked = True
		block_thread = Thread(target=self.block)
		block_thread.start()
		rospy.loginfo("[GameClient@%s] Started GameClient with tag ID: %s" % (self.hostname, self.mytag))
		self.run()

	def run(self):
		while not rospy.is_shutdown():
			rospy.spin()

	def rpc_score(self, request):
		time_since_last_score = (rospy.Time.now() - self.last_score)
		if (time_since_last_score < self.interval):
			time_to_sleep = self.interval - time_since_last_score
			rospy.logerr("[GameClient@%s] Requested score too early. Blocking for %s seconds" % (self.hostname, time_to_sleep.to_sec()))
			rospy.sleep(time_to_sleep)
			raise rospy.ServiceException("Could not process scoring request")
		else:
			try:
				score_master = rospy.ServiceProxy('/rpc_score_master', ClientScore)
				rospy.wait_for_service('/rpc_score', timeout=1)
				response_master = score_master(request.header, self.hostname, request.image, request.camerainfo)
				rospy.loginfo("[GameClient@%s] Scored: %s (%s+%s+%s) @ %s" % (self.hostname, response_master.score.total, response_master.score.align, response_master.score.center, response_master.score.distance, response_master.hostname_hit))
				response = Score()
				response.header.stamp = rospy.Time.now()
				response = response_master.score
				self.last_score = rospy.Time.now()
				return response
			except rospy.ServiceException, e:
				rospy.logerr("[GameClient@%s] Service call failed: %s" % (self.hostname, e))

	def game_state(self, data):
		blocked = False
		if data.gamestate == "waiting":
			blocked = True
		if self.hostname in data.playernames:
			if data.playerstates[data.playernames.index(self.hostname)] > 1:
				blocked = True
		self.blocked = blocked
		if self.blocked:
			rospy.logwarn("[GameClient@%s] blocked by GameMaster" % self.hostname)

	def alive(self):
		rospy.sleep(1)
		while not rospy.is_shutdown():
			msg = Alive()
			msg.header.stamp = rospy.Time.now()
			msg.hostname = self.hostname
			msg.tagid = self.mytag
			self.alive_pub.publish(msg)
			rospy.sleep(5)

	def block(self):
		rospy.sleep(1)
		self.block_pub = False
		while not rospy.is_shutdown():
			if self.blocked:
				if not self.block_pub:
					self.block_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
				self.block_pub.publish(self.block_msg)
			else:
				self.block_pub = False
			rospy.sleep(0.1)

	def exit(self):
		rospy.logerr("[GameClient@%s] Shutdown requested!" % self.hostname)

if __name__ == '__main__':
	try:
		game_client = GameClient()
	except rospy.ROSInterruptException:
		pass