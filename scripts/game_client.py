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
from kobuki_msgs.msg import MotorPower, Led

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
		self.motor_power_pub = rospy.Publisher('/mobile_base/commands/motor_power', MotorPower, queue_size=1, latch=True)
		self.led_pub = rospy.Publisher('mobile_base/commands/led2', Led, queue_size=1, latch=True)
		# Send Alive message every 5 seconds
		alive_thread = Thread(target=self.alive)
		alive_thread.start()
		rospy.loginfo("[GameClient@%s] Started Alive thread" % (self.hostname))
		# ServiceMiddleLayer
		self.score_service = rospy.Service('/rpc_score', PlayerScore, self.rpc_score)
		# Block 10 seconds between submission
		self.interval = rospy.Duration(10)
		self.last_score = rospy.Time(0)
		# Blocking Message/State & Blocker
		self.blocked = False
		self.motor_msg = MotorPower()
		self.motor_msg.state = self.motor_msg.ON
		self.motor_power_pub.publish(self.motor_msg)
		self.led_msg = Led()
		self.led_msg.value = self.led_msg.GREEN
		self.led_pub.publish(self.led_msg)
		rospy.loginfo("[GameClient@%s] Started Block thread" % (self.hostname))
		rospy.loginfo("[GameClient@%s] Started GameClient with tag ID: %s" % (self.hostname, self.mytag))
		self.run()

	def run(self):
		while not rospy.is_shutdown():
			rospy.spin()

	def rpc_score(self, request):
		time_since_last_score = (rospy.Time.now() - self.last_score)
		if self.blocked:
			raise rospy.ServiceException("Player is blocked by GameMaster")
		if (time_since_last_score < self.interval):
			time_to_sleep = self.interval - time_since_last_score
			rospy.logerr("[GameClient@%s] Requested score too early. Blocking for %s seconds" % (self.hostname, time_to_sleep.to_sec()))
			rospy.sleep(time_to_sleep)
			raise rospy.ServiceException("Could not process scoring request")
		else:
			try:
				score_master = rospy.ServiceProxy('/rpc_score_master', ClientScore)
				rospy.wait_for_service('/rpc_score_master', timeout=1)
				response_master = score_master(self.hostname, request.image, request.camerainfo)
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
		self.motor_msg.state = self.motor_msg.ON
		self.led_msg.value = self.led_msg.GREEN

		if data.gamestate == "paused":
			blocked = True
			self.motor_msg.state = self.motor_msg.OFF
			self.led_msg.value = self.led_msg.RED
		if self.hostname in data.playernames:
			if data.playerstates[data.playernames.index(self.hostname)] > 1:
				blocked = True
				self.motor_msg.state = self.motor_msg.OFF
				self.led_msg.value = self.led_msg.RED
		self.blocked = blocked
		self.motor_power_pub.publish(self.motor_msg)
		self.led_pub.publish(self.led_msg)
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

	def exit(self):
		self.motor_msg.state = self.motor_msg.ON
		self.led_msg.value = self.led_msg.BLACK
		self.motor_power_pub.publish(self.motor_msg)
		self.led_pub.publish(self.led_msg)
		rospy.logerr("[GameClient@%s] Shutdown requested!" % self.hostname)

if __name__ == '__main__':
	try:
		game_client = GameClient()
	except rospy.ROSInterruptException:
		pass
