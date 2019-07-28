import sys
import rospy
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
import thread
import time
from std_msgs.msg import Int8
from utils.geometry import Vector2D
import math 
from utils.config import *
from utils.geometry import *
RATIO = MAX_BALL_SPEED*1.0/MAX_BOT_SPEED

# UTILITY FILE FOR FUZZY_ULTRA
class pass_analysis(object):
	
	def __init__(self, bot_id, state):
		self.bot_id = bot_id

	def if_intercept_poss( self, state, rival, reciever_id, in_line_dist, perp_dist, sep, in_line_dist2):
		x = in_line_dist
		y = perp_dist
		if x < sep and in_line_dist2 < sep:
			if (x*1.0/y) < RATIO:
				if sep*sep < (1-1.0/(RATIO**2))*y*y:
					if sep*(1-1.0/(RATIO**2)) < x:
						print "/"*50
						return 0
					else:
						return 1
				else:
					return 1
			else :
				return 1
		else :
			return 1

	def line( self, opp, bot_id, reciever_id, state):
		if (state.homePos[bot_id].x - state.homePos[reciever_id].x) != 0:
			slope = (state.homePos[bot_id].y - state.homePos[reciever_id].y)*1.0/(state.homePos[bot_id].x - state.homePos[reciever_id].x)
		else:
			return math.fabs((state.homePos[bot_id].x-state.awayPos[opp].x)*1.0/(state.homePos[bot_id].y - state.homePos[reciever_id].y))
		up = math.fabs((state.awayPos[opp].y - state.homePos[bot_id].y) - slope*(state.awayPos[opp].x - state.homePos[bot_id].x))
		low = (1 + slope**2)**0.5
		return up*1.0/low		
	
	def intercept_ratio(self, state, reciever_id):
		rivals = [x for x in range(6)]
		min_ratio = 9999999
		index = -1
		botPos = Vector2D( state.homePos[self.bot_id].x, state.homePos[self.bot_id].y)
		for player in rivals:
			distance = botPos.dist(Vector2D(state.awayPos[player].x, state.awayPos[player].y))
			dist = self.line(player, self.bot_id, reciever_id, state)
			in_line_dist = (distance**2-dist**2)**(0.5)
			if in_line_dist*1.0/dist < min_ratio:
				min_ratio = in_line_dist*1.0/dist
				index = player
			else:
				continue
		return min_ratio

	def nearness_ratio( self, state, reciever_id):
		rivals = [x for x in range(6)]
		min_ratio = 9999999
		index = -1
		bot_Pos = Vector2D( state.homePos[reciever_id].x, state.homePos[reciever_id].y)
		passerPos = Vector2D( state.homePos[self.bot_id].x, state.homePos[self.bot_id].y)
		distance = bot_Pos.dist(passerPos)
		for player in rivals:
			dist = bot_Pos.dist(Vector2D(state.awayPos[player].x, state.awayPos[player].y))
			if distance*1.0/dist < min_ratio:
				min_ratio = distance*1.0/dist
				index = player
			else:
				continue
		return min_ratio
		
	def ratios( self, state, reciever_id):
		rivals = [x for x in range(6)]
		intercept_ratio = -1
		nearness_ratio = -1
		#index = -1
		botPos = Vector2D( state.homePos[self.bot_id].x, state.homePos[self.bot_id].y)
		recieverPos = Vector2D( state.homePos[reciever_id].x, state.homePos[reciever_id].y)
		sep = botPos.dist(recieverPos)
		print "#"*50
		print "PLAYER: ", reciever_id, "     in_line_dist     sep    sep_from_rival      perp_dist     in_line_dist2"
		for player in rivals:
			sep_from_rival = recieverPos.dist(Vector2D(state.awayPos[player].x, state.awayPos[player].y))
			distance = botPos.dist(Vector2D(state.awayPos[player].x, state.awayPos[player].y))
			distance2 = recieverPos.dist(Vector2D(state.awayPos[player].x, state.awayPos[player].y))
			dist = self.line(player, self.bot_id, reciever_id, state)
			in_line_dist = (distance**2-dist**2)**(0.5)
			in_line_dist2 = (distance2**2-dist**2)**(0.5)
			print "DETAILS--> ",player, in_line_dist, sep, sep_from_rival, dist, in_line_dist2
			if in_line_dist*1.0/dist > intercept_ratio and in_line_dist < sep and in_line_dist2 < sep:
				intercept_ratio = in_line_dist*1.0/dist
				#index = player
			if sep*1.0/sep_from_rival > nearness_ratio:
				nearness_ratio = sep*1.0/sep_from_rival
		if nearness_ratio > 400.0/27:
			nearness_ratio = 400.0/27
		# if self.if_intercept_poss( self, state, rival, reciever_id, in_line_dist, perp_dist, sep, in_line_dist2) == 0:
		# 	intercept_ratio = 0
		# 	return intercept_ratio, nearness_ratio
		if intercept_ratio*intercept_ratio < RATIO*RATIO-1:
			intercept_ratio = 0
			return intercept_ratio, nearness_ratio
		if intercept_ratio == -1:
			print "GOTCHA...."
			intercept_ratio = 0
			return intercept_ratio, nearness_ratio
		if intercept_ratio > 400.0/27:
			intercept_ratio = 400.0/27
		return intercept_ratio, nearness_ratio

	def prod( self, x, y, a):
		global RATIO
		return (RATIO-(x*1.0/y))*( y**2 + (x-a)**2 - (a*1.0/RATIO)**2)

	def evr_params( self, state, reciever_id, bp, rp, sp):
		global RATIO
		oppteam = [x for x in xrange(6)]
		botPos = bp
		recieverPos = rp
		sep = sp
		nearness_ratio = -1
		intercept_ratio = -1
		count1 = 0
		count2 = 0
		print "#"*50
		print "PLAYER: ", reciever_id, "     in_line_dist     sep    sep_from_rival      perp_dist     in_line_dist2"
		for player in oppteam:
			distance = botPos.dist(Vector2D(state.awayPos[player].x, state.awayPos[player].y))
			distance2 = recieverPos.dist(Vector2D(state.awayPos[player].x, state.awayPos[player].y))
			if distance2 > HALF_FIELD_MAXX*0.5 and distance > HALF_FIELD_MAXX*0.75:
				continue
			dist = self.line(player, self.bot_id, reciever_id, state)
			in_line_dist = (distance**2-dist**2)**(0.5)
			in_line_dist2 = (distance2**2-dist**2)**(0.5)
			if in_line_dist > sep or in_line_dist2 > sep:
				if nearness_ratio < sep*1.0/distance2:
					nearness_ratio = sep*1.0/distance2
				if sep*1.0/distance2 > RATIO*0.75:
					count1 = count1 + 1
			else:
				if intercept_ratio < in_line_dist*1.0/dist:
					intercept_ratio = in_line_dist*1.0/dist
				# if self.prod(in_line_dist, dist, sep) < 0:
				# 	count2 = count2 + 1
			print "DETAILS--> ",player, in_line_dist, sep, distance2, dist, in_line_dist2
		if intercept_ratio == -1 and nearness_ratio == -1:
			return 0.1, 0
		if intercept_ratio > 400.0/27:
			intercept_ratio = 400.0/27
		if nearness_ratio > 400.0/27:
			nearness_ratio = 400.0/27
		if intercept_ratio > nearness_ratio:
			return intercept_ratio, count1+count2
		else:
			return nearness_ratio, count2+count1

	def all_params( self, state, reciever_id):
		intercept_ratio = self.intercept_ratio( state, reciever_id)
		nearness_ratio = self.nearness_ratio( state, reciever_id)
		#botPos = Vector2D( state.homePos[reciever_id].x, state.homePos[reciever_id].y)
		#passerPos = Vector2D( state.homePos[self.bot_id].x, state.homePos[self.bot_id].y)
		#dis_bn_p_r = botPos.dist(passerPos)
		#dis_from_goal = botPos.dist(Vector2D(HALF_FIELD_MAXX, 0))
		return intercept_ratio, nearness_ratio
