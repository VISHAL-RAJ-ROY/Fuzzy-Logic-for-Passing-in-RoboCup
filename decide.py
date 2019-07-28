import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from utils import *

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
HALF_FIELD_MAXX_REAL = 4431


RATIO = MAX_BALL_SPEED*1.0/MAX_BOT_SPEED

#near_max_ratio = (HALF_FIELD_MAXX)*2.0/BOT_RADIUS
#near_max_ratio = 400.0/27
near_max_ratio = RATIO*(10.0/3)
near_min_ratio = BOT_RADIUS*1.0/(HALF_FIELD_MAXX-BOT_RADIUS)

print "                      ",near_max_ratio
#DISTANCE BETWEEN PASSER AND RECIEVER
# dis_bn_p_r = ctrl.Antecedent(np.arange(0, (HALF_FIELD_MAXX)*2, HALF_FIELD_MAXX*1.0/9 ), 'dis_bn_p_r') 

#RATION OF PERPENDICULAR DISTANCE AND IN-LINE DISTANCE
# intercept_ratio = ctrl.Antecedent(np.arange(0, near_max_ratio, near_max_ratio/20), 'intercept_ratio') 

#RATIO OF THE NEAREST OPPONENT TO TARGET AND DISTANCE BETWEEN THE TWO
# nearness_ratio = ctrl.Antecedent(np.arange(0, near_max_ratio, near_max_ratio/20), 'nearness_ratio')	

#DISTANCE FROM THE GOAL
dp = ctrl.Antecedent(np.arange(0, HALF_FIELD_MAXX_REAL*2, HALF_FIELD_MAXX_REAL*1.0/9), 'dis_bn_p_r') 

nr = ctrl.Antecedent(np.arange(0, near_max_ratio, near_max_ratio/20), 'net_ratio')

#Danger number
dn = ctrl.Antecedent(np.arange(0,6,1), 'danger_no')

dg = ctrl.Antecedent(np.arange(0, HALF_FIELD_MAXX_REAL*2, HALF_FIELD_MAXX_REAL*1.0/9), 'dis_from_goal')

#PASS SCORE
ps = ctrl.Consequent( np.arange(0,60,1), 'pass_score')



# DISTANCE BETWEEN THE PLAYERS
dp['N'] = fuzz.trapmf(dp.universe, [0, 0, HALF_FIELD_MAXX_REAL*2.0*2.0/18, HALF_FIELD_MAXX_REAL*2.0*5.0/18])
dp['M'] = fuzz.trapmf(dp.universe, [HALF_FIELD_MAXX_REAL*2.0*2.0/18.0, HALF_FIELD_MAXX_REAL*2.0*5.0/18.0, HALF_FIELD_MAXX_REAL*10.0/18.0, HALF_FIELD_MAXX_REAL*14.0/18])
dp['F'] = fuzz.trapmf(dp.universe, [HALF_FIELD_MAXX_REAL*10.0/18.0, HALF_FIELD_MAXX_REAL*14.0/18, HALF_FIELD_MAXX_REAL*2.0, HALF_FIELD_MAXX_REAL*2.0])


#NET RATIO
nr['L'] = fuzz.trapmf(nr.universe, [0, 0, near_max_ratio*2.0/10, near_max_ratio*3.5/10])
nr['M'] = fuzz.trimf(nr.universe, [near_max_ratio*2.0/10, near_max_ratio*3.5/10, near_max_ratio*5.0/10])
nr['H'] = fuzz.trapmf(nr.universe, [near_max_ratio*3.5/10, near_max_ratio*5.0/10, near_max_ratio, near_max_ratio])

#DANGER NUMBER
dn['L'] = fuzz.trapmf(dn.universe, [0,0,1,2])
dn['M'] = fuzz.trimf(dn.universe, [1,2,3])
dn['H'] = fuzz.trapmf(dn.universe, [2,3,6,6])

#DISTANCE FROM THE GOAL
dg['N'] = fuzz.trapmf(dg.universe, [0, 0, HALF_FIELD_MAXX_REAL*2.0/9, HALF_FIELD_MAXX_REAL*2.0*5.0/18])
dg['M'] = fuzz.trapmf(dg.universe, [HALF_FIELD_MAXX_REAL*2.0*2.0/18, HALF_FIELD_MAXX_REAL*2.0*5.0/18, HALF_FIELD_MAXX_REAL*2.0*6.0/18, HALF_FIELD_MAXX_REAL*2.0*9.0/18])
dg['F'] = fuzz.trapmf(dg.universe, [HALF_FIELD_MAXX_REAL*2.0*6.0/18, HALF_FIELD_MAXX_REAL*2.0*8.0/18, HALF_FIELD_MAXX_REAL*2, HALF_FIELD_MAXX_REAL*2])

#pass_score['g0'] = fuzz.trimf(pass_score.universe, [0,0,1])

#SCORE ASSIGNMENT
ps['g52'] = fuzz.trimf(ps.universe, [59,60,60])
ps['g51'] = fuzz.trimf(ps.universe, [56,58,60])
ps['g50'] = fuzz.trimf(ps.universe, [55,57,59])
ps['g49'] = fuzz.trimf(ps.universe, [54,56,58])
ps['g48'] = fuzz.trimf(ps.universe, [53,55,57])
ps['g47'] = fuzz.trimf(ps.universe, [52,54,56])
ps['g46'] = fuzz.trimf(ps.universe, [51,53,55])
ps['g45'] = fuzz.trimf(ps.universe, [49,51,53])
ps['g44'] = fuzz.trimf(ps.universe, [48,50,52])
ps['g43'] = fuzz.trimf(ps.universe, [47,49,51])
ps['g42'] = fuzz.trimf(ps.universe, [46,48,50])
ps['g41'] = fuzz.trimf(ps.universe, [45,47,49])
ps['g40'] = fuzz.trimf(ps.universe, [44,46,48])
ps['g39'] = fuzz.trimf(ps.universe, [43,45,47])
ps['g38'] = fuzz.trimf(ps.universe, [42,44,46])
ps['g37'] = fuzz.trimf(ps.universe, [40,42,44])
ps['g36'] = fuzz.trimf(ps.universe, [39,41,43])
ps['g35'] = fuzz.trimf(ps.universe, [38,40,42])
ps['g34'] = fuzz.trimf(ps.universe, [37,39,41])
ps['g33'] = fuzz.trimf(ps.universe, [36,38,40])
ps['g32'] = fuzz.trimf(ps.universe, [35,37,39])
ps['g31'] = fuzz.trimf(ps.universe, [34,36,38])
ps['g30'] = fuzz.trimf(ps.universe, [33,35,37])
ps['g29'] = fuzz.trimf(ps.universe, [32,34,36])
ps['g28'] = fuzz.trimf(ps.universe, [31,33,35])
ps['g27'] = fuzz.trimf(ps.universe, [29,31,33])
ps['g26'] = fuzz.trimf(ps.universe, [28,30,32])
ps['g25'] = fuzz.trimf(ps.universe, [27,29,31])
ps['g24'] = fuzz.trimf(ps.universe, [26,28,30])
ps['g23'] = fuzz.trimf(ps.universe, [25,27,29])
ps['g22'] = fuzz.trimf(ps.universe, [24,26,28])
ps['g21'] = fuzz.trimf(ps.universe, [23,25,27])
ps['g20'] = fuzz.trimf(ps.universe, [22,24,26])
ps['g19'] = fuzz.trimf(ps.universe, [21,23,25])
ps['g18'] = fuzz.trimf(ps.universe, [20,22,24])
ps['g17'] = fuzz.trimf(ps.universe, [19,21,23])
ps['g16'] = fuzz.trimf(ps.universe, [18,20,22])
ps['g15'] = fuzz.trimf(ps.universe, [17,19,21])
ps['g14'] = fuzz.trimf(ps.universe, [15,17,19])
ps['g13'] = fuzz.trimf(ps.universe, [14,16,18])
ps['g12'] = fuzz.trimf(ps.universe, [13,15,17])
ps['g11'] = fuzz.trimf(ps.universe, [12,14,16])
ps['g10'] = fuzz.trimf(ps.universe, [8,10,12])
ps['g9'] = fuzz.trimf(ps.universe, [7,9,11])
ps['g8'] = fuzz.trimf(ps.universe, [6,8,10])
ps['g7'] = fuzz.trimf(ps.universe, [5,7,9])
ps['g6'] = fuzz.trimf(ps.universe, [4,6,8])
ps['g5'] = fuzz.trimf(ps.universe, [3,5,7])
ps['g4'] = fuzz.trimf(ps.universe, [2,4,6])
ps['g3'] = fuzz.trimf(ps.universe, [1,3,5])
ps['g2'] = fuzz.trimf(ps.universe, [0,2,4])
ps['g1'] = fuzz.trimf(ps.universe, [0,0,1])


#rule3 = ctrl.Rule(dis_from_goal['N'] & dis_bn_p_r['F'] & intercept_ratio['M'] & nearness_ratio['L'] , pass_score['g15'])

#RULES
r1 = ctrl.Rule(dg['N'] & nr['L'] & dn['L'] & dp['N'], ps['g52'])
r2 = ctrl.Rule(dg['N'] & nr['L'] & dn['M'] & dp['N'], ps['g51'])
r3 = ctrl.Rule(dg['N'] & nr['L'] & dn['L'] & dp['M'], ps['g50'])
r4 = ctrl.Rule(dg['N'] & nr['L'] & dn['H'] & dp['N'], ps['g49'])
r5 = ctrl.Rule(dg['N'] & nr['L'] & dn['M'] & dp['M'], ps['g49'])
r6 = ctrl.Rule(dg['N'] & nr['L'] & dn['L'] & dp['F'], ps['g48'])
r7 = ctrl.Rule(dg['N'] & nr['L'] & dn['M'] & dp['F'], ps['g47'])
r8 = ctrl.Rule(dg['N'] & nr['L'] & dn['H'] & dp['M'], ps['g47'])
r9 = ctrl.Rule(dg['N'] & nr['L'] & dn['H'] & dp['F'], ps['g46'])
r10 = ctrl.Rule(dg['M'] & nr['L'] & dn['L'] & dp['N'], ps['g45'])
r11 = ctrl.Rule(dg['N'] & nr['M'] & dn['L'] & dp['N'], ps['g45'])
r12 = ctrl.Rule(dg['N'] & nr['M'] & dn['L'] & dp['N'], ps['g44'])
r13 = ctrl.Rule(dg['M'] & nr['L'] & dn['M'] & dp['N'], ps['g43'])
r14 = ctrl.Rule(dg['N'] & nr['M'] & dn['M'] & dp['N'], ps['g43'])
r15 = ctrl.Rule(dg['M'] & nr['L'] & dn['L'] & dp['M'], ps['g43'])
r16 = ctrl.Rule(dg['N'] & nr['M'] & dn['L'] & dp['F'], ps['g43'])
r17 = ctrl.Rule(dg['N'] & nr['M'] & dn['M'] & dp['M'], ps['g42'])
r18 = ctrl.Rule(dg['M'] & nr['L'] & dn['H'] & dp['N'], ps['g41'])
r19 = ctrl.Rule(dg['N'] & nr['M'] & dn['H'] & dp['N'], ps['g41'])
r20 = ctrl.Rule(dg['M'] & nr['L'] & dn['L'] & dp['F'], ps['g41'])
r21 = ctrl.Rule(dg['M'] & nr['L'] & dn['M'] & dp['M'], ps['g41'])
r22 = ctrl.Rule(dg['N'] & nr['M'] & dn['M'] & dp['F'], ps['g41'])
r23 = ctrl.Rule(dg['M'] & nr['L'] & dn['H'] & dp['M'], ps['g40'])
r24 = ctrl.Rule(dg['N'] & nr['M'] & dn['H'] & dp['M'], ps['g40'])
r25 = ctrl.Rule(dg['M'] & nr['M'] & dn['L'] & dp['N'], ps['g39'])
r26 = ctrl.Rule(dg['M'] & nr['L'] & dn['M'] & dp['F'], ps['g39'])
r27 = ctrl.Rule(dg['F'] & nr['L'] & dn['L'] & dp['N'], ps['g38'])
r28 = ctrl.Rule(dg['N'] & nr['M'] & dn['H'] & dp['F'], ps['g37'])
r29 = ctrl.Rule(dg['M'] & nr['L'] & dn['H'] & dp['F'], ps['g36'])
r30 = ctrl.Rule(dg['F'] & nr['L'] & dn['M'] & dp['N'], ps['g35'])
r31 = ctrl.Rule(dg['F'] & nr['L'] & dn['M'] & dp['M'], ps['g34'])
r32 = ctrl.Rule(dg['M'] & nr['M'] & dn['L'] & dp['M'], ps['g33'])
r33 = ctrl.Rule(dg['F'] & nr['L'] & dn['M'] & dp['F'], ps['g33'])
r34 = ctrl.Rule(dg['M'] & nr['M'] & dn['M'] & dp['N'], ps['g32'])
r35 = ctrl.Rule(dg['F'] & nr['L'] & dn['L'] & dp['M'], ps['g32'])
r36 = ctrl.Rule(dg['F'] & nr['L'] & dn['H'] & dp['N'], ps['g31'])
r37 = ctrl.Rule(dg['F'] & nr['L'] & dn['L'] & dp['F'], ps['g30'])
r38 = ctrl.Rule(dg['F'] & nr['L'] & dn['H'] & dp['M'], ps['g29'])
r39 = ctrl.Rule(dg['F'] & nr['L'] & dn['H'] & dp['F'], ps['g28'])
r40 = ctrl.Rule(dg['M'] & nr['M'] & dn['L'] & dp['F'], ps['g27'])
r41 = ctrl.Rule(dg['M'] & nr['M'] & dn['M'] & dp['M'], ps['g26'])
r42 = ctrl.Rule(dg['M'] & nr['M'] & dn['H'] & dp['N'], ps['g25'])
r43 = ctrl.Rule(dg['M'] & nr['M'] & dn['M'] & dp['F'], ps['g25'])
r44 = ctrl.Rule(dg['F'] & nr['M'] & dn['L'] & dp['N'], ps['g24'])
r45 = ctrl.Rule(dg['N'] & nr['H'] & dn['L'] & dp['N'], ps['g24'])
r46 = ctrl.Rule(dg['F'] & nr['M'] & dn['M'] & dp['N'], ps['g23'])
r47 = ctrl.Rule(dg['F'] & nr['M'] & dn['L'] & dp['M'], ps['g23'])
r48 = ctrl.Rule(dg['M'] & nr['M'] & dn['H'] & dp['M'], ps['g22'])
r49 = ctrl.Rule(dg['F'] & nr['M'] & dn['L'] & dp['F'], ps['g21'])
r50 = ctrl.Rule(dg['M'] & nr['M'] & dn['H'] & dp['F'], ps['g20'])
r51 = ctrl.Rule(dg['F'] & nr['M'] & dn['H'] & dp['N'], ps['g19'])
r52 = ctrl.Rule(dg['F'] & nr['M'] & dn['M'] & dp['M'], ps['g18'])
r53 = ctrl.Rule(dg['F'] & nr['M'] & dn['M'] & dp['F'], ps['g17'])
r54 = ctrl.Rule(dg['F'] & nr['M'] & dn['H'] & dp['M'], ps['g16'])
r55 = ctrl.Rule(dg['F'] & nr['M'] & dn['H'] & dp['F'], ps['g15'])
r56 = ctrl.Rule(dg['N'] & nr['H'] & dn['L'] & dp['M'], ps['g14'])
r57 = ctrl.Rule(dg['N'] & nr['H'] & dn['M'] & dp['N'], ps['g13'])
r58 = ctrl.Rule(dg['N'] & nr['H'] & dn['L'] & dp['F'], ps['g13'])
r59 = ctrl.Rule(dg['N'] & nr['H'] & dn['H'] & dp['N'], ps['g12'])
r60 = ctrl.Rule(dg['N'] & nr['H'] & dn['M'] & dp['M'], ps['g12'])
r61 = ctrl.Rule(dg['N'] & nr['H'] & dn['M'] & dp['M'], ps['g11'])
r62 = ctrl.Rule(dg['N'] & nr['H'] & dn['H'] & dp['N'], ps['g10'])
r63 = ctrl.Rule(dg['N'] & nr['H'] & dn['H'] & dp['F'], ps['g10'])
r64 = ctrl.Rule(dg['M'] & nr['H'] & dn['M'] & dp['N'], ps['g9'])
r65 = ctrl.Rule(dg['M'] & nr['H'] & dn['L'] & dp['M'], ps['g9'])
r66 = ctrl.Rule(dg['M'] & nr['H'] & dn['H'] & dp['N'], ps['g8'])
r67 = ctrl.Rule(dg['M'] & nr['H'] & dn['L'] & dp['F'], ps['g8'])
r68 = ctrl.Rule(dg['M'] & nr['H'] & dn['M'] & dp['M'], ps['g8'])
r69 = ctrl.Rule(dg['F'] & nr['H'] & dn['L'] & dp['N'], ps['g7'])
r70 = ctrl.Rule(dg['M'] & nr['H'] & dn['H'] & dp['M'], ps['g7'])
r71 = ctrl.Rule(dg['M'] & nr['H'] & dn['M'] & dp['F'], ps['g7'])
r72 = ctrl.Rule(dg['F'] & nr['H'] & dn['L'] & dp['M'], ps['g6'])
r73 = ctrl.Rule(dg['M'] & nr['H'] & dn['H'] & dp['F'], ps['g6'])
r74 = ctrl.Rule(dg['F'] & nr['H'] & dn['L'] & dp['F'], ps['g5'])
r75 = ctrl.Rule(dg['F'] & nr['H'] & dn['M'] & dp['N'], ps['g4'])
r76 = ctrl.Rule(dg['F'] & nr['H'] & dn['H'] & dp['N'], ps['g3'])
r77 = ctrl.Rule(dg['F'] & nr['H'] & dn['M'] & dp['M'], ps['g3'])
r78 = ctrl.Rule(dg['F'] & nr['H'] & dn['H'] & dp['M'], ps['g2'])
r79 = ctrl.Rule(dg['F'] & nr['H'] & dn['M'] & dp['F'], ps['g2'])
r80 = ctrl.Rule(dg['F'] & nr['H'] & dn['H'] & dp['F'], ps['g1'])

# nr['L'].view()
# dn['L'].view()
# dg['N'].view()
# dp['N'].view()
# ps.view()
# r5.view()


# FUZZY SIMULATION CONTROLLER
rule_list = [r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15,r16,r17,r18,r19,r20,r21,r22,r23,r24,r25,r26,r27,r28,r29,r30,r31,r32,r33,r34,r35,r36,r37,r38,r39,r40,r41,r42,r43,r44,r45,r46,r47,r48,r49,r50,r51,r52,r53,r54,r55,r56,r57,r58,r59,r60,r61,r62,r63,r64,r65,r66,r67,r68,r69,r70,r71,r72,r73,r74,r75,r76,r77,r78,r79,r80]
rule_list_new = [r1,r3,r5,r7,r9,r11,r13,r15,r17,r19,r21,r23,r25,r27,r29,r31,r33,r35,r37,r39,r41,r43,r45,r47,r49,r51,r53,r55,r57,r59,r61,r63,r65,r67,r69,r71,r73,r75,r77,r79, r80]
fuzz_ctrl = ctrl.ControlSystem(rule_list)
fuzz_passing = ctrl.ControlSystemSimulation(fuzz_ctrl)

print "\n\n\n\n\n\n"
print "i am not here"
print "\n\n\n\n\n\n"

# fuzz_passing.input['dis_from_goal'] = 10.2
# fuzz_passing.input['dis_bn_p_r'] = 10.3
# fuzz_passing.input['net_ratio'] = 1.1
# fuzz_passing.input['danger_no'] = 2
# fuzz_passing.compute()

# fuzz_passing.input['dis_from_goal'] = 20.4
# fuzz_passing.input['dis_bn_p_r'] = 80.2
# fuzz_passing.input['net_ratio'] = 3.1
# fuzz_passing.input['danger_no'] = 4
# fuzz_passing.compute()

# fuzz_passing.input['dis_from_goal'] = 10.5
# fuzz_passing.input['dis_bn_p_r'] = 13.5
# fuzz_passing.input['net_ratio'] = 1.6
# fuzz_passing.input['danger_no'] = 3
# fuzz_passing.compute()

# fuzz_passing.input['dis_from_goal'] = 14.2
# fuzz_passing.input['dis_bn_p_r'] = 10.8
# fuzz_passing.input['net_ratio'] = 3.2
# fuzz_passing.input['danger_no'] = 3
# fuzz_passing.compute()

# fuzz_passing.input['dis_from_goal'] = 15.2
# fuzz_passing.input['dis_bn_p_r'] = 13.2
# fuzz_passing.input['net_ratio'] = 1.3
# fuzz_passing.input['danger_no'] = 0
# fuzz_passing.compute()
# UTILITY FUNCTION
T = []
def get_all(state, bot_id):
	global fuzz_passing, HALF_FIELD_MAXX_REAL
	team = [x for x in xrange(6)]
	our_player = pass_analysis(bot_id, state)
	param = ()
	max_score = -1
	index = -1
	botPos = Vector2D(state.homePos[bot_id].x, state.homePos[bot_id].y)
	oppgoalPos = Vector2D(HALF_FIELD_MAXX_REAL, 0)
	for player in team:
		if player == bot_id:
			continue
		else: 
			recPos = Vector2D(state.homePos[player].x, state.homePos[player].y)
			#param = our_player.ratios( state, player)
			sep = botPos.dist(recPos)
			sep_from_goal = recPos.dist(oppgoalPos)
			if sep_from_goal < HALF_FIELD_MAXX_REAL*2.0:
				fuzz_passing.input['dis_from_goal'] = sep_from_goal
			else:
				fuzz_passing.input['dis_from_goal'] = HALF_FIELD_MAXX_REAL*2
			if sep < HALF_FIELD_MAXX_REAL*2.0:
				fuzz_passing.input['dis_bn_p_r'] = sep
			else:
				fuzz_passing.input['dis_bn_p_r'] = HALF_FIELD_MAXX_REAL*2.0
			t = time.time()
			param = our_player.evr_params(state, player, botPos, recPos, sep)
			print "$"*20, "  ", time.time()-t , "  ", "$"*20
			fuzz_passing.input['danger_no'] = param[1]
			fuzz_passing.input['net_ratio'] = param[0]
			fuzz_passing.compute()
			print "$"*20, "  ", time.time()-t , "  ", "$"*20
			if len(T) < 5:
				T.append([time.time() - t, player])
			score = fuzz_passing.output['pass_score']
			#print player, state.homePos[player].x, state.homePos[player].y
			print"                 ---------                   "
			print "dis_from_goal: ",recPos.dist(oppgoalPos),"\ndis_bn_p_r: ",botPos.dist(recPos),"\nintercept_ratio: ",param[0],"\nnearness_ratio: ",param[1]
			print "SCORE--> ", score
			print recPos.x, "      ", recPos.y
			if score > max_score:
				max_score = score
				index = player
	print "#"*50
	print T
	print "BOT_TO_PASS---->"
	return index, max_score
