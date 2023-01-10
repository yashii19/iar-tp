#!/usr/bin/env python

from radarGuidance import *
from wallFollower import *

import random #used for the random choice of a strategy
import sys
import numpy as np
import math
import time
#import collectons
#--------------------------------------
# Position of the goal:
goalx = 300
goaly = 450
# Initial position of the robot:
initx = 300
inity = 35
# strategy choice related stuff:
choice = -1
choice_tm1 = -1
tLastChoice = 0
rew = 0

i2name=['wallFollower','radarGuidance']

# Parameters of State building:
# threshold for wall consideration
th_neglectedWall = 35
# threshold to consider that we are too close to a wall
# and a punishment should be delivered
th_obstacleTooClose = 13
# angular limits used to define states
angleLMin = 0
angleLMax = 55

angleFMin=56
angleFMax=143

angleRMin=144
angleRMax=199

global timeOut
timeOut = 2
global lastTime
lastTime = 0

# Q-learning related stuff:

qdict = dict()
alpha=0.4
beta=4
gamma=0.95
wasChanged = False

#collections.defaultdict(lambda :np.zeros(2))
# definition of states at time t and t-1
S_t = ''
S_tm1 = ''


#--------------------------------------

def add_stats(fname): 
	# Add the first quantile, the median and the third quantile to the log file
	f = open(fname, "r")
	v = []
	x = f.readline()
	while (x != ''):
		v.append(float(x))
		x = f.readline()

	values = np.array(v)
	median = np.median(values)
	stats = np.quantile(values, [0.25, 0.5, 0.75]) 
	f = open(fname, "a")
	f.write(np.array2string(stats))
	print(f)
	f.close

def softmax(Q,x,beta):
    # Returns a soft-max probability distribution over action
    p = np.zeros((len(Q[x])))
    sump = 0
    for i in range(len(p)) :
        p[i] = np.exp((Q[x][i] * beta))
        sump += p[i]
    
    p = p/sump
    
    return p

def proba_discrete(p):
    # Draw a random number using probability table p
    res = np.random.random()
    proba_cumule=np.hstack((np.zeros(1),p.cumsum()))
    sample = -1
    for i in range(p.size):
        if (res>proba_cumule[i]) & (res<=proba_cumule[i+1]):
            sample = i
            break
    return sample
#--------------------------------------
# the function that selects which controller (radarGuidance or wallFollower) to use
# sets the global variable "choice" to 0 (wallFollower) or 1 (radarGuidance)
# * arbitrationMethod: how to select? 'random','randPersist','qlearning'
def strategyGating(arbitrationMethod,verbose=True):
  global choice
  global choice_tm1
  global tLastChoice
  global rew
  global lastTime
  global wasChanged
  global qdict
  global alpha
  global beta
  global gamma
  global S_t
  global S_tm1
  
  
  choice_tm1 = choice

  # The chosen gating strategy is to be coded here:
  #------------------------------------------------
  if arbitrationMethod=='random':
    choice = random.randrange(2)
  #------------------------------------------------
  elif arbitrationMethod=='randomPersist':
     #print('Persistent Random selection')
    
    now = time.time()
    #print("Start = ", start, " and now = ", now)
    if (start == -1 or now - start > 2.0) :
    	start = now = time.time()
    	choice = random.randrange(2)
    	tLastChoice = choice
    	print("switch strategies to : ", choice)
    else :
    	#print("same choice : ", tLastChoice)
    	choice = tLastChoice
      
      
  #------------------------------------------------
  elif arbitrationMethod=='qlearning':
    #print('Q-Learning selection')
    
    #Init dict
    if S_t not in qdict:
  
    	qdict[S_t] = [0] * len(i2name)
    if S_tm1 not in qdict:

    	qdict[S_tm1] = [0] * len(i2name)
      

    if S_tm1!=S_t or rew!=0 or wasChanged is True:
 
    	delta = rew + gamma  * np.max(qdict[S_t]) - qdict[S_tm1][choice_tm1]
    	qdict[S_tm1][choice_tm1] = qdict[S_tm1][choice_tm1] + alpha * delta
    
    t = time.time()
    
    if S_tm1!=S_t or t-tLastChoice>2 or rew!=0:

    	wasChanged = True
    	tLastChoice = t
    	#choice = np.random.choice(softmax(qdict, S_tm1, beta))
    	choice = proba_discrete(softmax(qdict, S_tm1, beta))
    	print(choice)
    else:
 
    	wasChanged = False
    
    rew = 0
    


  #------------------------------------------------
  else:
    print(arbitrationMethod+' unknown.')
    exit()

  if verbose:
    print("strategyGating: Active Module: "+i2name[choice])


#--------------------------------------
def buildStateFromSensors(laserRanges,radar,dist2goal):
  S   = ''
  # determine if obstacle on the left:
  wall='0'
  if min(laserRanges[angleLMin:angleLMax]) < th_neglectedWall:
    wall ='1'
  S += wall
  # determine if obstacle in front:
  wall='0'
  if min(laserRanges[angleFMin:angleFMax]) < th_neglectedWall:
    wall ='1'
    #print("Mur Devant")
  S += wall
  # determine if obstacle on the right:
  wall='0'
  if min(laserRanges[angleRMin:angleRMax]) < th_neglectedWall:
    wall ='1'
  S += wall

  S += str(radar)

  if dist2goal < 125:
    S+='0'
  elif dist2goal < 250:
    S+='1'
  else:
    S+='2'
  #print('buildStateFromSensors: State: '+S)

  return S

#--------------------------------------
def main():
  global S_t
  global S_tm1
  global rew

  settings = Settings('worlds/entonnoir.xml')

  env_map = settings.map()
  robot = settings.robot()

  d = Display(env_map, robot)

  method = 'qlearning'
  # experiment related stuff
  startT = time.time()
  trial = 0
  nbTrials = 40
  trialDuration = np.zeros((nbTrials))
  
  if method == 'qlearning':
      #Log Qlearning
      positions = []
      position = []
      
      pt = time.time()

  i = 0
  while trial<nbTrials:
    # update the display
    #-------------------------------------
    d.update()
    # get position data from the simulation
    #-------------------------------------
    pos = robot.get_pos()
    # print("##########\nStep "+str(i)+" robot pos: x = "+str(int(pos.x()))+" y = "+str(int(pos.y()))+" theta = "+str(int(pos.theta()/math.pi*180.)))
    
    if method == 'qlearning' and time.time() - pt > 1:
    	pt = time.time()
    	position.append(pos)


    # has the robot found the reward ?
    #------------------------------------
    dist2goal = math.sqrt((pos.x()-goalx)**2+(pos.y()-goaly)**2)
    # if so, teleport it to initial position, store trial duration, set reward to 1:
    if (dist2goal<20): # 30
      print('***** REWARD REACHED *****')
      pos.set_x(initx)
      pos.set_y(inity)
      robot.set_pos(pos) # format ?
      # and store information about the duration of the finishing trial:
      currT = time.time()
      trialDuration[trial] = currT - startT
      startT = currT
      print("Trial "+str(trial)+" duration:"+str(trialDuration[trial]))
      trial +=1
      rew = 1
      
      if method == 'qlearning':
      	positions.append(position)
      	position = []


    # get the sensor inputs:
    #------------------------------------
    lasers = robot.get_laser_scanners()[0].get_lasers()
    laserRanges = []
    for l in lasers:
      laserRanges.append(l.get_dist())

    radar = robot.get_radars()[0].get_activated_slice()

    bumperL = robot.get_left_bumper()
    bumperR = robot.get_right_bumper()


    # 2) has the robot bumped into a wall ?
    #------------------------------------
    if bumperR or bumperL or min(laserRanges[angleFMin:angleFMax]) < th_obstacleTooClose:
      rew = -1
      print("***** BING! ***** "+i2name[choice])

    # 3) build the state, that will be used by learning, from the sensory data
    #------------------------------------
    S_tm1 = S_t
    S_t = buildStateFromSensors(laserRanges,radar, dist2goal)

    #------------------------------------
    strategyGating(method,verbose=False)
    if choice==0:
      v = wallFollower(laserRanges,verbose=False)
    else:
      v = radarGuidance(laserRanges,bumperL,bumperR,radar,verbose=False)

    i+=1
    robot.move(v[0], v[1], env_map)
    time.sleep(0.01)

  # When the experiment is over:
  print("******************************************************")
  np.savetxt('log/'+str(startT)+'-TrialDurations-'+method+'.txt',trialDuration)
  name_file = 'log/'+str(startT)+'-TrialDurations-'+method+'.txt'
  add_stats(name_file)
  if method == 'qlearning' :
  	np.save('log/'+str(startT) +'-Qpos', positions)
  	np.save('log/'+str(startT) +'-Qvalues', dict(qdict))

#--------------------------------------

if __name__ == '__main__':
  random.seed()
  main()
