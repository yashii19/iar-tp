#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" Benoît Girard
    ISIR CNRS/UPMC
    01/10/2019
"""

# basic pyfastsim requirements:
from pyfastsim import *
import time

# imports specific to wallFollower
from math import *

# behavioral parameters:
#--------------------------------------
lMaxRange = 3000
robotRadius = 10 #15
th_obstacleFront = 30 #35
th_wallTooClose = 20 #25
th_wallTooFar = 30 #35
th_neglectedWall = 40 #50
v_fwd = 2.
v_turn = .6

# computation of the scans used to check front obstacle:
angleFrontMin = 99 - int(atan2(robotRadius,th_obstacleFront)/pi*180)
angleFrontMax = 100 + int(atan2(robotRadius,th_obstacleFront)/pi*180)

# scans used to check wall distances:
angleLMin = 0
angleLMax = 55

angleRMin = 199-55
angleRMax = 199
lastWallOnLeft = True

#--------------------------------------
# wallFollower: if a wall is seen on the right or on the left, follows this
# wall by maintaining a distance between th_wallTooClose and th_wallTooFar
# if no wall seen, then the robot turns (on the right if lastWallOnLeft,
# and on the left otherwise), in order to find a wall.
# * laserRanges: the distance measurments from the laser scanner
def wallFollower(laserRanges,verbose=True):
  global lastWallOnLeft
  obstacleFront = False
  wallTooCloseL = False
  wallTooFarL = False
  wallOKL = False
  wallOKR = False
  wallTooCloseR = False
  wallTooFarR = False

  distFrontMin = lMaxRange

  v=[0.,0.]

  #print(len(laserRanges))
  #print(laserRanges,'\n\n')

  # determine if obstacle in front:
  for l in laserRanges[angleFrontMin:angleFrontMax]:
    if l < distFrontMin:
      distFrontMin = l
    if l < th_obstacleFront:
      obstacleFront = True

  # determine if walls are within the "too close" and the "too far" L & R bands:
  distWallLMin = lMaxRange
  distWallRMin = lMaxRange

  for i in range(angleLMin,angleLMax):
    if laserRanges[i] < distWallLMin:
      distWallLMin = laserRanges[i]
    if laserRanges[i]*cos((10-i)/180.*pi) < th_wallTooClose:
      #print("Too close L("+str(i)+"):"+str(laserRanges[i])+" "+str(cos((10-i)/180.*pi))+" "+str(laserRanges[i]*cos((10-i)/180.*pi)))
      wallTooCloseL = True
    elif laserRanges[i]*cos((10-i)/180.*pi) < th_wallTooFar:
      wallOKL = True
    elif laserRanges[i]*cos((10-i)/180.*pi) < th_neglectedWall:
      wallTooFarL = True

  for i in range(angleRMin,angleRMax):
    if laserRanges[i] < distWallRMin:
      distWallRMin = laserRanges[i]
    if laserRanges[i]*cos((189-i)/180.*pi) < th_wallTooClose:
      #print("Too close L("+str(i)+"):"+str(laserRanges[i])+" "+str(cos((10-i)/180.*pi))+" "+str(laserRanges[i]*cos((10-i)/180.*pi)))
      wallTooCloseR = True
    elif laserRanges[i]*cos((189-i)/180.*pi) < th_wallTooFar:
      wallOKR = True
    elif laserRanges[i]*cos((189-i)/180.*pi) < th_neglectedWall:
      wallTooFarR = True

  # print string perception for debug
  if verbose:
    if wallTooCloseL:
      print('***')
    elif wallOKL:
      print('---')
    else:
      print('   ')

    if obstacleFront:
      print('O |')
    else :
      print('O  ')

    if wallTooCloseR:
      print('***')
    elif wallOKR:
      print('---')
    else:
      print('   ')

  # Choose policy based on front obstacle and lateral walls detection:
  if obstacleFront:
    #print("MUR DEVANT")
    if lastWallOnLeft:
      v[0]= -v_turn * 1.1
      v[1]=  v_turn
    else:
      v[0]=  v_turn
      v[1]= -v_turn * 1.1
    if verbose:
      print("Wall Follower: OBSTACLE - Speed L:"+str(v[0])+" R:"+str(v[1]))
  elif wallTooCloseL and (not(wallTooCloseR) or distWallLMin<distWallRMin):
    lastWallOnLeft = True
    v[0]= v_turn *0.8
    v[1]= v_fwd *0.8
    #v[0]=  v_fwd
    #v[1]= v_turn
    if verbose:
      print("Wall Follower: L TOO CLOSE - Speed L:"+str(v[0])+" R:"+str(v[1]))
  elif wallTooCloseR and (not(wallTooCloseL) or distWallLMin>distWallRMin):
    lastWallOnLeft = False
    v[0]= v_fwd *0.8
    v[1]= v_turn *0.8
    #v[0]=  v_turn
    #v[1]= v_fwd
    if verbose:
      print("Wall Follower: R TOO CLOSE - Speed L:"+str(v[0])+" R:"+str(v[1]))
  elif wallOKL :
    lastWallOnLeft = True
    v[0]=  v_fwd
    v[1]= v_fwd
    if verbose:
      print("Wall Follower: L OK - Speed L:"+str(v[0])+" R:"+str(v[1]))
  elif wallOKR :
    lastWallOnLeft = False
    v[0]=  v_fwd
    v[1]= v_fwd
    if verbose:
      print("Wall Follower: R OK - Speed L:"+str(v[0])+" R:"+str(v[1]))
  elif wallTooFarL and (not(wallTooFarR) or distWallLMin<distWallRMin):
    lastWallOnLeft = True
    v[0]= v_fwd
    v[1]= v_turn
    #v[0]=  v_turn
    #v[1]= v_fwd
    if verbose:
      print("Wall Follower: L TOO FAR - Speed L:"+str(v[0])+" R:"+str(v[1]))
  elif wallTooFarR and (not(wallTooFarL) or distWallLMin>distWallRMin):
    lastWallOnLeft = False
    v[0]= v_turn
    v[1]= v_fwd
    #v[0]=  v_fwd
    #v[1]= v_turn
    if verbose:
      print("Wall Follower: R TOO FAR - Speed L:"+str(v[0])+" R:"+str(v[1]))
  elif lastWallOnLeft:
    v[0]= v_turn
    v[1]= - v_turn
    if verbose:
      print("Wall Follower: LOST WALL, L - Speed L:"+str(v[0])+" R:"+str(v[1]))
  else:
    v[0]= - v_turn
    v[1]= v_turn
    if verbose:
      print("Wall Follower: LOST WALL, R - Speed L:"+str(v[0])+" R:"+str(v[1]))

  return v

#--------------------------------------
def main():
  settings = Settings('worlds/entonnoir.xml')

  env_map = settings.map()
  robot = settings.robot()

  d = Display(env_map, robot)

  for i in range(1000):
    d.update()
    # get sensory data from the simulation
    #------------------------------------
    pos = robot.get_pos()
    print("Step %d robot pos: x = %f    y = %f    theta = %f" % (i, pos.x(), pos.y(), pos.theta()))
    lasers = robot.get_laser_scanners()[0].get_lasers()
    laserRanges = []
    for l in lasers:
      laserRanges.append(l.get_dist())

    #------------------------------------
    v = wallFollower(laserRanges)

    robot.move(v[0], v[1], env_map)
    time.sleep(0.01)


#--------------------------------------

if __name__ == '__main__':
  main()
