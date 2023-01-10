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
v_fwd = 4.
v_turn = 0.5
# scans used to check wall distances:
angleLMin = 0
angleLMax = 55

angleFMin = 56
angleFMax = 143

angleRMin = 144 #199-55
angleRMax = 199

th_obstacleTooClose = 13


#--------------------------------------
# radarGuidance: orients the robot towars the beacon detected by the radar sensor
# * laserRanges: used to avoid getting stuck in obstacles not detected by the bumpers
# * bumper_l: activation of the left bumper
# * bumper_r: activation of the right bumper
# * radar : the direction sensor indicating the direction of the goal
def radarGuidance(laserRanges, bumper_l, bumper_r,radar,verbose=True):
  wallTooCloseL = False
  wallTooCloseF = False
  wallTooCloseR = False
  v = [0,0]

  # determine if obstacle too close:
  for i in range(len(laserRanges)):
    if laserRanges[i] < th_obstacleTooClose:
      if i in range(angleLMin,angleLMax):
        wallTooCloseL = True
      if i in range(angleFMin,angleFMax):
        wallTooCloseF = True
      if i in range(angleRMin,angleRMax):
        wallTooCloseR = True

  # Choose policy based on obstacle and beacon direction detection:
  if wallTooCloseF:
    if verbose:
      print('WALL F')
    v[0] = -v_fwd
    v[1] = -v_fwd
  elif bumper_r or wallTooCloseR:
    if verbose:
      print('WALL R')
    v[0] =  v_fwd
    v[1] = -v_fwd
  elif bumper_l or wallTooCloseL:
    if verbose:
      print('WALL L')
    v[0] =  -v_fwd
    v[1] = v_fwd
  elif (7 == radar) :
    if verbose:
      print('FWD L')
    v[0] =  v_fwd
    v[1] = v_fwd*.8
  elif (0 == radar):
    if verbose:
      print('FWD R')
    v[0] =  v_fwd*.8
    v[1] = v_fwd
  # if it is on the left :
  elif (6 == radar) or (5 == radar):
    if verbose:
      print('LEFT')
    v[0] =  v_fwd
    v[1] = v_turn
  # if it is on the right :
  elif (1 == radar) or (2 == radar):
    if verbose:
      print('RIGHT')
    v[0] =  v_turn
    v[1] = v_fwd
  # if it is  behind :
  elif (3 == radar) :
    if verbose:
      print('BEHIND R')
    v[0] =  -v_fwd
    v[1] = v_fwd
  elif (4 == radar) :
    if verbose:
      print('BEHIND L')
    v[0] =  v_fwd
    v[1] = -v_fwd

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

    # get the sensor inputs and prepare them for the radarGuidance() function:
    lasers = robot.get_laser_scanners()[0].get_lasers()
    laserRanges = []
    for l in lasers:
      laserRanges.append(l.get_dist())

    radar = robot.get_radars()[0].get_activated_slice()

    bumperL = robot.get_left_bumper()
    bumperR = robot.get_right_bumper()

    #------------------------------------
    v = radarGuidance(laserRanges,bumperL,bumperR,radar)

    robot.move(v[0], v[1], env_map)
    time.sleep(0.01)


#--------------------------------------

if __name__ == '__main__':
  main()
