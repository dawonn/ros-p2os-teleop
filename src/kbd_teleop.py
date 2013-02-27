#!/usr/bin/env python

import roslib; roslib.load_manifest('p2os_teleop')
import rospy
from geometry_msgs.msg import Twist

import threading
import time
import termios, sys, os
TERMIOS = termios

cmd = Twist()

class Teleop(object):

  def __init__(self):
    rospy.init_node('kbd_teleop')
    self.velocityCommandPublisher = rospy.Publisher("cmd_vel", Twist)

teleop = Teleop()

class Getch:
  def __init__(self):
    import termios, sys, os

  def __call__(self):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 1
    new[6][TERMIOS.VTIME] = 0
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
      c = os.read(fd, 1)
    finally:
      termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    return c
getch = Getch()

class StartTeleop(threading.Thread):
  global teleop, cmd
  def run(self):
    while True:
      teleop.velocityCommandPublisher.publish(cmd)
      cmd.linear.x = 0.0
      cmd.linear.y = 0.0
      cmd.angular.z = 0.0
      time.sleep(0.25)

# ROS GUI toolkit or GTK

class StartKeystroke(threading.Thread):

  global getch, teleop, cmd
  cmd.linear.x = cmd.linear.y = cmd.angular.z = 0

  def run(self):
    while not rospy.is_shutdown():
      key = getch()
      if key == 'w':
        cmd.linear.x =   0.25
      elif key == 's':
        cmd.linear.x =  -0.25
      elif key == 'a':
        cmd.angular.z =  0.4
      elif key == 'd':
        cmd.angular.z = -0.4       
      elif key == 'q':
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        teleop.velocityCommandPublisher.publish(cmd)
        sys.exit()

if __name__ == '__main__':
  sys.stdout.write("Use 'wasd' controls, 'q' to exit.  \n")
  StartKeystroke.daemon = True
  StartTeleop.daemon = True
  
  StartKeystroke().start()
  StartTeleop().start()
  try:
    rospy.spin()
 
  except (KeyboardInterrupt, SystemExit):
    cmd.linear.x  = 0.0
    cmd.linear.y  = 0.0
    cmd.angular.z = 0.0
    teleop.velocityCommandPublisher.publish(cmd)
    sys.exit()
  
  
  
  
  
