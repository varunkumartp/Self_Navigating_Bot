#!/usr/bin/env python

# sends twist messages on cmd_vel topic

from __future__ import print_function
import rospy
import sys, select, termios, tty
from geometry_msgs.msg import Twist

#############################################################################
MAX_LIN_SPEED = 0.37
MIN_LIN_SPEED = 0.25

#############################################################################
MAX_ANG_SPEED = 3
MIN_ANG_SPEED = 1.5

#############################################################################
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
        i    
   j    k    l
        ,    

anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
""" 

#############################################################################
moveBindings = {
        'i':(1,0,0,0),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        ',':(-1,0,0,0),
        'o':(1,0,0,-1),
        'u':(1,0,0,1),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1)
    }

#############################################################################
speedBindings={
        'q':(0.005,0.05),
        'z':(-0.005,-0.05),
        'w':(0.005,0),
        'x':(-0.005,0),
        'e':(0,0.05),
        'c':(0,-0.05),
    }

#############################################################################
def checkLinVel(vel):
#############################################################################
    if vel < MIN_LIN_SPEED:
        vel = MIN_LIN_SPEED
    elif vel > MAX_LIN_SPEED:
        vel = MAX_LIN_SPEED
    else:
        vel = vel
    return vel

#############################################################################
def checkAngVel(vel):
#############################################################################
    if vel < MIN_ANG_SPEED:
        vel = MIN_ANG_SPEED
    elif vel > MAX_ANG_SPEED:
        vel = MAX_ANG_SPEED
    else:
        vel = vel
    return vel

#############################################################################
def getKey():
#############################################################################
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

#############################################################################
def vels(speed,turn):
#############################################################################
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

#############################################################################
#############################################################################
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('navigator_teleop')
    nodename = rospy.get_name()
    rospy.loginfo("-I- %s started" % nodename)
    
    # publisher    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 50)

    speed = rospy.get_param("~speed", 0.37)
    turn = rospy.get_param("~turn", 2.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed,turn))
        while True:
            key=getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = checkLinVel(speed + speedBindings[key][0])
                turn = checkAngVel(turn + speedBindings[key][1])

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
