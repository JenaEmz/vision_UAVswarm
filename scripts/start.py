#!/usr/bin/env python3
from SwarmUav import quad
import rospy
from Vision.ObjDet import UavDect

if __name__ == '__main__':
    rospy.init_node("SwarmController")
    #Detector = UavDect()
    VisionSwarm = quad.Swarm(2)
    while(1):
        for Uav in VisionSwarm.quads:
            Uav.pub_control()
        rospy.sleep(1)
    rospy.spin()
