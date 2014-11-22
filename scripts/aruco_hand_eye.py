#!/usr/bin/env python

import rospy
from aruco_hand_eye import HandEyeConnector

def main():
    rospy.init_node('aruco_hand_eye')

    hec = HandEyeConnector()

    rospy.spin()


if __name__ == '__main__':
    main()
