#!/usr/bin/env python

import sys
import yaml
import rospy
import numpy as np
import time
import tf
import tf_conversions
import std_msgs
from std_srvs.srv import Empty
from crazyflie_driver.srv import *
from crazyflie_driver.msg import TrajectoryPolynomialPiece, FullState, Position, Hover


def arrayToGeometryPoint(a):
    return geometry_msgs.msg.Point(a[0], a[1], a[2])


class TimeHelper:
    def __init__(self):
        pass
        #rospy.wait_for_service("/next_phase")
        #self.nextPhase = rospy.ServiceProxy("/next_phase", Empty)

    def time(self):
        return time.time()

    def sleep(self, duration):
        time.sleep(duration)

    def nextPhase(self):
        pass
        #self.nextPhase()


class Crazyflie:
    def __init__(self, id, initialPosition, timeHelper):
        self.id = id
        self.prefix = "/cf" + str(id)
        self.initialPosition = np.array(initialPosition)
        self.timeHelper = timeHelper # not used here, just compatible with crazyflieSim
        self.tf = tf.TransformListener()
        self.my_frame = "vicon" + self.prefix + self.prefix

        rospy.wait_for_service(self.prefix + "/set_group_mask")
        self.setGroupMaskService = rospy.ServiceProxy(self.prefix + "/set_group_mask", SetGroupMask)
        rospy.wait_for_service(self.prefix + "/takeoff")
        self.takeoffService = rospy.ServiceProxy(self.prefix + "/takeoff", Takeoff)
        rospy.wait_for_service(self.prefix + "/land")
        self.landService = rospy.ServiceProxy(self.prefix + "/land", Land)
        rospy.wait_for_service(self.prefix + "/stop")
        self.stopService = rospy.ServiceProxy(self.prefix + "/stop", Stop)
        rospy.wait_for_service(self.prefix + "/go_to")
        self.goToService = rospy.ServiceProxy(self.prefix + "/go_to", GoTo)
        rospy.wait_for_service(self.prefix + "/upload_trajectory")
        self.uploadTrajectoryService = rospy.ServiceProxy(self.prefix + "/upload_trajectory", UploadTrajectory)
        rospy.wait_for_service(self.prefix + "/start_trajectory")
        self.startTrajectoryService = rospy.ServiceProxy(self.prefix + "/start_trajectory", StartTrajectory)
        rospy.wait_for_service(self.prefix + "/update_params")
        self.updateParamsService = rospy.ServiceProxy(self.prefix + "/update_params", UpdateParams)

        self.cmdFullStatePublisher = rospy.Publisher(self.prefix + "/cmd_full_state", FullState, queue_size=1)
        self.cmdFullStateMsg = FullState()
        self.cmdFullStateMsg.header.seq = 0
        self.cmdFullStateMsg.header.frame_id = "/world"

        self.cmdPositionPublisher = rospy.Publisher(self.prefix + "/cmd_position", Position, queue_size=1)
        self.cmdPositionMsg = Position()
        self.cmdPositionMsg.header.seq = 0
        self.cmdPositionMsg.header.frame_id = "/world"

        self.cmdHoverPublisher = rospy.Publisher(self.prefix + "/cmd_hover", Hover, queue_size=1)
        self.cmdHoverMsg = Hover()
        self.cmdHoverMsg.header.seq = 0
        self.cmdHoverMsg.header.frame_id = "/world"

        self.cmdStopPublisher = rospy.Publisher(self.prefix + "/cmd_stop", std_msgs.msg.Empty, queue_size=1)

    def setGroupMask(self, groupMask):
        self.setGroupMaskService(groupMask)

    def takeoff(self, targetHeight, duration, groupMask = 0):
        self.takeoffService(groupMask, targetHeight, rospy.Duration.from_sec(duration))

    def land(self, targetHeight, duration, groupMask = 0):
        self.landService(groupMask, targetHeight, rospy.Duration.from_sec(duration))

    def stop(self, groupMask = 0):
        self.stopService(groupMask)

    def goTo(self, goal, yaw, duration, relative = False, groupMask = 0):
        gp = arrayToGeometryPoint(goal)
        self.goToService(groupMask, relative, gp, yaw, rospy.Duration.from_sec(duration))

    def uploadTrajectory(self, trajectoryId, pieceOffset, trajectory):
        pieces = []
        for poly in trajectory.polynomials:
            piece = TrajectoryPolynomialPiece()
            piece.duration = rospy.Duration.from_sec(poly.duration)
            piece.poly_x   = poly.px.p
            piece.poly_y   = poly.py.p
            piece.poly_z   = poly.pz.p
            piece.poly_yaw = poly.pyaw.p
            pieces.append(piece)
        self.uploadTrajectoryService(trajectoryId, pieceOffset, pieces)

    def startTrajectory(self, trajectoryId, timescale = 1.0, reverse = False, relative = True, groupMask = 0):
        self.startTrajectoryService(groupMask, trajectoryId, timescale, reverse, relative)

    def position(self):
        self.tf.waitForTransform("/world", self.my_frame, rospy.Time(0), rospy.Duration(10))
        position, quaternion = self.tf.lookupTransform("/world", self.my_frame, rospy.Time(0))
        return np.array(position)

    def getParam(self, name):
        return rospy.get_param(self.prefix + "/" + name)

    def setParam(self, name, value):
        rospy.set_param(self.prefix + "/" + name, value)
        self.updateParamsService([name])

    def setParams(self, params):
        for name, value in params.iteritems():
            rospy.set_param(self.prefix + "/" + name, value)
        self.updateParamsService(params.keys())

    def cmdHover(self, vx, vy, yawrate, zDistance):
        self.cmdHoverMsg.header.stamp = rospy.Time.now()
        self.cmdHoverMsg.header.seq += 1
        self.cmdHoverMsg.vx        = vx
        self.cmdHoverMsg.vy        = vy
        self.cmdHoverMsg.yawrate   = yawrate
        self.cmdHoverMsg.zDistance = zDistance
        self.cmdHoverPublisher.publish(self.cmdHoverMsg)

    def cmdPosition(self, pos, yaw):
        self.cmdPositionMsg.header.stamp = rospy.Time.now()
        self.cmdPositionMsg.header.seq += 1
        self.cmdPositionMsg.x   = pos[0]
        self.cmdPositionMsg.y   = pos[1]
        self.cmdPositionMsg.z   = pos[2]
        self.cmdPositionMsg.yaw = yaw
        self.cmdPositionPublisher.publish(self.cmdPositionMsg)

    def cmdFullState(self, pos, vel, acc, yaw, omega):
        self.cmdFullStateMsg.header.stamp = rospy.Time.now()
        self.cmdFullStateMsg.header.seq += 1
        self.cmdFullStateMsg.pose.position.x    = pos[0]
        self.cmdFullStateMsg.pose.position.y    = pos[1]
        self.cmdFullStateMsg.pose.position.z    = pos[2]
        self.cmdFullStateMsg.twist.linear.x     = vel[0]
        self.cmdFullStateMsg.twist.linear.y     = vel[1]
        self.cmdFullStateMsg.twist.linear.z     = vel[2]
        self.cmdFullStateMsg.acc.x              = acc[0]
        self.cmdFullStateMsg.acc.y              = acc[1]
        self.cmdFullStateMsg.acc.z              = acc[2]
        self.cmdFullStateMsg.pose.orientation   = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, yaw))
        self.cmdFullStateMsg.twist.angular.x    = omega[0]
        self.cmdFullStateMsg.twist.angular.y    = omega[1]
        self.cmdFullStateMsg.twist.angular.z    = omega[2]
        self.cmdFullStatePublisher.publish(self.cmdFullStateMsg)

    def cmdStop(self):
        self.cmdStopPublisher.publish(std_msgs.msg.Empty())
