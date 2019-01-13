import os,  inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)


import pybullet as p
import numpy as np
import pybullet_data
import math


class NeobotixSchunk:

    def __init__(self, urdfRootPath=pybullet_data.getDataPath(), timeStep=0.01):
        self.urdfRootPath = urdfRootPath
        self.timeStep = timeStep
        self.maxVelocity = .35
        self.maxForce = 100
        self.useSimulation = 1
        self.useNullSpace = 0
        self.useOrientation = 1
        self.neobotixschunkEndEffectorIndex = 15
        self.reset()
        # joint damping coefficents
        self.jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

    def reset(self):
        objects = p.loadURDF(os.path.join(self.urdfRootPath,"NeobotixSchunk/NeobotixSchunk.urdf"))
        self.neobotixschunkUid = objects[0]

        for i in range(p.getNumJoints(self.neobotixschunkUid)):
            print(p.getJointInfo(self.neobotixschunkUid, i))

        self.wheels = [0, 1]
        self.joints = [2, 3, 4, 5, 6, 7, 8]

        initial_wheelVel=[0, 0]
        initial_jointstate=[0, 0, 0, 0, 0, 0, 0]

        for wheel in range(self.wheels):
            p.resetjointState(self.neobotixschunkUid, wheel,initial_wheelVel)
            p.setJointMotorControl2(self.neobotixschunkUid, wheel, p.VELOCITY_CONTROL, targetVelocity=initial_wheelVel[wheel],force=self.maxForce)

        for joint in range(self.joints):
            p.resetjointState(self.neobotixschunkUid,joint,initial_jointstate)
            p.setJointMotorControl2(self.neobotixschunkUid, joint, p.POSITION_CONTROL,targetPosition=initial_jointstate[joint], force=self.maxForce)

    def getActionDimension(self):
        return 9

    def getObservationDimension(self):
        return len(self.getObservation())

    def getObservation(self):
        observation = []
        state = p.getLinkState(self.neobotixschunkUid, self.neobotixschunkEndEffectorIndex)
        pos = state[0]
        orn = state[1]
        euler = p.getEulerFromQuaternion(orn)

        observation.extend(list(pos))
        observation.extend(list(euler))

        return observation

    def applyAction(self, Commands):
        targetVelocity = Commands[0]
        targetangle = Commands[1]
        for motor in self.wheels:
            p.setJointMotorControl2(self.neobotixschunkUid, motor,p.VELOCITY_CONTROL,
                                          targetVelocity=targetVelocity, force=self.maxForce)
        for motor_2 in self.joints:
            p.setJointMotorControl2(self.neobotixschunkUid, motor_2, p.POSITION_CONTROL,
                                          targetPosition=targetangle)
