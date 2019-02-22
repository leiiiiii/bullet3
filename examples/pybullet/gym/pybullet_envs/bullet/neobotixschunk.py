#import os,  inspect
#
# currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
# parentdir = os.path.dirname(os.path.dirname(currentdir))
# os.sys.path.insert(0,'/home/lei/Documents/Projektpraktikum/Pybullet/bullet3/examples/pybullet/gym/')
import sys
sys.path.append('/home/lei/Documents/Projektpraktikum/Pybullet/bullet3/examples/pybullet/gym/pybullet_data')


import pybullet as p
import numpy as np
# import pybullet_data
import math


class NeobotixSchunk:

    def __init__(self, urdfRootPath='/home/lei/Documents/Projektpraktikum/Pybullet/bullet3/examples/pybullet/gym/pybullet_data/', timeStep=0.01):
        self.urdfRootPath = urdfRootPath
        self.timeStep = timeStep
        self.maxVelocity = 1.5
        self.maxAng=2
        self.maxForce = 100
        self.useSimulation = 1
        self.useNullSpace = 0
        self.useInverseKinematics = 1
        self.neobotixschunkEndEffectorIndex = 13
        self.wheels = [0, 1]
        self.joints = [6, 7, 8, 9, 10, 11, 12]
        self.j1_limit = 3.1415  # limits for arm link 1, 3, 5
        self.j4_limit = 2.1118  #121 / 180 * np.pi   limits for arm link 4
        self.j6_limit = 2.0071  #115 / 180 * np.pi  # limits for arm link 2, 6
        self.j7_limit = 2.9671  #170 / 180 * np.pi  # limits for arm link 7
        self.reset()

        # lower limits for null space
        self.ll = [-self.j1_limit,-self.j1_limit,-self.j1_limit, -self.j6_limit, -self.j1_limit, -self.j4_limit, -self.j1_limit, -self.j6_limit, -self.j7_limit]
        # upper limits for null space
        self.ul = [self.j1_limit,self.j1_limit,self.j1_limit, self.j6_limit, self.j1_limit, self.j4_limit, self.j1_limit, self.j6_limit, self.j7_limit]
        # joint ranges for null space
        self.jr = [6.283,6.283,6.283, 4.0142, 6.283, 4.2236, 6.283, 4.0142, 5.9344]
        # joint damping coefficents
        self.jd = [0.1,0.1,0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        # restposes for null space
        self.rp = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    def reset(self):
        #os.path.join: path combination and return
        #load urdf returns a uniqueId,a noninteger value
        self.neobotixschunkUid = p.loadURDF('/home/lei/Documents/Projektpraktikum/Pybullet/bullet3/examples/pybullet/gym/pybullet_data/neobotixschunk/NeobotixSchunk.urdf')

        for i in range(p.getNumJoints(self.neobotixschunkUid)):
            print(p.getJointInfo(self.neobotixschunkUid, i))


        initial_wheelVel=[0, 0]
        initial_jointstate=[0, 0, 0, 0, 0, 0, 0]
        self.basevelocity = 0
        self.baseangulervelocity = 0
        self.endEffectorPos = [0.19, 0.0, 1.147]


        for wheel in self.wheels:
            p.resetJointState(self.neobotixschunkUid, wheel,initial_wheelVel[wheel])
            p.setJointMotorControl2(self.neobotixschunkUid, wheel, p.VELOCITY_CONTROL, targetVelocity=initial_wheelVel[wheel],force=self.maxForce)

        for joint in self.joints:
            p.resetJointState(self.neobotixschunkUid,joint,initial_jointstate[joint-6])
            p.setJointMotorControl2(self.neobotixschunkUid, joint, p.POSITION_CONTROL,targetPosition=initial_jointstate[joint-6], force=self.maxForce)

    # def getActionDimension(self):
    #     return 5

    def getObservationDimension(self):
        return len(self.getObservation())

    #get the endeffector position and orientation(euler angles)
    def getObservation(self):
        observation = []
        state = p.getLinkState(self.neobotixschunkUid, self.neobotixschunkEndEffectorIndex)
        # print(state)
        pos = state[0]
        orn = state[1]
        euler = p.getEulerFromQuaternion(orn)
        observation.extend(list(pos))
        observation.extend(list(euler))

        base = p.getBasePositionAndOrientation(self.neobotixschunkUid)
        basepos= base[0]
        baseorn= base[1]
        baseeul = p.getEulerFromQuaternion(baseorn)
        observation.extend(list(basepos))
        observation.extend(list(baseeul))

        print("observation")
        print(observation)

        return observation

    def check_baseV(self, base_vel, delta_bv):
        if (abs(base_vel) > self.maxVelocity):
            base_vel = base_vel - delta_bv
        return base_vel

    def check_baseA(self, base_ang, delta_ba):
        if (abs(base_ang) > self.maxAng):
            base_ang = base_ang - delta_ba
        return base_ang

    def check_jointstate(self, jointstate, delta_j):
        if abs(jointstate[0]) > self.j1_limit:
            jointstate[0] = jointstate[0] - delta_j[0]
        if abs(jointstate[1]) > self.j6_limit:
            jointstate[1] = jointstate[1] - delta_j[1]
        if abs(jointstate[2]) > self.j1_limit:
            jointstate[2] = jointstate[2] - delta_j[2]
        if abs(jointstate[3]) > self.j4_limit:
            jointstate[3] = jointstate[3] - delta_j[3]
        if abs(jointstate[4]) > self.j1_limit:
            jointstate[4] = jointstate[4] - delta_j[4]
        if abs(jointstate[5]) > self.j6_limit:
            jointstate[5] = jointstate[5] - delta_j[5]
        if abs(jointstate[6]) > self.j7_limit:
            jointstate[6] = jointstate[6] - delta_j[6]
        return jointstate

    def applyAction(self, action):

        dbasevelocity = action[0]
        dbaseangulervelocity = action[1]
        self.basevelocity = self.basevelocity + dbasevelocity
        self.baseangulervelocity = self.baseangulervelocity + dbaseangulervelocity

        self.basevelocity = self.check_baseV(self.basevelocity, dbasevelocity)
        self.baseangulervelocity = self.check_baseA(self.baseangulervelocity, dbaseangulervelocity)

        self.wheelVelR = (self.basevelocity + 0.2535 * self.baseangulervelocity) / 0.13
        self.wheelVelL = (self.basevelocity - 0.2535 * self.baseangulervelocity) / 0.13

        self.wheelstate = [self.wheelVelL, self.wheelVelR]

        if (self.useInverseKinematics==1):
            dx = action[2]
            dy = action[3]
            dz = action[4]
            endstate = p.getLinkState(self.neobotixschunkUid, self.neobotixschunkEndEffectorIndex)
            actualposition=list(endstate[0])
            actualposition[0] = actualposition[0] + dx
            actualposition[1] = actualposition[1] + dy
            actualposition[2] = actualposition[2] + dz
            position = actualposition
            if (self.useNullSpace == 1):
                jointPoses = p.calculateInverseKinematics(self.neobotixschunkUid, self.neobotixschunkEndEffectorIndex, position,
                                                              lowerLimits=self.ll, upperLimits=self.ul,
                                                              jointRanges=self.jr, restPoses=self.rp)
            else:

                jointPoses = p.calculateInverseKinematics(self.neobotixschunkUid, self.neobotixschunkEndEffectorIndex, position)
                # print("jointPoses")
                # print(jointPoses)

            for motor in self.wheels:
                p.setJointMotorControl2(self.neobotixschunkUid, motor, p.VELOCITY_CONTROL,
                                        targetVelocity=self.wheelstate[motor], force=self.maxForce)
            for motor_2 in self.joints:
                p.setJointMotorControl2(self.neobotixschunkUid, motor_2, p.POSITION_CONTROL,
                                        targetPosition=jointPoses[motor_2-4 ], force=self.maxForce)

        else:

            djoint=np.array(action[2:9])

            if self.useSimulation:
                baseve, baseva = p.getBaseVelocity(self.neobotixschunkUid)
                self.basevelocity = baseve[0]
                self.baseangulervelocity = baseva[2]
                dae = []
                for joint in self.joints:
                    self.Jointstate = p.getJointState(self.neobotixschunkUid, joint)
                    self.jointstate = self.Jointstate[0]
                    dae.append(self.jointstate)
                self.jointposition = dae



        # dwe = []
        # for joint in self.wheels:
        #     self.Wheelstate = p.getJointState(self.neobotixschunkUid, joint)
        #     self.wheelstate = self.Wheelstate[1]
        #     dwe.append(self.wheelstate)

        # dwl = dtargetVelocityL+dwe[0]
        # dwr = dtargetVelocityR+dwe[1]
        # self.wheelstate= [dwl,dwr]

            self.jointposition= self.jointposition + djoint
            self.jointposition = self.check_jointstate(self.jointposition, djoint)

        # print('targetAngle=', targetangle)
            for motor in self.wheels:
                p.setJointMotorControl2(self.neobotixschunkUid, motor,p.VELOCITY_CONTROL,
                                          targetVelocity=self.wheelstate[motor], force=self.maxForce)
            for motor_2 in self.joints:
                p.setJointMotorControl2(self.neobotixschunkUid, motor_2, p.POSITION_CONTROL,
                                          targetPosition=self.jointposition[motor_2-6],force=self.maxForce)



