import os, inspect
# currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
# parentdir = os.path.dirname(os.path.dirname(currentdir))
# os.sys.path.insert(0,parentdir)

import sys
sys.path.append('/home/lei/Documents/Projektpraktikum/Pybullet/bullet3/examples/pybullet/gym/pybullet_data')

import pybullet as p
import math
# import random
import numpy as np

URDF_USE_SELF_COLLISION=1

class MMNeobotixSchunk:

    def __init__(self,urdfRootPath='/home/lei/Documents/Projektpraktikum/Pybullet/bullet3/examples/pybullet/gym/pybullet_data/', timeStep=0.01, randomInitial = False):
        self.urdfRootPath = urdfRootPath
        self.timeStep = timeStep
        # self.randInitial = randomInitial
        self.maxVelocity = .35
        self.maxAng = 2
        self.maxForce = 100
        self.useInverseKinematics = 1
        self.useSimulation = 1
        # self.useOrientation = 1
        self.schunkEndEffectorIndex = 6
        self.wheels = [0, 1]
        self.joints = [0, 1, 2, 3, 4, 5, 6]
        self.reset()
        self.useNullSpace = 0
        self.j1_limit = np.pi  # limits for arm link 1, 3, 5
        self.j4_limit = 121/180*np.pi  # limits for arm link 4
        self.j6_limit = 115/180*np.pi  # limits for arm link 2, 6
        self.j7_limit = 170/180*np.pi  # limits for arm link 7
        # lower limits for null space
        self.ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
        # upper limits for null space
        self.ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
        # joint ranges for null space
        self.jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
        # restposes for null space
        self.rp = [0, 0, 0, 0, 0, 0, 0]
        # joint damping coefficents
        self.jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

    def reset(self):
        #p.setGravity(0, 0, -9.8)
        self.neoUID = p.loadURDF('/home/lei/Documents/Projektpraktikum/Pybullet/bullet3/examples/pybullet/gym/pybullet_data/neobotixschunk/base_mp500.urdf',
                                 [0.2, 0.3, 0])
        self.schunkUID = p.loadURDF('/home/lei/Documents/Projektpraktikum/Pybullet/bullet3/examples/pybullet/gym/pybullet_data/neobotixschunk/arm_lwa4d.urdf',
                                    [0.2, 0.3, 0.40],flags=p.URDF_USE_SELF_COLLISION)

        for i in range(p.getNumJoints(self.neoUID)):
            print(p.getJointInfo(self.neoUID, i))
        for i in range(p.getNumJoints(self.schunkUID)):
            print(p.getJointInfo(self.schunkUID, i))

        p.createConstraint(self.neoUID, -1, self.schunkUID, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [-0.2, 0., -0.1])

        initial_wheelVel = [0, 0]
        initial_jointstate = [0, 0, 0, 0, 0, 0, 0]
        self.basevelocity = 0
        self.baseangulervelocity = 0
        self.endEffectorPos = [0.19, 0.0, 1.147]
        self.numJoints=7

        #p.resetBaseVelocity(self.neoUID, initial_wheelVel, initial_wheelVel)
        for wheelIndex in range(len(self.wheels)):
            # reset no-zero base velocities
            # not necessary
            # p.resetJointState(self.neoUID, wheelIndex, self.wheelVel[wheelIndex], self.wheelVel[wheelIndex])
            p.setJointMotorControl2(self.neoUID, wheelIndex, controlMode=p.VELOCITY_CONTROL,
                                    targetVelocity=initial_wheelVel[wheelIndex], force=self.maxForce)
        neoPos, neoOrn = p.getBasePositionAndOrientation(self.neoUID)
        huskyEul = p.getEulerFromQuaternion(neoOrn)

        initial_schunkstate = p.getLinkState(self.schunkUID, self.schunkEndEffectorIndex)
        # print('kuka',initial_kukastate)
        self.schunkstate = [initial_schunkstate[0][0],initial_schunkstate[0][1], initial_schunkstate[0][2]]

        initial_base_vel = p.getBaseVelocity(self.neoUID)
        self.baseVel = 0
        self.baseAng = 0

        for joint in self.joints:
            p.resetJointState(self.schunkUID,joint,initial_jointstate[joint])
            p.setJointMotorControl2(self.schunkUID, joint, p.POSITION_CONTROL,targetPosition=initial_jointstate[joint], force=self.maxForce)


    def getObservationDimension(self):
        return len(self.getObservation())

    def getObservation(self):
        observation = []
        state = p.getLinkState(self.schunkUID, self.schunkEndEffectorIndex)
        # print(state)
        pos = state[0]
        orn = state[1]
        euler = p.getEulerFromQuaternion(orn)
        observation.extend(list(pos))
        observation.extend(list(euler))

        base = p.getBasePositionAndOrientation(self.neoUID)
        basepos = base[0]
        baseorn = base[1]
        baseeul = p.getEulerFromQuaternion(baseorn)
        observation.extend(list(basepos))
        observation.extend(list(baseeul))

        print("observation")
        print(observation)

        return observation

    def check_jointstates(self, joint_state, delta_j):
        # joint limits from lwa4d data sheet and modified based on rviz visual
        if np.abs(joint_state[0]) > self.j1_limit:
            joint_state[0] = joint_state[0] - delta_j[0]
        if np.abs(joint_state[1]) > self.j6_limit:
            joint_state[1] = joint_state[1] - delta_j[1]
        if np.abs(joint_state[2]) > self.j1_limit:
            joint_state[2] = joint_state[2] - delta_j[2]
        if np.abs(joint_state[3]) > self.j4_limit:
            joint_state[3] = joint_state[3] - delta_j[3]
        if np.abs(joint_state[4]) > self.j1_limit:
            joint_state[4] = joint_state[4] - delta_j[4]
        if np.abs(joint_state[5]) > self.j6_limit:
            joint_state[5] = joint_state[5] - delta_j[5]
        if np.abs(joint_state[6]) > self.j7_limit:
            joint_state[6] = joint_state[6] - delta_j[6]
        return joint_state

    def check_baseV(self, base_vel, delta_bv):
        if (abs(base_vel) > 1.5):
            base_vel =base_vel - delta_bv
        return base_vel

    def check_baseA(self, base_ang, delta_ba):
        if (abs(base_ang) > 2):
            base_ang =base_ang - delta_ba
        return base_ang

    def applyAction(self,action):
        dbasevelocity = action[0]
        dbaseangulervelocity = action[1]
        self.basevelocity = self.basevelocity + dbasevelocity
        self.baseangulervelocity = self.baseangulervelocity + dbaseangulervelocity

        self.basevelocity = self.check_baseV(self.basevelocity, dbasevelocity)
        self.baseangulervelocity = self.check_baseA(self.baseangulervelocity, dbaseangulervelocity)

        self.wheelVelR = (self.basevelocity + 0.2535 * self.baseangulervelocity) / 0.13
        self.wheelVelL = (self.basevelocity - 0.2535 * self.baseangulervelocity) / 0.13

        self.wheelstate = [self.wheelVelL, self.wheelVelR]

        if (self.useInverseKinematics == 1):
            dx = action[2]
            dy = action[3]
            dz = action[4]
            endstate = p.getLinkState(self.schunkUID, self.schunkEndEffectorIndex)
            actualposition = list(endstate[0])
            print 'ee pos:', actualposition
            actualposition[0] = actualposition[0] + dx
            actualposition[1] = actualposition[1] + dy
            actualposition[2] = actualposition[2] + dz
            position = actualposition

            print 'ee pos2:', position
            if (self.useNullSpace == 1):
                jointPoses = p.calculateInverseKinematics(self.schunkUID,self.schunkEndEffectorIndex,
                                                          position,
                                                          lowerLimits=self.ll, upperLimits=self.ul,
                                                          jointRanges=self.jr, restPoses=self.rp)
            else:

                jointPoses = p.calculateInverseKinematics(self.schunkUID, self.schunkEndEffectorIndex,
                                                          position)
                print("jointPoses")
                print(jointPoses)

            for motor in self.wheels:
                p.setJointMotorControl2(self.neoUID, motor, p.VELOCITY_CONTROL,
                                        targetVelocity=self.wheelstate[motor], force=self.maxForce)
            for motor_2 in self.joints:
                p.setJointMotorControl2(self.schunkUID, motor_2, p.POSITION_CONTROL,
                                        targetPosition=jointPoses[motor_2], force=self.maxForce)

        else:

            djoint = np.array(action[2:9])

            if self.useSimulation:
                baseve, baseva = p.getBaseVelocity(self.neoUID)
                self.basevelocity = baseve[0]
                self.baseangulervelocity = baseva[2]
                dae = []
                for joint in self.joints:
                    self.Jointstate = p.getJointState(self.schunkUID, joint)
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

            self.jointposition = self.jointposition + djoint
            self.jointposition = self.check_jointstates(self.jointposition, djoint)

            # print('targetAngle=', targetangle)
            for motor in self.wheels:
                p.setJointMotorControl2(self.neoUID, motor, p.VELOCITY_CONTROL,
                                        targetVelocity=self.wheelstate[motor], force=self.maxForce)
            for motor_2 in self.joints:
                p.setJointMotorControl2(self.schunkUID, motor_2, p.POSITION_CONTROL,
                                        targetPosition=self.jointposition[motor_2], force=self.maxForce)
