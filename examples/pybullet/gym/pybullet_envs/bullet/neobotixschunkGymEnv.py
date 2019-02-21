import os, inspect
#
# currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
# print("current_dir=" + currentdir)
# os.sys.path.insert(0, currentdir)

import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import pybullet as p
import pyglet
# from . import neobotixschunk
# import pybullet_data
import time
import sys
sys.path.append('/home/lei/Documents/Projektpraktikum/Pybullet/bullet3/examples/pybullet/gym/pybullet_data')

import random
import neobotixschunk
from pkg_resources import parse_version

pyglet.clock.set_fps_limit(10000)


largeValObservation = 100

RENDER_HEIGHT = 720
RENDER_WIDTH = 960


class NeobotixSchunkGymEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }

    def __init__(self,
                 urdfRoot='/home/lei/Documents/Projektpraktikum/Pybullet/bullet3/examples/pybullet/gym/pybullet_data',
                 actionRepeat=50,
                 isEnableSelfCollision=True,
                 isDiscrete=False,
                 renders=False,
                 maxSteps=1000,
                 rewardtype='rdense'):
        # print("init")
        self._timeStep = 0.01
        self._urdfRoot = urdfRoot
        self._actionRepeat = actionRepeat
        self._isEnableSelfCollision = isEnableSelfCollision
        self._observation = []
        self._envStepCounter = 0
        self._renders = renders
        self._rewardtype = rewardtype
        self._maxSteps = maxSteps
        self._isDiscrete = isDiscrete
        self._terminated = 0
        self._cam_dist = 1.3
        self._cam_yaw = 180
        self._cam_pitch = -40
        self._p = p
        self._dis_vor = 100
        self._count = 0
        if self._renders:
            cid = p.connect(p.SHARED_MEMORY)
            if (cid < 0):
                cid = p.connect(p.GUI)
            p.resetDebugVisualizerCamera(self._cam_dist, self._cam_yaw, self._cam_pitch, [0.52, -0.2, -0.33])
        else:
            p.connect(p.DIRECT)
        self._seed()
        self.reset()
        observation_dim = len(self.getExtendedObservation())
        # print("observationDim")
        # print(observationDim)
        # observation_high = np.array([np.finfo(np.float32).max] * observationDim)

        observation_high = np.array([largeValObservation] * observation_dim)

        if (self._isDiscrete):
            self.action_space = spaces.Discrete(9)
        else:
            action_dim = 5
            self._action_bound = 1
            action_high = np.array([self._action_bound] * action_dim)
            self.action_space = spaces.Box(low=-action_high, high=action_high, dtype=np.float32)

        self.observation_space = spaces.Box(low=-observation_high, high=observation_high, dtype=np.float32)
        self.viewer = None

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _reset(self):
        self.terminated = 0
        p.resetSimulation()
        p.setTimeStep(self._timeStep)
        self._p.setGravity(0, 0, -9.8)
        p.setPhysicsEngineParameter(numSolverIterations=150)

        p.loadURDF(os.path.join(self._urdfRoot, "plane.urdf"), [0, 0, 0])
        # print 'path', self._urdfRoot
        d_space_scale = len(str(abs(self._count))) * 0.5
        print('scale here: ', self._count, d_space_scale, self._maxSteps)

        xpos = np.random.uniform(-d_space_scale, d_space_scale)+0.20
        ypos = np.random.uniform(-d_space_scale, d_space_scale)
        zpos = np.random.uniform(0.4, 1.3)
        self.goal = np.array([xpos, ypos, zpos])

        self.goalUid = p.loadURDF(os.path.join(self._urdfRoot, "sphere_small.urdf"), xpos, ypos, zpos)
        #private variante
        self._neobotixschunk = neobotixschunk.NeobotixSchunk(urdfRootPath=self._urdfRoot, timeStep=self._timeStep)
        self._envStepCounter = 0
        p.stepSimulation()
        self._observation = self.getExtendedObservation()

        eedisvec = np.subtract(self._observation[0:3], self.goal)
        self.dis_init = np.linalg.norm(eedisvec)

        return np.array(self._observation)

    #return the endeffector vec9 [position(vec3),orientation(euler angles)(vec3),goalPosInEndeffector(vec3)],distance, goal position
    def getExtendedObservation(self):
        self._observation = self._neobotixschunk.getObservation()
        EndeffectorState = p.getLinkState(self._neobotixschunk.neobotixschunkUid, self._neobotixschunk.neobotixschunkEndEffectorIndex)
        # EndeffectorState= self._observation[0:3]
        # print('EndeffectorState',EndeffectorState)
        # show the position of endeffector, is vec3
        EndeffectorrPos = self._observation[0:3]
        # print('EndeffectorrPos',EndeffectorrPos)
        #show the orientation of endeffector, is vec4 in quaternion
        EndeffectorOrn = EndeffectorState[1]
        # print('EndeffectorOrn', EndeffectorOrn)
        # returns the position(vec3) and quaternion orientation(vec4)
        goalPos, goalOrn = p.getBasePositionAndOrientation(self.goalUid)
        #show the inverse transformed matrix
        invEndeffectorPos, invEndeffectorOrn = self._p.invertTransform(EndeffectorrPos, EndeffectorOrn)
        #multiply the transformed matrix and goal position
        goalPosInEndeffector, goalOrnInEndeffector = self._p.multiplyTransforms(invEndeffectorPos, invEndeffectorOrn, goalPos, goalOrn)
        # print('goalPosInEndeffector',goalPosInEndeffector)
        goalInEndeffectorPosXYEulZ = [goalPosInEndeffector[0], goalPosInEndeffector[1], goalPosInEndeffector[2]]
        #at end of list is relative coordinate system (goal location in endeffector position)
        self._observation.extend(list(goalInEndeffectorPosXYEulZ))
        return self._observation


    def __del__(self):
        p.disconnect()

    def _step(self, action):
        p_scale = 0.01
        action_scaled = np.multiply(action, self._action_bound * p_scale)
        for i in range(self._actionRepeat):
            self._neobotixschunk.applyAction2(action_scaled)
            p.stepSimulation()
            if self._termination():
                break
            self._envStepCounter += 1

        if self._renders:
            time.sleep(self._timeStep)

        self._observation = self.getExtendedObservation()
        self._actions = action
        reward = self._reward()
        done = self._termination()
        return np.array(self._observation), reward, done, {}

    def _termination(self):
        self._observation = self.getExtendedObservation()
        # state = p.getLinkState(self._neobotixschunk.neobotixschunkUid, self._neobotixschunk.neobotixschunkEndEffectorIndex)
        # actualEndEffectorPos = state[0]

        if self._terminated or (self._envStepCounter > self._maxSteps):
            # self._observation = self.getExtendedObservation()
            return True

        disvec = np.subtract(self._observation[0:3], self.goal)
        self.ee_dis = np.linalg.norm(disvec)
        #calculate the linear algebra normiert distance
        dis = np.linalg.norm(disvec)

        #base and goal distance
        bdisvec = np.subtract(self._observation[6:8], self.goal[0:2])
        self.base_dis = np.linalg.norm(bdisvec)

        if self.ee_dis < 0.05:
            self._terminated = 1
            self._count += 1
            # self._observation = self.getExtendedObservation()
            print('terminate:', self._observation, self.ee_dis, self.goal)
            return True

        return False

    def _reward(self):
    # rewards is accuracy of target position
    #     closestPoints = self._p.getClosestPoints(self._neobotixschunk.neobotixschunkUid, self.goalUid, 1000,self._neobotixschunk.neobotixschunkEndEffectorIndex,-1)
    #
    #     numPt = len(closestPoints)
    #     reward = -1000
    #     if (numPt > 0):
    #         reward = -closestPoints[0][8]  # contact distance
    #     return reward
    #     state = p.getLinkState(self._neobotixschunk.neobotixschunkUid, self._neobotixschunk.neobotixschunkEndEffectorIndex)
    #     actualEndEffectorPos = state[0]
    #     disvec = [x - y for x, y in zip(actualEndEffectorPos, self.goal)]
    #     self.dis = np.linalg.norm(disvec)
        delta_dis = self.ee_dis - self._dis_vor
        self._dis_vor = self.ee_dis

        tau = (self.ee_dis / self.dis_init) ** 2 #power
        if tau > 1:
            penalty = (1 - tau) * self.ee_dis + self._envStepCounter / self._maxSteps / 2
        else:
            penalty = self._envStepCounter / self._maxSteps / 2

        if self._rewardtype == 'rdense':
            reward = (1-tau)*self.ee_dis + tau*self.base_dis - penalty
            reward = -reward

        elif self._rewardtype == 'rsparse':
            if delta_dis > 0:
                reward = 0
            else:
                reward = 1
        return reward

    def _render(self, mode='human', close=False):
        if mode != "rgb_array":
            return np.array([])
        base_pos, orn = self._p.getBasePositionAndOrientation(self._neobotixschunk.neobotixschunkUid)
        view_matrix = self._p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=base_pos,
            distance=self._cam_dist,
            yaw=self._cam_yaw,
            pitch=self._cam_pitch,
            roll=0,
            upAxisIndex=2)
        proj_matrix = self._p.computeProjectionMatrixFOV(
            fov=60, aspect=float(RENDER_WIDTH) / RENDER_HEIGHT,
            nearVal=0.1, farVal=100.0)
        (_, _, px, _, _) = self._p.getCameraImage(
            width=RENDER_WIDTH, height=RENDER_HEIGHT, viewMatrix=view_matrix,
            projectionMatrix=proj_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        rgb_array = np.array(px)
        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    if parse_version(gym.__version__)>=parse_version('0.9.6'):
        render = _render
        reset = _reset
        seed = _seed
        step = _step

