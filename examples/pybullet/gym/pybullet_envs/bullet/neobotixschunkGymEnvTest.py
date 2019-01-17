import sys
sys.path.append('/home/lei/Documents/Projektpraktikum/Pybullet/bullet3/examples/pybullet/gym/pybullet_envs')
#print ('ff',sys.path)
import neobotixschunk
import neobotixschunkGymEnv

def main():

    environment = neobotixschunkGymEnv.NeobotixSchunkGymEnv(renders=True,isDiscrete=False, maxSteps=100000000)
    leftwheelVelocitySlider = environment._p.addUserDebugParameter("left_wheel_Velocity",-1,1,0)
    rightwheelVelocitySlider = environment._p.addUserDebugParameter("right_wheel_Velocity",-1,1,0)
    joint_1_Slider=environment._p.addUserDebugParameter("arm_1_joint",-3.1215926,3.1215926,0)
    joint_2_Slider=environment._p.addUserDebugParameter("arm_2_joint",-2.12,2.12,0)
    joint_3_Slider=environment._p.addUserDebugParameter("arm_3_joint",-3.1215926,3.1215926,0)
    joint_4_Slider=environment._p.addUserDebugParameter("arm_4_joint",-2.16,2.16,0)
    joint_5_Slider=environment._p.addUserDebugParameter("arm_5_joint",-3.1215926,3.1215926,0)
    joint_6_Slider=environment._p.addUserDebugParameter("arm_6_joint",-2.07,2.07,0)
    joint_7_Slider=environment._p.addUserDebugParameter("arm_7_joint",-2.94,2.94,0)
    actionIds = [leftwheelVelocitySlider, rightwheelVelocitySlider, joint_1_Slider, joint_2_Slider, joint_3_Slider, joint_4_Slider, joint_5_Slider, joint_6_Slider, joint_7_Slider]

    done = 0
    while(not done):
        # leftwheelVelocity = environment._p.readUserDebugParameter(leftwheelVelocitySlider)
        # rightwheelVelocity = environment._p.readUserDebugParameter(rightwheelVelocitySlider)
        # joint_1 = environment._p.readUserDebugParameter(joint_1_Slider)
        # joint_2 = environment._p.readUserDebugParameter(joint_2_Slider)
        # joint_3 = environment._p.readUserDebugParameter(joint_3_Slider)
        # joint_4 = environment._p.readUserDebugParameter(joint_4_Slider)
        # joint_5 = environment._p.readUserDebugParameter(joint_5_Slider)
        # joint_6 = environment._p.readUserDebugParameter(joint_6_Slider)
        # joint_7 = environment._p.readUserDebugParameter(joint_7_Slider)
        # action = [leftwheelVelocity, rightwheelVelocity, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7]
        # print(action)
        action = []
        for actionId in actionIds:
            action.append(environment._p.readUserDebugParameter(actionId))
        state, reward, done, info = environment.step(action)
        print 'ob', state
        obs = environment.getExtendedObservation()

if __name__=="__main__":
    main()
