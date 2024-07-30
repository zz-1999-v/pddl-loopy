# import pybullet as p

# import pybullet_data as pd
# import time
# import random
# import pickle

# class RobotCtrl:

#     def __init__(self, robot) -> None:
#         # initial information
#         self.robot = robot
#         self.position_home = [-3.14, -1.347, 1.432, -1.853, -1.558, 0.000, 0.0]
#         self.acc_DOF = 6

#         # set up collision between 2 links
#         p.setCollisionFilterPair(robot, robot , linkIndexA=7, linkIndexB=8, enableCollision=True)
#         p.setCollisionFilterPair(robot, robot , linkIndexA=7, linkIndexB=15, enableCollision=True)
#         p.setCollisionFilterPair(robot, robot , linkIndexA=7, linkIndexB=22, enableCollision=True)
#         p.setCollisionFilterPair(robot, robot , linkIndexA=7, linkIndexB=29, enableCollision=True)
#         p.setCollisionFilterPair(robot, robot , linkIndexA=7, linkIndexB=36, enableCollision=True)

#         # initialize the joints states
#         joint_num = p.getNumJoints(robot)
        
#         controllable_joint_id = list(x for x in range(joint_num) \
#                                  if p.getJointInfo(robot, x)[2] != p.JOINT_FIXED)[: self.acc_DOF]
        
#         # judge the number of controllable joints
#         assert len(controllable_joint_id) > 0

#         # set controllable joint initial properties
#         for pos_index, joint_id in enumerate(controllable_joint_id):
#             p.changeDynamics(robot, joint_id, linearDamping=0, angularDamping=0)
#             p.resetJointState(robot, joint_id, self.position_home[pos_index])

#     pass


# def load_world():
#     robot = arm_controller_id = p.loadURDF('/home/zz/lupy/urdf/urdf/ur5_hand.urdf',basePosition=[0, 0, 0.01])
#     # table = p.loadURDF("table/table.urdf", [0,0,-2*0.625],  flags = p.URDF_USE_INERTIA_FROM_FILE ,globalScaling=2.0)
#     gaizi= p.loadURDF('/home/zz/lupy/urdf/urdf/gaizi.urdf', [-0.4, 0.4 ,0.2] ,      
#                      baseOrientation = p.getQuaternionFromEuler([ 0, 0, 0]),
#                      useFixedBase=False)
#     init_robot = RobotCtrl(robot=robot)

#     return robot, gaizi 

#     pass

# def main():
#     client = p.connect(p.GUI)
#     robot, gaizi = load_world()

#     # 创建一个固定关节，将机器人与立方体连接
#     constraint_id = p.createConstraint(robot, -1, gaizi, -1,
#                                    p.JOINT_FIXED, jointAxis=[0, 0, 0],
#                                    parentFramePosition=[0, 0, 0],
#                                    childFramePosition=[0, 0, 0])

#     # 运行仿真一段时间，观察初始约束效果
#     p.setGravity(0, 0, -10)


#     for _ in range(50000000):
#         p.stepSimulation()
#         time.sleep(1./240.)






# if __name__ == '__main__':    
#     main()











import pybullet as p
import pybullet_data
import time

# # 定义RobotCtrl类（假设这是你的机器人控制类）
# class RobotCtrl:
#     def __init__(self, robot):
#         self.robot = robot

# def load_world():
#     # 连接到物理引擎
#     p.connect(p.GUI)
#     p.setAdditionalSearchPath(pybullet_data.getDataPath())

#     # 加载机器人和盖子模型
#     robot = p.loadURDF('/home/zz/pddlstream/examples/pybullet/loopy/sim-master/urdfs/ur5.urdf')
#     gaizi = p.loadURDF('/home/zz/lupy/urdf/urdf/gaizi.urdf', [-0.4, 0.4, 0.2], 
#                        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
#                        useFixedBase=False)

    # # 创建一个固定关节来连接机器人和盖子
    # constraint = p.createConstraint(robot, 43, gaizi, 1, p.JOINT_FIXED,
    #                                 jointAxis=[0, 0, 0], parentFramePosition=[0, 0, 0],
    #                                 childFramePosition=[0, 0, 0],
    #                                 parentFrameOrientation=[0, 0, 0, 1],
    #                                 childFrameOrientation=[0, 0, 0, 1])

    # # 可选：设置约束的最大力
    # max_force = 100
    # p.changeConstraint(constraint, maxForce=max_force)

    # 返回机器人和盖子对象及约束对象
    # return robot, gaizi
# 启动仿真引擎的GUI
p.connect(p.GUI)

# 设置重力加速度
p.setGravity(0, 0, -9.81)

# 加载URDF模型路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# 加载仿真世界并获取对象及约束
robotId  = p.loadURDF("/home/zz/pddlstream/examples/pybullet/loopy/sim-master/urdfs/ur5.urdf",useFixedBase=True )  #basePosition=[0.0,0,0.62]

# 获取机械臂末端执行器的索引
endEffectorIndex =6

# 获取机械臂的关节数量
numJoints = p.getNumJoints(robotId)
print("关节数量:"+str(numJoints))

# 打印每个关节的信息
for joint_index in range(numJoints):
    joint_info = p.getJointInfo(robotId, joint_index)
    print(f"Joint {joint_index}: {joint_info}")

# 机械臂的初始位置
restingPosition = [0,3.14, -1.57, 1.57, 1.57, 1.57, -1.57, 0]
for jointNumber in range(numJoints):
    p.resetJointState(robotId, jointNumber, restingPosition[jointNumber])

try:
    while True:
        # 移动机械臂
            jointPoses = p.calculateInverseKinematics(robotId, endEffectorIndex, [50, 50, 50],[0, 0, 0, 1])
            p.setJointMotorControl2(bodyIndex=robotId,jointIndex=1,controlMode=p.POSITION_CONTROL,targetPosition=jointPoses[0],targetVelocity=0,force=500,positionGain=0.03,velocityGain=1)
            p.setJointMotorControl2(bodyIndex=robotId,jointIndex=2,controlMode=p.POSITION_CONTROL,targetPosition=jointPoses[1],targetVelocity=0,force=500,positionGain=0.03,velocityGain=1)
            p.setJointMotorControl2(bodyIndex=robotId,jointIndex=3,controlMode=p.POSITION_CONTROL,targetPosition=jointPoses[2],targetVelocity=0,force=500,positionGain=0.03,velocityGain=1)
            p.setJointMotorControl2(bodyIndex=robotId,jointIndex=4,controlMode=p.POSITION_CONTROL,targetPosition=jointPoses[3],targetVelocity=0,force=500,positionGain=0.03,velocityGain=1)
            p.setJointMotorControl2(bodyIndex=robotId,jointIndex=5,controlMode=p.POSITION_CONTROL,targetPosition=jointPoses[4],targetVelocity=0,force=500,positionGain=0.03,velocityGain=1)
            p.setJointMotorControl2(bodyIndex=robotId,jointIndex=6,controlMode=p.POSITION_CONTROL,targetPosition=jointPoses[5],targetVelocity=0,force=500,positionGain=0.03,velocityGain=1)

            num_joints = p.getNumJoints(robotId)

            # 遍历所有链接，获取链接的详细信息
            for i in range(num_joints):
                joint_info = p.getJointInfo(robotId, i)
                link_name = joint_info[1].decode('UTF-8')  # 获取链接名称
                link_id = joint_info[0]  # 获取链接的唯一ID
                print(f"Link Name: {link_name}, Link ID: {link_id}")


            p.stepSimulation()
            time.sleep(0.01)

except KeyboardInterrupt:
    # 用户中断程序时，退出循环
    print("Circle drawing interrupted by user.")
# 断开与仿真引擎的连接
p.disconnect()