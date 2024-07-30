import pybullet as p
import time
import pybullet_data
import math
from collections import namedtuple
from attrdict import AttrDict

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)

p.resetDebugVisualizerCamera(cameraDistance=2,cameraYaw=0,cameraPitch=-40,cameraTargetPosition=[0.5,-0.9,0.5])#转变视角


planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("/home/zz/pddlstream/examples/pybullet/loopy/sim-master/urdfs/ur5.urdf",useFixedBase = True)

#登记各个节点的信息
jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
numJoints = p.getNumJoints(robotId)
jointInfo = namedtuple("jointInfo",["id","name","type","lowerLimit","upperLimit","maxForce","maxVelocity"])
joints = AttrDict()
for i in range(numJoints):
    info = p.getJointInfo(robotId,i)
    jointID = info[0]
    jointName = info[1].decode('utf-8')
    jointType = jointTypeList[info[2]] 
    jointLowerLimit = info[8]
    jointUpperLimit = info[9]
    jointMaxForce = info[10]
    jointMaxVelocity = info[11]
    singleInfo = jointInfo(jointID,jointName,jointType,jointLowerLimit,jointUpperLimit,jointMaxForce,jointMaxVelocity)
    joints[singleInfo.name] = singleInfo
print(joints)

for jointName in joints:
    print("jointName:",jointName)
position_control_group = []
position_control_group.append(p.addUserDebugParameter('world_joint', -math.pi, math.pi, 0))
position_control_group.append(p.addUserDebugParameter('base_joint', -2.0 / 3 * math.pi, -1.0 / 3 * math.pi, -0.5 * math.pi))
position_control_group.append(p.addUserDebugParameter('shoulder_joint', -0.5 * math.pi, 0.5 * math.pi, 0))
position_control_group.append(p.addUserDebugParameter('elbow_joint', -math.pi, 0, -0.5 * math.pi))
position_control_group.append(p.addUserDebugParameter('wrist_1_joint', -0.5 * math.pi, 0.5 * math.pi, 0))
position_control_group.append(p.addUserDebugParameter('wrist_2_joint', -math.pi, math.pi, 0))
position_control_group.append(p.addUserDebugParameter('wrist_3_joint', -math.pi, math.pi, 0))


position_control_joint_name = ['world_joint','base_joint','shoulder_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
print("position_control_group:",position_control_group)
while True:
    time.sleep(0.01)
    #p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)  # 允许机械臂慢慢渲染
    
    parameter = []
    for i in range(7):
        parameter.append(p.readUserDebugParameter(position_control_group[i]))
    num = 0
    # print("parameter:",parameter)
    for jointName in joints:
        if jointName in position_control_joint_name:
            joint = joints[jointName]
            parameter_sim = parameter[num]
            p.setJointMotorControl2(robotId, joint.id, p.POSITION_CONTROL,
                                    targetPosition=parameter_sim,
                                    force=joint.maxForce,
                                    maxVelocity=joint.maxVelocity)
            num = num + 1
    p.stepSimulation()


     










# #!/usr/bin/env python

# from __future__ import print_function

# from pddlstream.algorithms.meta import solve, create_parser
# from examples.pybullet.utils.pybullet_tools.loopy_primitives import BodyPose, BodyConf, Command, get_grasp_gen, \
#     get_stable_gen, get_ik_fn, get_free_motion_gen, \
#     get_holding_motion_gen, get_cfree_obj_approach_pose_test, get_cfree_pose_pose_test,\
#     get_movable_collision_test, plan_joint_motion
# from examples.pybullet.utils.pybullet_tools.kuka_primitives import plan_direct_joint_motion
# from examples.pybullet.utils.pybullet_tools.utils import WorldSaver, connect, dump_body, get_pose, set_pose, Pose, \
#     Point, set_default_camera, stable_z, \
#     BLOCK_URDF, SMALL_BLOCK_URDF, get_configuration, SINK_URDF, STOVE_URDF, load_model, is_placement, get_body_name, \
#     disconnect, DRAKE_IIWA_URDF, get_bodies, HideOutput, wait_for_user, KUKA_IIWA_URDF, add_data_path, load_pybullet, \
#     LockRenderer, has_gui, draw_pose, draw_global_system,UR5_UNI_ARM_URDF, UR5_URDF,\
#     get_movable_joints, set_joint_positions, get_link_state,disable_real_time, enable_gravity, joint_controller,\
#     step_simulation
# from pddlstream.language.generator import from_gen_fn, from_fn, empty_gen, from_test, universe_test
# from pddlstream.utils import read, INF, get_file_path, find_unique, Profiler, str_from_object, negate_test
# from pddlstream.language.constants import print_solution, PDDLProblem
# # from examples.pybullet.tamp.streams import get_cfree_approach_pose_test, get_cfree_pose_pose_test, get_cfree_traj_pose_test, \
# #     move_cost_fn, get_cfree_obj_approach_pose_test
# import pybullet as p
# import time
# HOME_CONF=[0,-1.57,1.57,0,0,0]
# # HOME_CONF=[-6.666624994380624, -6.289973104499651, -2.719662298974622, -5.458182921074028, -3.2201010639495706, -6.276911581047255]

# #######################################################
# def load_world():
#     # TODO: store internal world info here to be reloaded
#     set_default_camera()
#     draw_global_system()
#     with HideOutput():
#         #add_data_path()
#         # robot = load_model(UR5_UNI_ARM_URDF, fixed_base=True) # DRAKE_IIWA_URDF | KUKA_IIWA_URDF
#         robot = load_model(UR5_URDF, fixed_base=True) # DRAKE_IIWA_URDF | KUKA_IIWA_URDF
#         # floor = load_model('models/short_floor.urdf')
#         floor = load_pybullet("plane.urdf")
#         sink = load_model(SINK_URDF, pose=Pose(Point(x=-0.5)))
#         stove = load_model(STOVE_URDF, pose=Pose(Point(x=+0.5)))
#         celery = load_model(BLOCK_URDF, fixed_base=False)
#         radish = load_model(SMALL_BLOCK_URDF, fixed_base=False)
#         #cup = load_model('models/dinnerware/cup/cup_small.urdf',
#         # Pose(Point(x=+0.5, y=+0.5, z=0.5)), fixed_base=False)

#     # draw_pose(Pose(), parent=robot, parent_link=get_tool_link(robot)) # TODO: not working
#     # dump_body(robot)
#     # wait_for_user()

#     body_names = {
#         sink: 'sink',
#         stove: 'stove',
#         celery: 'celery',
#         radish: 'radish',
#     }
#     movable_bodies = [celery, radish]

#     movable_joints = get_movable_joints(robot)
#     print("执行set_joint")
#     set_joint_positions(robot, movable_joints, HOME_CONF)
#     print(get_link_state(robot, 7))


#     set_pose(celery, Pose(Point(y=0.5, z=stable_z(celery, floor))))
#     set_pose(radish, Pose(Point(y=-0.5, z=stable_z(radish, floor))))

#     return robot, body_names, movable_bodies

# def plan_path(initial_joint_positions, target_joint_positions):
#     num_steps = 100
#     path = []
#     for i in range(num_steps + 1):
#         interpolated_positions = [
#             initial + (target - initial) * i / num_steps
#             for initial, target in zip(initial_joint_positions, target_joint_positions)
#         ]
#         path.append(interpolated_positions)
#     return path

# #######################################################

# def main():
#     parser = create_parser()
#     parser.add_argument('-enable', action='store_true', help='Enables rendering during planning')
#     parser.add_argument('-teleport', action='store_true', help='Teleports between configurations')
#     parser.add_argument('-simulate', action='store_true', help='Simulates the system')
#     args = parser.parse_args()
#     print('Arguments:', args)

#     connect(use_gui=True)
#     robot, names, movable = load_world()
#     print('Objects:', names)
#     saver = WorldSaver()

#     # 单个位姿
#     conf=[56.9022734602204, 46.800680880004705, -29.70772076602826, 97.60460192873298, 70.25756172403447, 14.072923841888288]
#     movable_joints = get_movable_joints(robot)
#     print("执行set_joint")
#     set_joint_positions(robot, movable_joints, conf)

    
#     try :
#         while True:
#             # plan_direct_joint_motion()路径规划测试
#                 start_conf = [0, 0, -1.57, 1.57, 0, 0]
#                 end_conf = [67.44512450074156, 75.82868589045351, -70.70962391721162, -85.26258456177109, -60.83342267525028, -7.952706686963723]
#                 traj = plan_path(start_conf, end_conf)
#                 print(traj)
#                 # print(movable_joints)
#                 # p.setGravity(0, 0, -9.81)
#                 # disable_real_time()
#                 # for values in traj:
#                 #     for _ in joint_controller(robot, movable_joints, values):
#                 #             enable_gravity()
#                 #             step_simulation()
#                 #             time.sleep(0)
#                 for i, configuration in enumerate(traj):
#                     set_joint_positions(robot, movable_joints, configuration)
#                     yield i


#                 # for value in traj:
#                 #      for i in movable_joints:
#                 #         # print(i)
#                 #         p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, targetPosition = value[i-1],
#                 #                                 targetVelocity=0,force=500,positionGain=0.03,velocityGain=1)
#                         # p.stepSimulation()
#                         # time.sleep(1./240.)
#     except KeyboardInterrupt:
#     # 用户中断程序时，退出循环
#         print("Circle drawing interrupted by user.")
#     wait_for_user('Finish?')
#     disconnect()

# if __name__ == '__main__':
#     main()