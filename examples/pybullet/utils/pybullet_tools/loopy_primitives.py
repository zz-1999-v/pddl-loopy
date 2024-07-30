import time
import pybullet as p
import numpy as np
import copy

from itertools import count

from examples.pybullet.utils.pybullet_tools.pr2_utils import get_top_grasps
from examples.pybullet.utils.pybullet_tools.utils import get_pose, set_pose,\
    is_pose_close, set_joint_positions, enable_real_time, disable_real_time, joint_controller, \
    enable_gravity, get_refine_fn, wait_for_duration, link_from_name, get_body_name, sample_placement, \
    end_effector_from_body, approach_from_grasp, GraspInfo, Pose, INF, Point, \
    pairwise_collision, remove_fixed_constraint, get_sample_fn, \
    step_simulation, refine_path,  get_joint_positions, dump_world, wait_if_gui, flatten,\
    get_extend_fn, get_distance_fn, multiply, invert, unit_point, unit_quat, get_link_pose,\
    body_from_end_effector, set_real_time


# from scipy.spatial.transform import Rotation as R
# # set ur5_arm initial posture
# def set_ur5_arm_home_pos() :
#     # jointId = p.getJointInfo()
#     pass

# set information
GRASP_INFO = {
    'top' : GraspInfo(lambda body : get_top_grasps(body, under=True, tool_pose = Pose(), max_width=INF, grasp_length=00),
                      approach_pose=Pose(0.1 * Point(z=1))),
    'gaizi' : GraspInfo(lambda body : get_gaizi_grasps(body), approach_pose=Pose(0.01 * Point(z=1))),

    'camera_slot': GraspInfo(lambda body : get_camera_slot_grasps(body), approach_pose=Pose(0.01 * Point(z=1)))
}

TOOL_FRAMES = {
    'universal_robot': 'hand_base_link', # iiwa_link_ee | iiwa_link_ee_kuka
    # **********************#**********************
    # *********************##**********************
    # ********************####*********************
    # ******************########*******************
    # ********************####*********************
    # *********************##**********star********
    # *********************#***********************

    # *********************************************
    # ***********#**********#**********************
    # **********###********###*********************
    # ***********#**********#**********************
    # *********#****************#******************
    # **********#*************#********************
    # ***********#**********#**********************
    # *************#######*************smile*******
    # *********************************************

    
    # *********************************************
    # *********/\_/\*******************************
    # ********(*o.o*)******************************
    # *********>*^*<*******************************
    # *******/*******\*****************************
    # ******(***)*(***)****************************
    # *******\*/*_*\*/*****************************
    # ********||***||*******************cat********
    # *********************************************

    # *********************************************
    # *********************************************
    # *********************************************
    # *********************************************
    # *********************************************
    # *********************************************
    # *********************************************
    # *********************************************
    # *********************************************
    # *********************************************

    # 'universal_robot': 'scoop_base_joint'
}

DEBUG_FAILURE = False
DOF_CONFIGURATIONS = 6
BASE_LINK = -1

# get function
def get_gaizi_grasps(body):
    # position , orientation = p.getBasePositionAndOrientation(body)
    # return get_grasp_pose(position, orientation)
    # (-0.5738580986190119, 0.35120943799261045, 0.12113823791316473, 0.7370214385671082, -0.5988816135101904, -0.24354942606737165, -0.19703778610752581)
    # pos = (-0.5738580986190119, 0.35120943799261045, 0.12113823791316473,)
    # orn = ( 0.7370214385671082, -0.5988816135101904, -0.24354942606737165, -0.19703778610752581)

    # pos = (-0.4226291495076493, 0.5903369442342297, 0.18498999999999996,)
    # orn = (0.6350646414078193, -0.6304129898184218, -0.31718695930416, -0.31410952922370494)

    # pos = (-3.4985669149836043, -1.5433852992315609, 1.9693869864010511,)
    # orn = (-2.890343084124677, -1.2862935892602207, -0.22077589611121534, 0.0)

    # pos = (-0.5551782991020652, 0.38254715085448715, 0.18498997693438812,)
    # orn = (0.7897430567292802, 0.420753522291077, -0.3937214767857276, 0.210370569579518)

    # pos = (-0.47912829755117337, 0.37249214232739747, 0.21322497578325308,)
    # orn = (0.8627139075717218, -0.23759700142278967, -0.4305176928640446, -0.11801226516023568)

    # pos = (-0.4557290655484278, 0.5757843784385822, 0.18498999999999996, )
    # orn = (0.891980653278984, 0.07140133524130064, -0.44493294757516666, 0.036152948203851794)

    # pos = (-0.5551782991020652, 0.38254715085448715, 0.18498997693438812,)
    # orn = (0.7897430567292802, 0.420753522291077, -0.3937214767857276, 0.210370569579518)

    # pos = (-0.40052243045695335, 0.4035837940444229, 0.15812466200788317)
    # orn = (0.8880354088732896, 0.018374171615649505, -0.4580939182007232, -0.03471980000226193)

    pos = (-0.34632724962204897, 0.40941236549677473, 0.16776093161074188)
    orn =(0.8900525136775204, 0.04808556059259193, -0.45192380269110144, -0.03548490270577528)

    # pos = (-0.5551782991020652, 0.38254715085448715, 0.18498997693438812,)
    # orn = (0.7897430567292802, 0.420753522291077, -0.3937214767857276, 0.210370569579518)

    # pos = (-0.4284738482665965, 0.46488066479370393, 0.31322330904094897)
    # orn = (0.7440502790838068, 0.4971086672519023, -0.3708813016092868, 0.24843352280831166)

    # return ((-0.5564564412720575, 0.4395771846398905, 0.2849899652895477,),
        #        ( 0.7938324943780971, -0.41298633466491713, -0.3962600743605656, -0.20554856291700122))

    # (-0.5551782991020652, 0.38254715085448715, 0.18498997693438812, 0.7897430567292802, 0.420753522291077, -0.3937214767857276, 0.210370569579518)
    return [(pos, orn)]
    # # calculate position
    # x, y, z = position
    # z += 0.1
    # position = (x, y, z)

    # # calculate orientation
    # gaizi_qua = 0,0,0,1
    # zhuazi_qua = 0.8948337891191338, 0.00035545595693414685,-0.44639876321289096,0.000712533337948919
    # R1 = Transformer.quaternion2rot(gaizi_qua)
    # R2 = Transformer.quaternion2rot(zhuazi_qua)
    # Rot = R1.inv() * R2
    # R_now = Transformer.quaternion2rot(orientation) * Rot
    # R_now = R_now.as_matrix()
    # orientation = Transformer.rot2quaternion(R_now)
    # px,py,pz,w = orientation
    # orientation = (px,py,pz,w)
    # # orientation
    
    # grasps = [(position, orientation)]
    # # print("**************ik_grasp***********************")
    # # print(grasps)
    
def get_camera_slot_grasps(body):

    pos = (-0.5551782991020652, 0.38254715085448715, 0.18498997693438812,)
    orn = (0.7897430567292802, 0.420753522291077, -0.3937214767857276, 0.210370569579518)

    # pos = (-0.47912829755117337, 0.37249214232739747, 0.21322497578325308,)
    # orn = (0.8627139075717218, -0.23759700142278967, -0.4305176928640446, -0.11801226516023568)

    # return ((-0.5564564412720575, 0.4395771846398905, 0.2849899652895477,),
        #        ( 0.7938324943780971, -0.41298633466491713, -0.3962600743605656, -0.20554856291700122))

    # (-0.5551782991020652, 0.38254715085448715, 0.18498997693438812, 0.7897430567292802, 0.420753522291077, -0.3937214767857276, 0.210370569579518)
    return [(pos, orn)]


def get_grasp_pose(position, orientation):
    # calculate position
    x, y, z = position
    z += 0.1
    position = (x, y, z)

    # calculate orientation
    gaizi_qua = 0,0,0,1
    zhuazi_qua = 0.8948337891191338, 0.00035545595693414685,-0.44639876321289096,0.000712533337948919
    R1 = Transformer.quaternion2rot(gaizi_qua)
    R2 = Transformer.quaternion2rot(zhuazi_qua)
    Rot = R1.inv() * R2
    R_now = Transformer.quaternion2rot(orientation) * Rot
    R_now = R_now.as_matrix()
    orientation = Transformer.rot2quaternion(R_now)
    px,py,pz,w = orientation
    orientation = (px,py,pz,w)
    # orientation
    
    grasps = [(position, orientation)]
    # print("**************ik_grasp***********************")
    # print(grasps)
    return grasps

def get_gripper_pose(name):
    if name == 'gaizi':
        gripper_pose = np.array( [[1.9684194,  2.08600387, 2.20679851, 2.20418981],
                            [0.1563665,  0.1653953 , 0.09571796, 0.15994406],
                            [0.08710276, 0.08024534, 0.10765833, 0.15180324],
                            [0.21472628, 0.07927919, 0,0]])
        return gripper_pose

def get_approach_pose(pose, all_bodies): 
    # approach_pose = p.calculateInverseKinematics(robot, num_joints,
    #                                     pos, orn,
    #                                     rest_pose, maxNumIterations=max_iterations)
    if pose.name() == 'gaizi_on_table' :
        # print("ik__________________________________")
        # print("return the pose", pose.name())
        # return  ((-0.585, 0.047, 0.326,), 
        #          (0.8948337891191338, 0.00035545595693414685,-0.44639876321289096,0.000712533337948919))
        
        # return ((-0.5564564412720575, 0.4395771846398905, 0.2849899652895477,),
        #        ( 0.7938324943780971, -0.41298633466491713, -0.3962600743605656, -0.20554856291700122))
    


       return ((-0.4284738482665965, 0.46488066479370393, 0.31322330904094897), 
               (0.7440502790838068, 0.4971086672519023, -0.3708813016092868, 0.24843352280831166))

        # return ((-0.4326029555066139, 0.4541442760226165, 0.18498999999999996,), 
        # [ -0.463, 0.026,0.274, 0.5727454180395177,-0.4787998017902368,-0.44347175619443363,0.49603048029147645]
        #         (0.7252213361633466, -0.5241961942552558, -0.36209921946934925,0.0))
        # return (( -0.463,0.026, 0.5727454180395177,),
        #         ( -0.4787998017902368,-0.44347175619443363,0.49603048029147645, 0.0))
    
    if pose.name() == 'gaizi_on_camera_slot':
        # return  ((-0.585, 0.047, 0.326,), (0.8948337891191338, 0.00035545595693414685,-0.44639876321289096,0.000712533337948919))
        # pos, orn = p.getBasePositionAndOrientation(all_bodies[0])
        # grasps = get_grasp_pose(pos, orn)[0]
        # return grasps
        return  ((-0.585, 0.047, 0.326,), 
                 (0.8948337891191338, 0.00035545595693414685,-0.44639876321289096,0.000712533337948919))

def get_joint_pose(body, link_id = 6) :
    # current_pose = ()
    # joints = p.getNumJoints(robot)
    # current_pose += tuple(p.getJointState(robot, joint_id)[0] for joint_id in range(joints))
    link_info = p.getLinkState(body, link_id)
    link_pose = (link_info[0], link_info[1])
    return link_pose

def get_joint_positions_dof(robot) :
    current_pose = ()
    joints = p.getNumJoints(robot)
    current_pose += tuple(p.getJointState(robot, joint_id)[0] for joint_id in range(joints))
    return current_pose[:DOF_CONFIGURATIONS]

def get_movable_joints(body):
    joint_num = p.getNumJoints(body)
    joints = list(x for x in range(joint_num) \
                        if p.getJointInfo(body, x)[2] != p.JOINT_FIXED)[: DOF_CONFIGURATIONS]
    return joints

def get_configurations(body):
    # joints = get_movable_joints(body)
    # return tuple(get_joint_positions(body, joint) for joint in joints) 
    return get_joint_positions(body, get_movable_joints(body))

# def get_end_link(robot):
#  the end_link_name='ee_link'
#     end_link_name = p.getJointInfo(robot, 6)[12].decode('UTF-8')
#     # end_link = (p.)
#     for id in range(p.getNumJoints(robot)):
#         p.getJointInfo(robot, 6)
#     return end_link

# stream function
# def get_tool_link(robot):
#     joint_num = p.getNumJoints(robot)
#     # return list(x for x in range(joint_num) if x == 7)
#     print("*******************get_tool_link****************************")
#     name = list(p.getJointInfo(robot, x)[1].decode('utf-8') for x in range(joint_num) if x == 7)
#     print("name", name)
#     return link_from_name(robot, TOOL_FRAMES[get_body_name(robot)])
def get_tool_link(robot):
    return 6

def get_end_eff_link(robot):
    return 7

# def get_stable_gen(fixed=[]):
#     def gen(body, surface):
#         while True:
#             pose = sample_placement(body, surface)
#             if (pose is None) or any(pairwise_collision(body, b) for b in fixed):
#                 continue
#             body_pose = BodyPose(body, pose)
#             yield (body_pose,)
#     return gen

def get_stable_gen(fixed=[], body_names=[]):
    def gen(body, surface):
        # while True:
            # give a fixed coordinate
            #  [ -0.585, 0.047, 0.326, 0.8948337891191338, 0.00035545595693414685,-0.44639876321289096,0.000712533337948919]
            # pose = ((-2.6,2.4,1.1),(0.5, -0.5, -0.5, 0.5))
            # pose = ((-0.585, 0.047, 0.326,), (0.8948337891191338, 0.00035545595693414685,-0.44639876321289096,0.000712533337948919))
            # body_pose = BodyPose(body, pose)
            # print("************get_sample_poses**************************")
            # name1 = body_names[body]
            # name2 = body_names[surface]
            # print("name1:", name1, "name2", name2,body_pose.pose)
            # yield(body_pose,)
            # name = body_names
            # if (name[surface] == 'camera_slot' and name[body] == 'gaizi'):
            #     pose = ((-2.6,2.4,1.1),(0.5, -0.5, -0.5, 0.5))
            #     body_pose = BodyPose(body, pose)
            #     print("************get_poses**************************")
            #     print(body_pose.pose)
            #     yield (body_pose,)
            # if (name[surface] == 'table' and name[body] == 'table') :
            #     pose =
        name = body_names
        pose = ((0, 0, 0), (0, 0, 0, 0))
        if (name[body] == 'gaizi' and name[surface] == 'camera_slot'):
            body_pose = BodyPose(body, pose)
            body_pose.name_ = 'gaizi_on_camera_slot'
            yield(body_pose,) 
    return gen


# def get_sample_fn():

#     def fn():   
#         return tuple(next(generator))
#     return fn
# def get_grasp_gen(robot, grasp_name='top'):
#     grasp_info = GRASP_INFO[grasp_name]
#     tool_link = get_tool_link(robot)
#     def gen(body):
#         grasp_poses = grasp_info.get_grasps(body)
#         print("********************************GRASP_INFO**************")
#         print("grasp_pose:", grasp_info.get_grasps(body))
#         print("approach_pose", grasp_info.approach_pose)
#         # TODO: continuous set of grasps
#         for grasp_pose in grasp_poses:
#             body_grasp = BodyGrasp(body, grasp_pose, grasp_info.approach_pose, robot, tool_link)
#             print("********************************get_grasp_gen**************")
#             print("grasp_pose:", grasp_pose)
#             print("approach_pose", grasp_info.approach_pose)
#             print("body_grasp", body_grasp)
#             print("body_grasp_value", body_grasp.value)
#             yield (body_grasp,)
#     return gen

# def get_body_name(all, body):
#     if body in all
#     return name


def get_grasp_gen(robot, all_bodies):
    # grasp_info = GRASP_INFO[grasp_name]
    tool_link = get_tool_link(robot)
    def gen(body):
        # print("********************************GRASP_INFO**************")
        # print("grasp_pose:", grasp_info.get_grasps(body), type(grasp_info.get_grasps(body)))
        grasp_info = GRASP_INFO[all_bodies[body]]
        grasp_poses = grasp_info.get_grasps(body)
        for grasp_pose in grasp_poses:
            # print("********************************GRASP_INFO**************")
            # print("grasp_pose:", grasp_pose, type(grasp_pose))
            body_grasp = BodyGrasp(body, grasp_pose, grasp_info.approach_pose, robot, tool_link)
        # print("*********************ik_get_movable_joints*******************")
        # print(body_grasp, body_grasp.value(), type(body_grasp.value()))
        yield(body_grasp,)
    return gen

def get_ik_fn(robot, fixed=[], all_bodies = [], teleport=False, num_attempts=10):
    # movable_joints = get_movable_joints(robot)
    # print("*********************ik_get_movable_joints*******************")
    # print(movable_joints, type(movable_joints))
    # get the limits of joints
    # sample_fn = get_sample_fn(robot, movable_joints)
    def fn(body, pose, grasp):
        # obstacles = [body] + fixed
        for _ in range(num_attempts):
            approach_pose = get_approach_pose(pose, all_bodies)
            grasp_pose = grasp.value()
            q_approach = inverse_kinematics(robot, approach_pose)
            print("q_qpproach ik is over", q_approach)
            if (q_approach is None) :
                continue
            conf = BodyConf(robot, q_approach)
            # print("*********************get_ik_conf*******************")
            # print("grasp_posse",grasp_pose)

            q_grasp = inverse_kinematics(robot, grasp_pose)
            print("q_grasp is over", q_grasp)
            if (q_grasp is None) :
                continue

            gripper_pose = end_effector_from_body(pose.pose, grasp_pose)
            print("grriper_pose", gripper_pose)
            q_gripper = inverse_kinematics(robot, gripper_pose)
            print("q_gripper is over", q_gripper)

            # gripper_info = GripperCtrl(robot, body)
            # gripper_links = gripper_info.gripper_joints
            # gripper_path = gripper_info.paths()

            if teleport:
                path = [q_approach, q_grasp]
            else:
                # go to the approach_posture
                conf.assign()
                # print("end_eff_pos::::::::::::::::::::::::::::::::::::::::::::::::;;")
                end_link_state = p.getLinkState(robot, 6)[0:2]
                # print(end_link_state)
                # plan the path from approach_pos to grasp_pos
                # print("*********************get_q_approach*******************")
                # print("q_approach:", q_approach)
                # print("q_grasp", q_grasp[:7])
                path = plan_direct_joint_motion(robot, conf.joints, q_grasp[:DOF_CONFIGURATIONS])
                # print("path", path)  
                if path is None:
                    if DEBUG_FAILURE: wait_if_gui('Approach motion failed')
                    continue
            # gripper_pose = get_gripper_pose(name = grasp.name)
            # pos , orn = grasp.value()
            # approach_pose = get_approach_pose(robot, num_joints=7, 
            #                                pos = pos, orn =orn,
            #                                rest_pose = rest_pose, max_iterations = 5)
            # print("**********************ik_approach_pose****************")
            # print(approach_pose)
            # if (approach_pose is None) or any(pairwise_collision(robot, b) for b in obstacles):
            #     continue
            # conf = BodyConf(robot, approach_pose)
            # print("**********************ik_conf****************")
            # print(conf)
            # if teleport:
            #     path = [approach_pose, gripper_pose]
            # else:
            #     conf.assign()
                # move_gripper(gripper_pose)
                # grriper = GripperCtrl(robot)
            command = Command([BodyPath(robot, path),
                            #    GripperCtrl(robot),
                               Attach(robot, body, end_link_state),
                               BodyPath(robot, path[::-1])])
                            #    BodyPath(robot, path[::-1])])
            print("*************IK_IS_OVER*************")
            print("conf", conf, conf.configuration)
            print("comman_path", command.body_paths)
            print("path", path)  
            return (conf, command)
        # gripper_pose = end_effector_from_body(pose.pose, grasp.grasp_pose)
        # approach_pose = approach_from_grasp(grasp.approach_pose, gripper_pose)
        # for _ in range(num_attempts):
        #     set_joint_positions(robot, movable_joints, sample_fn()) # Random seed
        #     # TODO: multiple attempts?
        #     q_approach = inverse_kinematics(robot, grasp.link, approach_pose)
        #     if (q_approach is None) or any(pairwise_collision(robot, b) for b in obstacles):
        #         continue
        #     conf = BodyConf(robot, q_approach)
        #     q_grasp = inverse_kinematics(robot, grasp.link, gripper_pose)
        #     if (q_grasp is None) or any(pairwise_collision(robot, b) for b in obstacles):
        #         continue
        #     if teleport:
        #         path = [q_approach, q_grasp]
        #     else:
        #         conf.assign()
        #         path = plan_direct_joint_motion(robot, conf.joints, q_grasp, obstacles=obstacles)
        #         if path is None:
        #             if DEBUG_FAILURE: wait_if_gui('Approach motion failed')
        #             continue
        #     print("*********************ik_fun*********************************")
        #     print(path)
        #     print(type(path))
        #     command = Command([BodyPath(robot, path),
        #                        Attach(body, robot, grasp.link),
        #                        BodyPath(robot, path[::-1], attachments=[grasp])])
        #     return (conf, command)
            # TODO: holding collisions
        return None
    return fn

def get_free_motion_gen(robot, fixed=[], teleport=False, self_collisions=True):
    def fn(conf1, conf2, fluents=[]):
        assert((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        if teleport:
            path = [conf1.configuration, conf2.configuration]
        else :
            conf1.assign()
            print("*****************free_motion_begin******************")
            # print("path", path)
            # path = plan_joint_motion(robot, conf2.joints, conf2.configuration)
            path = plan_direct_joint_motion(robot, conf2.joints, conf2.configuration)
            # print("*****************free_motion******************")
            # print("path", path)
            if path is None:
                if DEBUG_FAILURE: wait_if_gui('Free motion failed')
                return None
        command = Command([BodyPath(robot, path, joints=conf2.joints)])
        print("comman_path", command.body_paths)
        print("path", path)  
        return (command,)
    return fn

def get_holding_motion_gen(robot, fixed=[], teleport=False, self_collisions=True):
    def fn(conf1, conf2, body, grasp, fluents=[]):
        assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        if teleport:
            path = [conf1.configuration, conf2.configuration]
        else:
            conf1.assign()
            # path = plan_joint_motion(robot, conf2.joints, conf2.configuration)
            path = plan_direct_joint_motion(robot, conf2.joints, conf2.configuration)
            if path is None:
                if DEBUG_FAILURE: wait_if_gui('Holding motion failed')
                return None
            print("*****************holding_motion******************")
            # print("path", path)
        command = Command([BodyPath(robot, path, joints=conf2.joints)])
        print("comman_path", command.body_paths)
        print("path", path)  
        return (command,)
    return fn


def get_movable_collision_test():
    def test(command, body, pose):
        # if body in command.bodies():
        #     return False
        # pose.assign()
        # for path in command.body_paths:
        #     moving = path.bodies()
        #     if body in moving:
        #         # TODO: cannot collide with itself
        #         continue
        #     for _ in path.iterator():
        #         # TODO: could shuffle this
        #         if any(pairwise_collision(mov, body) for mov in moving):
        #             if DEBUG_FAILURE: wait_if_gui('Movable collision')
        #             return True
        return False
    return test

# def postprocess_plan(plan):
#     paths = []
#     for name, args in plan:
#         if name == 'place':
#             paths += args[-1].reverse().body_paths
#         elif name in ['move', 'move_free', 'move_holding', 'pick']:
#             paths += args[-1].body_paths
#             print("****************args_body_paths*********************")
#             print(args[-1], args[-1].body_paths)
#             print(type(args))
#     print("*************************************")
#     print(paths)
#     print(type(paths))
#     return Command(paths)
    # if plan is None:
    #     return None
    # commands = []
    # for name, args in plan:
    #         # if name == 'move_free':
    #         #     com = args[-1]
    #         # if name == 'pick':
    #         #     com = args[]
                

    #  return True



class ApplyForce(object):
    def __init__(self, body, robot):
        self.body = body
        self.robot = robot
    def bodies(self):
        return {self.body, self.robot}
    def iterator(self, **kwargs):
        return []
    def refine(self, **kwargs):
        return self
    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__, self.robot, self.body)

# class Attachment(object):
#     def __init__(self, parent, grasp_pose, child, parent_link=6 ):
#         self.parent = parent # TODO: support no parent
#         self.parent_link = parent_link
#         self.grasp_pose = grasp_pose
#         self.child = child
#         #self.child_link = child_link # child_link=BASE_LINK
#     @property
#     def bodies(self):
#         # return flatten_links(self.child) | flatten_links(self.parent, get_link_subtree(
#         #     self.parent, self.parent_link))
#         return self.body
#     def assign(self):
#         parent_link_pose = get_link_pose(self.parent, self.parent_link)
#         child_pose = body_from_end_effector(parent_link_pose, self.grasp_pose)
#         # print("********************ATTACHMENT_INFO*********************************")
#         # print("attachment_joint_id, joint_pose", self.child, child_pose)
#         set_pose(self.child, child_pose)
#         return child_pose
#     def apply_mapping(self, mapping):
#         self.parent = mapping.get(self.parent, self.parent)
#         self.child = mapping.get(self.child, self.child)
#     def __repr__(self):
#         return '{}({},{})'.format(self.__class__.__name__, self.parent, self.child)


# class Attachment(object):
#     def __init__(self, robot, body=None, link=[], grasp_pose=[]):
#         self.parent = robot
#         self.body = body
#         self.parent_link = link
#         self.grasp_pose = grasp_pose
#         self.child = body

#     def bodieds(self):
#         return self.body
    
#     def assign(self):
#         parent_link_pose = get_link_pose(self.parent, self.parent_link)
#         child_pose = body_from_end_effector(parent_link_pose, self.grasp_pose)
#         set_pose(self.child, child_pose)
#         return child_pose
    
#     def __repr__(self):
#         return '{}({},{})'.format(self.__class__.__name__, self.parent, self.child)



# class Attach(ApplyForce):
#     def control(self, **kwargs):
#         # TODO: store the constraint_id?
#         add_fixed_constraint(self.body, self.robot)
#     def reverse(self):
#         return Detach(self.body, self.robot)


# class Detach(ApplyForce):
#     def control(self, **kwargs):
#         remove_fixed_constraint(self.body, self.robot)
#     def reverse(self):
#         return Attach(self.body, self.robot)


class RobotCtrl:

    def __init__(self, robot) -> None:
        # initial information
        self.robot = robot
        self.position_home = [-3.14, -1.347, 1.432, -1.853, -1.558, 0.000, 0.0]
        self.acc_DOF = 6

        # set up collision between 2 links
        p.setCollisionFilterPair(robot, robot , linkIndexA=7, linkIndexB=8, enableCollision=True)
        p.setCollisionFilterPair(robot, robot , linkIndexA=7, linkIndexB=15, enableCollision=True)
        p.setCollisionFilterPair(robot, robot , linkIndexA=7, linkIndexB=22, enableCollision=True)
        p.setCollisionFilterPair(robot, robot , linkIndexA=7, linkIndexB=29, enableCollision=True)
        p.setCollisionFilterPair(robot, robot , linkIndexA=7, linkIndexB=36, enableCollision=True)

        # initialize the joints states
        joint_num = p.getNumJoints(robot)
        
        controllable_joint_id = list(x for x in range(joint_num) \
                                 if p.getJointInfo(robot, x)[2] != p.JOINT_FIXED)[: self.acc_DOF]
        
        # judge the number of controllable joints
        assert len(controllable_joint_id) > 0

        # set controllable joint initial properties
        for pos_index, joint_id in enumerate(controllable_joint_id):
            # set dynamics
            p.changeDynamics(robot, joint_id, linearDamping=0, angularDamping=0)
            p.resetJointState(robot, joint_id, self.position_home[pos_index])
        # self.init_gripper()
        # p.setJointMotorControl2(robot,  9, p.POSITION_CONTROL, 80*1.2, force=5 * 240.)
        # p.resetJointState(robot, 9, 80*1.2)
        # p.stepSimulation()

    # def init_gripper(self):
    #     robot = self.robot
    #     ctl_array = np.zeros((4, 4))
    #     # ctl_array = np.array([
    #     #               [1.9684194,  2.08600387, 2.20679851, 2.20418981],
    #     #               [0.1563665,  0.1653953,  0.09571796, 0.15994406],
    #     #               [0.08710276, 0.08024534, 0.10765833, 0.15180324],
    #     #               [0.21472628, 0.07927919, 0.,         0.        ]
    #     #               ])
    #     # ctl_array = np.array([
    #     #         [0.50737378,0.5034353 ,0.54908505,0.38536392],
    #     #         [1.40010227,1.40297468,1.44520645,1.41979466],
    #     #         [0.40067853,0.40093792,0.40044203,0.620571  ],
    #     #         [0.27839908,0.25129627,0.        ,0.        ]
    #     #     ])
    #     move_gripper(robot, ctl_array)
    #     p.stepSimulation()
        # pass
    pass

class Transformer:
    def euler2rot(euler):   #欧拉角=>转旋矩阵
        r = R.from_euler('xyz', euler, degrees=True)
        
        return r

    def rot2euler(rot):    #转旋矩阵=>欧拉角
        rot = rot[:3, :3]  # 提取旋转矩阵的3x3部分
        r = R.from_matrix(rot)  # 使用scipy的R模块将旋转矩阵转换为旋转对象

        # 获取欧拉角
        euler_angles = r.as_euler('xyz', degrees=True)  # 使用'xyz'顺序，返回角度制的欧拉角

        return euler_angles

    def quaternion2rot(quat):    #四元数转旋转矩阵
        rot  = R.from_quat(quat)#.as_matrix()
        return rot

    def rot2quaternion(rot):   #旋转矩阵转四元数
            rot = rot[:3,:3]
            quat = R.from_matrix(rot).as_quat()
            return quat
    pass

class BodyConf(object):

    num = count()

    def __init__(self, body, configuration=None, joints=None):
        # get controllable joints configgurations
        if joints is None:
            joints = get_movable_joints(body)
        if configuration is None:
            configuration = get_joint_positions(body, joints)
        
        # change the type of parameters
        self.body = body
        self.joints = joints
        self.configuration = configuration[:DOF_CONFIGURATIONS]
        self.index = next(self.num)
        pass

    # get the joint's current posture 
    def values(self):
        return self.configuration
    
    # set the robot postures
    def assign(self):
        for i, joint_id in enumerate(self.joints):
            p.resetJointState(self.body, joint_id, targetValue=self.configuration[i],
                            targetVelocity = 0)
            
        # set_joint_positions(self.body, self.joints, self.configuration)
        return self.configuration
    
    # define the ouput of robot posture
    def __repr__(self) -> str:
        index = self.index
        return 'q{}, {}'.format(index, self.configuration)
    pass

class BodyPose(object):
    num = count()

    def __init__(self, body, pose=None, name = None) -> None:
        if pose is None:
            pose = get_pose(body)
        self.body = body
        self.pose = pose
        self.name_ = name
        self.index = next(self.num)
        pass
    
    def iterate(self):
        yield self 

    # get object position
    def value(self):
        return self.pose
    
    # set object position
    def assign(self):
        set_pose(self.body, self.pose)
        # return self.pose
    
    # set grasp and place pose
    def name(self):
        return self.name_
    
    def __repr__(self) -> str:
        index = self.index
        return 'p{}'.format(index)

# def grriper_controller(body, joints, target, tolerance=1e-3, timeout=INF, **kwargs):
#     assert(len(joints) == len(target))
#     dt = get_time_step()
#     time_elapsed = 0.
#     control_joints(body, joints, target, **kwargs)
#     positions = get_joint_positions(body, joints)
#     while not all_close(positions, target, atol=tolerance) and (time_elapsed < timeout):
#         yield positions
#         time_elapsed += dt
#         positions = get_joint_positions(body, joints)

class GripperCtrl(object):
    def __init__(self, robot, path = None, joints=None) -> None:
        if joints is None:
            joints = np.array([    [9, 16, 23, 20],
                                   [10, 17, 24, 31],
                                   [11, 18, 25, 32],
                                   [39, 38, 36, 37]])
        self.robot = robot
        self.gripper_joints = joints.flatten()
        self.start_conf = np.zeros((4, 4)).flatten()
        self.end_conf = np.array([
                      [1.9684194,  2.08600387, 2.20679851, 2.20418981],
                      [0.1563665,  0.1653953,  0.09571796, 0.15994406],
                      [0.08710276, 0.08024534, 0.10765833, 0.15180324],
                      [0.21472628, 0.07927919, 0.,         0.        ]
                      ]).flatten()
        self.path = [self.start_conf, self.end_conf] if path is None else path
        pass
    # def __init__(self, robot, array=[]):
    #     self.robot = robot
    #     self.gripper_joints = np.array([
    #                                [9, 16, 23, 20],
    #                                [10, 17, 24, 31],
    #                                [11, 18, 25, 32],
    #                                [39, 38, 36, 37],
    #                                [8, 15, 22, 29]])
    #     self.start_conf = np.zeros((4, 4))
    #     self.end_conf = np.array([
    #                   [1.9684194,  2.08600387, 2.20679851, 2.20418981],
    #                   [0.1563665,  0.1653953,  0.09571796, 0.15994406],
    #                   [0.08710276, 0.08024534, 0.10765833, 0.15180324],
    #                   [0.21472628, 0.07927919, 0.,         0.        ]
    #                   ])
    #     self.path = [self.start_conf, self.end_conf]
    #     self.gripper_array = self.start_conf if array == [] else array


    # def paths(self):
    #     start_conf = np.zeros((4, 4))
    #     end_conf = np.array([
    #                   [1.9684194,  2.08600387, 2.20679851, 2.20418981],
    #                   [0.1563665,  0.1653953,  0.09571796, 0.15994406],
    #                   [0.08710276, 0.08024534, 0.10765833, 0.15180324],
    #                   [0.21472628, 0.07927919, 0.,         0.        ]
    #                   ])
    #     path = [start_conf, end_conf]
    #     return path
    
    # def bodies(self):
    #     return set(flatten(path.bodies() for path in self.body_paths))

    def iterator(self, dt=0):
        # TODO: compute and cache these
        # TODO: compute bounding boxes as well
        #  对于每个手臂的Pose,进行一次抓取的遍历， 或者说，将机械臂设置固定到一个状态，执行抓取， 怎么判断抓取成功?
        #  应该是没有进行是否抓取完成的判断，也是写死，一个最简单的demo。
        #  还有理解，这里的grasp.assign是为了在每一次仿真更新的时候保持对物体的抓取状态，抓取是在attach过程中完成的。
        # enable_real_time()
        for i, configuration in enumerate(self.path):
            configuration = configuration.reshape((4, 4))
            move_gripper(self.robot, configuration)
            enable_gravity()
            step_simulation()
            # print("*******ppe_conf******************")
            # print(configuration)
            # disable_real_time()
        yield i
        

    def control(self, real_time=False, dt=0):
        # TODO: just waypoints
        if real_time:
            enable_real_time()
        else:
            disable_real_time()
        for values in self.path:
            for _ in joint_controller(self.robot, self.gripper_joints, values):
                enable_gravity()
                if not real_time:
                    step_simulation()
                time.sleep(dt)

    def joints(self):
        return self.gripper_joints
    
    def arrays(self):
        return self.path
    
    def refine(self, num_steps=0):
        refined_path = get_refined_paths(self.path)
        return self.__class__(self.robot,refined_path, self.gripper_joints)
    def reverse(self):
        return self.__class__(self.robot, self.path[::-1], self.gripper_joints)
    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__, len(self.gripper_joints), len(self.path))

    # def assign(self) :
    #     for position in self.path:
    #         set_gripper_positions(self.robot, self.gripper_joints, position)
    #         yield position

    # def reverse(self):
    #     return GripperCtrl_reverse(self.robot)

    # def __repr__(self):
    #     return '{}({})'.format(self.__class__.__name__, get_body_name(self.robot))

# a simple refine path function
def get_refined_paths(path, num_steps=3):
    # def refine_fn(v1, v2):
    #     return [v1 + (v2 - v1) * i / (num_steps + i) for i in range(num_steps + 2)]
    
    # def get_pairs(waypoints):
    #     return zip(waypoints[:-1], waypoints[1:])

    # refined_path = []
    # for v1, v2 in get_pairs(path):
    #     refined_path.extend(refine_fn(v1, v2))

    return path

class Attach():
    def __init__(self, robot, body, grasp) :
        self.robot = robot
        self.body = body
        self.grasp = grasp
        
    def iterator(self):
        max_force = 500
        pos, orn = self.grasp
        print(":::::::::::::::::::::::body_link:::::::::::::::::::::::::::;;")
        num_links = p.getNumJoints(self.body)
        print(self.body)
        print(f"Number of links: {num_links}")
        for link_index in range(num_links):
            link_info = p.getJointInfo(self.body, link_index)
            link_name = link_info[1].decode('utf-8')  # 获取链接名称，解码为字符串
            print(f"Link {link_index}: {link_name}")
        num_links = p.getNumJoints(self.robot)
        print(f"Number of links: {num_links}")
        for link_index in range(num_links):
            link_info = p.getJointInfo(self.robot, link_index)
            link_name = link_info[1].decode('utf-8')  # 获取链接名称，解码为字符串
            print(f"Link {link_index}: {link_name}")
        constraint = p.createConstraint(self.robot, 6, self.body, 0,  # Both seem to work
                                    p.JOINT_FIXED, 
                                    jointAxis=unit_point(),
                                    parentFramePosition=pos,
                                    childFramePosition=unit_point(),
                                    parentFrameOrientation=orn,
                                    childFrameOrientation=(0, 0, 0, 1),
                                    physicsClientId=0)
        # constraint= p.createConstraint(self.robot, 6, self.body, 0,
        #                            p.JOINT_FIXED, jointAxis=[0, 0, 0],
        #                            parentFramePosition=[0, 0, 0],
        #                            childFramePosition=[0, 0, 0])
       
        p.stepSimulation()
        if max_force is not None:
            p.changeConstraint(constraint, maxForce=max_force, physicsClientId=0)
        return []
    def reverse(self):
        return Detach(self.robot, self.body)
    def refine(self, **kwargs):
        return self

class Detach():
    def __init__(self, robot, body) :
        self.robot = robot
        self.body = body
        
    def iterator(self):
        max_force = 500
        constraint = p.createConstraint(self.robot, 6, self.body, -1,  
                                    p.JOINT_FIXED, 
                                    jointAxis=unit_point(),
                                    parentFramePosition=(-3.14, -1.347, 1.432),
                                    childFramePosition=unit_point(),
                                    parentFrameOrientation=(-1.853, -1.558, 0.0),
                                    childFrameOrientation=(0, 0, 0, 1),
                                    physicsClientId=0)
        if max_force is not None:
            p.changeConstraint(constraint, maxForce=max_force, physicsClientId=0)
        return []
    def reverse(self):
        return Attach(self.robot, self.body)
    def refine(self, **kwargs):
        return self

# class BodyPath(object):
#     def __init__(self, body, path, joints=None, attachments=[]) -> None:
#         if joints is None:
#             joints = get_movable_joints(body)
#         self.body = body
#         self.path = path
#         self.joints = joints
#         self.attachments = attachments
#         pass
    
#     def bodies(self):
#         # for second, attachments = [grasp], grasp.body
#         return set([self.body] + [attachment.body for attachment in self.attachments])
    
#     def iterator(self):
#         # TODO: compute and cache these
#         # TODO: compute bounding boxes as well
#         #  对于每个手臂的Pose,进行一次抓取的遍历， 或者说，将机械臂设置固定到一个状态，执行抓取， 怎么判断抓取成功?
#         #  应该是没有进行是否抓取完成的判断，也是写死，一个最简单的demo。
#         #  还有理解，这里的grasp.assign是为了在每一次仿真更新的时候保持对物体的抓取状态，抓取是在attach过程中完成的。
#         for i, configuration in enumerate(self.path):
#             set_joint_positions(self.body, self.joints, configuration)
#             for grasp in self.attachments:
#                 print("********************attachments**************************88")
#                 grasp.assign()
#                 print(i)
#             yield i

#     def control(self, real_time=False, dt=0):
#         # TODO: just waypoints
#         if real_time:
#             enable_real_time()
#         else:
#             disable_real_time()
#         for values in self.path:
#             for _ in joint_controller(self.body, self.joints, values):
#                 enable_gravity()
#                 if not real_time:
#                     step_simulation()
#                 time.sleep(dt)
    
#     def refine(self, num_steps=0):
#         return self.__class__(self.body, refine_path(self.body, self.joints, self.path, num_steps), self.joints, self.attachments)
#     def reverse(self):
#         return self.__class__(self.body, self.path[::-1], self.joints, self.attachments)
#     def __repr__(self):
#         return '{}({},{},{},{})'.format(self.__class__.__name__, self.body, len(self.joints), len(self.path), len(self.attachments))
#     pass

class BodyPath(object):
    def __init__(self, body, path, joints=None) -> None:
        if joints is None:
            joints = get_movable_joints(body)
        self.body = body
        self.path = path
        self.joints = joints
        pass
    
    def bodies(self):
        # for second, attachments = [grasp], grasp.body
        return set([self.body])
    
    def iterator(self):
        # TODO: compute and cache these
        # TODO: compute bounding boxes as well
        #  对于每个手臂的Pose,进行一次抓取的遍历， 或者说，将机械臂设置固定到一个状态，执行抓取， 怎么判断抓取成功?
        #  应该是没有进行是否抓取完成的判断，也是写死，一个最简单的demo。
        #  还有理解，这里的grasp.assign是为了在每一次仿真更新的时候保持对物体的抓取状态，抓取是在attach过程中完成的。
        for i, configuration in enumerate(self.path):
            set_joint_positions(self.body, self.joints, configuration)
            # for gripper in self.grippers:
            #     gripper.assign()
            yield i

    def control(self, real_time=True, dt=0):
        # TODO: just waypoints
        if real_time:
            enable_real_time()
        else:
            disable_real_time()
        for values in self.path:
            for _ in joint_controller(self.body, self.joints, values):
                enable_gravity()
                if not real_time:
                    step_simulation()
                time.sleep(dt)
    
    def refine(self, num_steps=0):
        return self.__class__(self.body, refine_path(self.body, self.joints, self.path, num_steps), self.joints)
    def reverse(self):
        return self.__class__(self.body, self.path[::-1], self.joints)
    def __repr__(self):
        return '{}({},{},{})'.format(self.__class__.__name__, self.body, len(self.joints), len(self.path))
    pass

class Command(object):
    num = count()
    def __init__(self, body_paths):
        self.body_paths = body_paths
        self.index = next(self.num)
    def bodies(self):
        return set(flatten(path.bodies() for path in self.body_paths))
    # def full_path(self, q0=None):
    #     if q0 is None:
    #         q0 = Conf(self.tree)
    #     new_path = [q0]
    #     for partial_path in self.body_paths:
    #         new_path += partial_path.full_path(new_path[-1])[1:]
    #     return new_path
    def step(self):
        for i, body_path in enumerate(self.body_paths):
            for j in body_path.iterator():
                msg = '{},{}) step?'.format(i, j)
                wait_if_gui(msg)
                #print(msg)
    def execute(self, time_step=0.05):
        for i, body_path in enumerate(self.body_paths):
            for j in body_path.iterator():
                #time.sleep(time_step)
                wait_for_duration(time_step)
    def control(self, real_time=False, dt=0): # TODO: real_time
        for body_path in self.body_paths:
            body_path.control(real_time=real_time, dt=dt)
    def refine(self, **kwargs):
        return self.__class__([body_path.refine(**kwargs) for body_path in self.body_paths])
    def reverse(self):
        return self.__class__([body_path.reverse() for body_path in reversed(self.body_paths)])
    def __repr__(self):
        index = self.index
        #index = id(self) % 1000
        return 'c{}'.format(index)


    
    

# class GripperCtrl_reverse():
#     def __init__(self, robot, array=[]):
#         self.robot = robot
#         self.gripper_array = array
#         self.gripper_joints = np.array([[9, 16, 23, 20],
#                                    [10, 17, 24, 31],
#                                    [11, 18, 25, 32],
#                                    [39, 38, 36, 37],
#                                    [8, 15, 22, 29]])
#         self.end_conf = np.zeros((4, 4))
#         self.start_conf = np.array([
#                       [1.9684194,  2.08600387, 2.20679851, 2.20418981],
#                       [0.1563665,  0.1653953,  0.09571796, 0.15994406],
#                       [0.08710276, 0.08024534, 0.10765833, 0.15180324],
#                       [0.21472628, 0.07927919, 0.,         0.        ]
#                       ])
#         self.path = [self.start_conf, self.end_conf]
#     # def paths(self):
#     #     end_conf = np.zeros((4, 4))
#     #     start_conf = np.array([
#     #                   [1.9684194,  2.08600387, 2.20679851, 2.20418981],
#     #                   [0.1563665,  0.1653953,  0.09571796, 0.15994406],
#     #                   [0.08710276, 0.08024534, 0.10765833, 0.15180324],
#     #                   [0.21472628, 0.07927919, 0.,         0.        ]
#     #                   ])
#     #     path = [start_conf, end_conf]
#     #     return path
    
#     # def bodies(self):
#     #     return set(flatten(path.bodies() for path in self.body_paths))
#     def joints(self):
#         return self.gripper_joints
    
#     def arrays(self):
#         return self.gripper_array
    
#     def assign(self) :
#         for position in self.path:
#             set_gripper_positions(self.robot, self.gripper_joints, position)
#             yield position

#     def reverse(self):
#         return GripperCtrl(self.robot)

#     def __repr__(self):
#         return '{}({})'.format(self.__class__.__name__, get_body_name(self.robot))
#     pass
    # def __init__(self, robot, arm, position, teleport=False):
    #     self.robot = robot
    #     self.arm = arm
    #     self.position = position
    #     self.teleport = teleport
    # def apply(self, state, **kwargs):
    #     joints = get_gripper_joints(self.robot, self.arm)
    #     start_conf = get_joint_positions(self.robot, joints)
    #     # end_conf = [self.position] * len(joints)
    #     end_conf = ([self.position] if isinstance(self.position, float) else list(self.position)) * len(joints) 
    #     if self.teleport:
    #         path = [start_conf, end_conf]
    #     else:
    #         extend_fn = get_extend_fn(self.robot, joints)
    #         path = [start_conf] + list(extend_fn(start_conf, end_conf))
    #         # path = [start_conf] + list(extend_fn(start_ex_conf, end_conf))
        
    #     # print("*****************paths*******************************")
    #     # print("path:", path, "path_type", type(path))
            
    #     for positions in path:
    #         set_joint_positions(self.robot, joints,  positions)
    #         yield positions
    # def control(self, **kwargs):
    #     joints = get_gripper_joints(self.robot, self.arm)
    #     positions = [self.position]*len(joints)
    #     for _ in joint_controller_hold(self.robot, joints, positions):
    #         yield

    # def __repr__(self):
    #     return '{}({},{},{})'.format(self.__class__.__name__, get_body_name(self.robot),
    #                                  self.arm, self.position)


class BodyGrasp(object):
    num = count()

    def __init__(self, body, grasp_pose, approach_pose, robot, link, gripper_array = [], name='') -> None:
        self.body = body
        self.grasp_pose = grasp_pose
        self.approach_pose = approach_pose
        self.robot = robot
        self.link = 7
        self.grasp_name = name
        self.index = next(self.num)
        self.array = np.zeros((4, 4))
        pass

    # return grasp posture
    def value(self):
        return self.grasp_pose
    
    def name(self):
        return self.grasp_name

    def approach(self):
        return self.approach_pose
    
    # def attachment(self):
    #     return Attachment(self.robot, self.link, self.grasp_pose, self.body)
    # def assign(self):
    #     return self.attachment().assign()
    
    # def attachment(self):
    #     return GripperCtrl()
    
    # def assign(self):
    #     return self.attachment().assign()
    #     # return move_gripper(self.robot, self.array)
    
    def __repr__(self) -> str:
        index = self.index
        return 'g{}'.format(index)
    pass


    # num = count()
    # def __init__(self, body_paths):
    #     self.body_paths = body_paths
    #     self.index = next(self.num)
    # def bodies(self):
    #     return set(flatten(path.bodies() for path in self.body_paths))
    # def step(self):
    #     for i, body_path in enumerate(self.body_paths):
    #         for j in body_path.iterator():
    #             msg = '{},{}) step?'.format(i, j)
    #             wait_if_gui(msg)
    #             #print(msg)
    # def execute(self, time_step=0.05):
    #     for i, body_path in enumerate(self.body_paths):
    #         for j in body_path.iterator():
    #             #time.sleep(time_step)
    #             wait_for_duration(time_step)
    # def control(self, real_time=False, dt=0): # TODO: real_time
    #     for body_path in self.body_paths:
    #         body_path.control(real_time=real_time, dt=dt)
    # def refine(self, **kwargs):
    #     return self.__class__([body_path.refine(**kwargs) for body_path in self.body_paths])
    # def reverse(self):
    #     return self.__class__([body_path.reverse() for body_path in reversed(self.body_paths)])
    # def __repr__(self):
    #     index = self.index
    #     #index = id(self) % 1000
    #     return 'c{}'.format(index)

class ApplyForce(object):
    def __init__(self, body, robot, link):
        self.body = body
        self.robot = robot
        self.link = link
    def bodies(self):
        return {self.body, self.robot}
    def iterator(self, **kwargs):
        return []
    def refine(self, **kwargs):
        return self
    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__, self.robot, self.body)

# class Attach(ApplyForce):
#     def control(self, **kwargs):
#         # TODO: store the constraint_id?
#         add_fixed_constraint(self.body, self.robot, self.link)
#     def reverse(self):
#         return Detach(self.body, self.robot, self.link)
    

# class Detach(ApplyForce):
#     def control(self, **kwargs):
#         remove_fixed_constraint(self.body, self.robot, self.link)
#     def reverse(self):
#         return Attach(self.body, self.robot, self.link)


# class Attachment(object):
#     def __init__(self, parent, parent_link, grasp_pose, child):
#         self.parent = parent # TODO: support no parent
#         self.parent_link = parent_link
#         self.grasp_pose = grasp_pose
#         self.child = child
#         #self.child_link = child_link # child_link=BASE_LINK
#     @property
#     def bodies(self):
#         return flatten_links(self.child) | flatten_links(self.parent, get_link_subtree(
#             self.parent, self.parent_link))
#     def assign(self):
#         parent_link_pose = get_link_pose(self.parent, self.parent_link)
#         child_pose = body_from_end_effector(parent_link_pose, self.grasp_pose)
#         set_pose(self.child, child_pose)
#         return child_pose
#     def apply_mapping(self, mapping):
#         self.parent = mapping.get(self.parent, self.parent)
#         self.child = mapping.get(self.child, self.child)
#     def __repr__(self):
#         return '{}({},{})'.format(self.__class__.__name__, self.parent, self.child)



# others
def is_placement(body, surface):
    if body == 'gaizi' and surface == 'table':
        return True
    if body == 'drill' and surface == 'taizi' :
        return True
    return False

def assign_fluent_state(fluents):
    obstacles = []
    for fluent in fluents:
        name, args = fluent[0], fluent[1:]
        if name == 'atpose':
            o, p = args
            obstacles.append(o)
            p.assign()
        else:
            raise ValueError(name)
    return obstacles

# def move_gripper(robot, pose):
#     paramIds = self.paramIds
#     for i in range(len(paramIds)):
#         if i>5:
#             if self.jointIds[i]==8:
#                 # c = paramIds[i]
#                 p.setJointMotorControl2(robot,  9, p.POSITION_CONTROL, pose[0,0]*1.2, force=5 * 240.)
#                 p.setJointMotorControl2(robot, 16, p.POSITION_CONTROL, pose[0,1]*1.2, force=5 * 240.)
#                 p.setJointMotorControl2(robot, 23, p.POSITION_CONTROL, pose[0,2]*1.2, force=5 * 240.)
#                 p.setJointMotorControl2(robot, 30, p.POSITION_CONTROL, pose[0,3], force=5 * 240.)
                
#                 p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL, pose[1,0], force=5 * 240.)
#                 p.setJointMotorControl2(robot, 17, p.POSITION_CONTROL, pose[1,1], force=5 * 240.)
#                 p.setJointMotorControl2(robot, 24, p.POSITION_CONTROL, pose[1,2], force=5 * 240.)
#                 p.setJointMotorControl2(robot, 31, p.POSITION_CONTROL, pose[1,3], force=5 * 240.)
                    
#                 p.setJointMotorControl2(robot, 11, p.POSITION_CONTROL, pose[2,0], force=5 * 240.)
#                 p.setJointMotorControl2(robot, 18, p.POSITION_CONTROL, pose[2,1], force=5 * 240.)
#                 p.setJointMotorControl2(robot, 25, p.POSITION_CONTROL, pose[2,2], force=5 * 240.)
#                 p.setJointMotorControl2(robot, 32, p.POSITION_CONTROL, pose[2,3], force=5 * 240.)

#                 p.setJointMotorControl2(robot, 39, p.POSITION_CONTROL, pose[3,0], force=5 * 240.)
#                 p.setJointMotorControl2(robot, 38, p.POSITION_CONTROL, pose[3,1]*1.2, force=5 * 240.)
                    
#                 p.setJointMotorControl2(robot, 36, p.POSITION_CONTROL, 1.5, force=5 * 240.)
#                 p.setJointMotorControl2(robot, 37, p.POSITION_CONTROL, 0, force=5 * 240.)

#                 p.setJointMotorControl2(robot,  8, p.POSITION_CONTROL, 0, force=5 * 240.)
#                 p.setJointMotorControl2(robot, 15, p.POSITION_CONTROL, 0, force=5 * 240.)
#                 p.setJointMotorControl2(robot, 22, p.POSITION_CONTROL, 0, force=5 * 240.)
#                 p.setJointMotorControl2(robot, 29, p.POSITION_CONTROL, 0, force=5 * 240.)
#     pass

# def inverse_kinematics(robot, link, target_pose, max_iterations=200, max_time=INF, custom_limits={}, **kwargs):
#     start_time = time.time()
#     movable_joints = get_movable_joints(robot)
#     for iteration in irange(max_iterations):
#         # TODO: stop is no progress (converged)
#         # TODO: stop if collision or invalid joint limits
#         if elapsed_time(start_time) >= max_time:
#             return None
#         kinematic_conf = inverse_kinematics_helper(robot, link, target_pose)
#         if kinematic_conf is None:
#             return None
#         set_joint_positions(robot, movable_joints, kinematic_conf)
#         if is_pose_close(get_link_pose(robot, link), target_pose, **kwargs):
#             break
#     else:
#         return None
#     lower_limits, upper_limits = get_custom_limits(robot, movable_joints, custom_limits)
#     if not all_between(lower_limits, kinematic_conf, upper_limits):
#         return None
#     return kinematic_conf
def irange(start, end=None, step=1):
    if end is None:
        end = start
        start = 0
    n = start
    while n < end:
        yield n
        n += step

# def is_pose_close(current_pos, target_pos):
#     (pos, orn) = current_pos
#     (tar_pos, tar_orn) = target_pos
#     if (tar_pos is not None) and not is_p
#     return False

def inverse_kinematics(robot, target_position, max_iterations=200):
    # get movable joints
    movable_joints = get_movable_joints(robot)[:DOF_CONFIGURATIONS]
    
    rest_poses = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # start to calmulate the ik solution
    for iteration in irange(max_iterations):
        pos, orn = target_position
        # print("*******ik_movable_joints**************")
        # print(movable_joints)
        # end_effector_link = p.getJointInfo(robot, 6)[12].decode('UTF-8')
        # print("end_link", end_effector_link)
        joint_poses = p.calculateInverseKinematics(robot,6, pos, orn, rest_poses, maxNumIterations=20)
        # kinematic_conf = p.calculateInverseKinematics(robot, link, target_point, lowerLimits=lower, upperLimits=upper,
        #                                               jointRanges=ranges, restPoses=rest, physicsClientId=CLIENT)
        # print("*******ik_solutions_initial**************")
        # print("ik_solution:", joint_poses)
        
        # calculate the limitis
        joint_angle = joint_poses[1]
        min_angle_limit = -2.3
        max_angle_limit = 0.0
        joint_angle = max(min_angle_limit, min(joint_angle, max_angle_limit))
        # update the joint_poses
        joint_poses = tuple(joint_poses[:1] + (joint_angle,) + joint_poses[2:])

        joint_angle = joint_poses[2]
        min_angle_limit = 0.8
        max_angle_limit = 2.7
        joint_angle = max(min_angle_limit, min(joint_angle, max_angle_limit))
        joint_poses = tuple(joint_poses[:2] + (joint_angle,) + joint_poses[3:])

        # make sure the ik is not none
        if joint_poses is None:
            return None
        # print("*******ik_solutions**************")
        # print("current_pos:" , get_joint_pose(robot))
        # print("target_pos:", joint_poses)
        # reset the joint_state
        for i, joint_id in enumerate(movable_joints):
            p.resetJointState(robot, joint_id, targetValue=joint_poses[i],
                            targetVelocity = 5.00)

        # set the break condition
        current_position = get_joint_pose(robot)
        # print("current, target", current_position, target_position)
        if is_pose_close(get_joint_pose(robot), target_position) :
            break
    else:
        return None
    # print("*******ik_over**************")
    # print("return_the_ik_solutions:" , joint_poses)
    return joint_poses


# no collision(default)
def plan_direct_joint_motion(robot, joints , end_conf) :
    # get the start_conf
    start_conf = get_joint_positions_dof(robot)
    print("*******plan**************")
    print("start_conf:" , start_conf)
    joints = list(joints)
    print("joints:" , joints, type(start_conf))
    assert len(start_conf) == len(end_conf)
    waypoints = [start_conf] + [tuple(end_conf)]
    print("waypoints", waypoints)
    path = waypoints[:1]
    # generate a interpolating function
    extend_fn = get_extend_fn(robot, joints, resolutions = None)
    for waypoint in waypoints[1:]:
        assert len(joints) == len(waypoint)
        for q in list(extend_fn(path[-1], waypoint)):
            path.append(q)
    return path

def plan_joint_motion(robot, joints, end_conf, obstacles=[], attachments=[],
                      self_collisions=True, disabled_collisions=set(),
                      weights=None, resolutions=None, max_distance=0,
                      use_aabb=False, cache=True, custom_limits={}, algorithm=None, **kwargs):
    assert len(joints) == len(end_conf)
    sample_fn = get_sample_fn(robot, joints, custom_limits=custom_limits)
    distance_fn = get_distance_fn(robot, joints, weights=weights)
    extend_fn = get_extend_fn(robot, joints, resolutions=resolutions)
    collision_fn = get_collision_fn(robot, joints, obstacles, attachments, self_collisions, disabled_collisions,
                                    custom_limits=custom_limits, max_distance=max_distance,
                                    use_aabb=use_aabb, cache=cache)
    start_conf = get_joint_positions_dof(robot)
    print("start_conf", start_conf, "end_conf", end_conf)
    
    if algorithm is None:
        from motion_planners.rrt_connect import birrt
        return birrt(start_conf, end_conf, distance_fn, sample_fn, extend_fn, collision_fn, **kwargs)


# abandon funtion
def get_cfree_obj_approach_pose_test():
    def test(b1, p1, g1, b2, p2):
        return True
    return test


def get_collision_fn(robot, joints, obstacles, attachments, self_collisions, disabled_collisions,
                                    custom_limits, max_distance,
                                    use_aabb, cache):
    def collision_fn(q, verbose=False):
        return True
    return collision_fn

# def set_gripper_positions(robot, joints, array):
#     paramIds = list(p.addUserDebugParameter(p.getJointInfo(robot,i)[1].decode("utf-8"), -4, 4, 0) for i in range( p.getNumJoints(robot))\
#                     if p.getJointInfo(robot,i)[2] != p.JOINT_FIXED)
#     jointIds = list(i for i in range( p.getNumJoints(robot))\
#                     if p.getJointInfo(robot,i)[2] != p.JOINT_FIXED)
#     # print("*******************************************************")
#     # print("paramIds", paramIds)
#     # print("jointIds", jointIds)
#     for i in range(len(paramIds)):
#             if i>5:
#                 if jointIds[i]==8:
#                     # 后续此段代码可优化
#                     p.resetJointState(robot, joints[0][0], array[0,0]*1.2)
#                     p.resetJointState(robot, joints[0][1], array[0,1]*1.2)
#                     p.resetJointState(robot, joints[0][2], array[0,2]*1.2)
#                     p.resetJointState(robot, joints[0][3], array[0,3])
                
#                     p.resetJointState(robot, joints[1][0], array[1,0])
#                     p.resetJointState(robot, joints[1][1], array[1,1])
#                     p.resetJointState(robot, joints[1][2], array[1,2])
#                     p.resetJointState(robot, joints[1][3], array[1,3])
                    
#                     p.resetJointState(robot, joints[2][0], array[2,0])
#                     p.resetJointState(robot, joints[2][1], array[2,1])
#                     p.resetJointState(robot, joints[2][2], array[2,2])
#                     p.resetJointState(robot, joints[2][3], array[2,3])

#                     p.resetJointState(robot, joints[3][0], array[3,0])
#                     p.resetJointState(robot, joints[3][1], array[3,1]*1.2)
                    
#                     p.resetJointState(robot, joints[3][2], 1.5)
#                     p.resetJointState(robot, joints[3][3], 0)

#                     p.resetJointState(robot, joints[4][0], 0)
#                     p.resetJointState(robot, joints[4][1], 0)
#                     p.resetJointState(robot, joints[4][2], 0)
#                     p.resetJointState(robot, joints[4][3], 0)

# def move_gripper(robot, array):

#     paramIds = list(p.addUserDebugParameter(p.getJointInfo(robot,i)[1].decode("utf-8"), -4, 4, 0) for i in range( p.getNumJoints(robot))\
#                     if p.getJointInfo(robot,i)[2] != p.JOINT_FIXED)
#     jointIds = list(i for i in range( p.getNumJoints(robot))\
#                     if p.getJointInfo(robot,i)[2] != p.JOINT_FIXED)
#     # print("*******************************************************")
#     # print("paramIds", paramIds)
#     # print("jointIds", jointIds)
#     for i in range(len(paramIds)):
#             if i>5:
#                 if jointIds[i]==8:
#                     p.resetJointState(robot,  9, array[0,0]*1.2)
#                     p.resetJointState(robot, 16, array[0,1]*1.2)
#                     p.resetJointState(robot, 23, array[0,2]*1.2)
#                     p.resetJointState(robot, 30, array[0,3])
                
#                     p.resetJointState(robot, 10, array[1,0])
#                     p.resetJointState(robot, 17, array[1,1])
#                     p.resetJointState(robot, 24, array[1,2])
#                     p.resetJointState(robot, 31, array[1,3])
                    
#                     p.resetJointState(robot, 11, array[2,0])
#                     p.resetJointState(robot, 18, array[2,1])
#                     p.resetJointState(robot, 25, array[2,2])
#                     p.resetJointState(robot, 32, array[2,3])

#                     p.resetJointState(robot, 39, array[3,0])
#                     p.resetJointState(robot, 38, array[3,1]*1.2)
                    
#                     p.resetJointState(robot, 36, 1.5)
#                     p.resetJointState(robot, 37, 0)

#                     p.resetJointState(robot,  8, 0)
#                     p.resetJointState(robot, 15, 0)
#                     p.resetJointState(robot, 22, 0)
#                     p.resetJointState(robot, 29, 0)    
#     pass


def move_gripper(robot, array):

    paramIds = list(p.addUserDebugParameter(p.getJointInfo(robot,i)[1].decode("utf-8"), -4, 4, 0) for i in range( p.getNumJoints(robot))\
                    if p.getJointInfo(robot,i)[2] != p.JOINT_FIXED)
    jointIds = list(i for i in range( p.getNumJoints(robot))\
                    if p.getJointInfo(robot,i)[2] != p.JOINT_FIXED)
    # print("*******************************************************")
    # print("paramIds", paramIds)
    # print("jointIds", jointIds)
    for i in range(len(paramIds)):
            if i>5:
                if jointIds[i]==8:
                    p.setJointMotorControl2(robot,  9, p.POSITION_CONTROL, array[0,0]*1.2, force=5 * 240.)
                    p.setJointMotorControl2(robot, 16, p.POSITION_CONTROL, array[0,1]*1.2, force=5 * 240.)
                    p.setJointMotorControl2(robot, 23, p.POSITION_CONTROL, array[0,2]*1.2, force=5 * 240.)
                    p.setJointMotorControl2(robot, 30, p.POSITION_CONTROL, array[0,3], force=5 * 240.)
                
                    p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL, array[1,0], force=5 * 240.)
                    p.setJointMotorControl2(robot, 17, p.POSITION_CONTROL, array[1,1], force=5 * 240.)
                    p.setJointMotorControl2(robot, 24, p.POSITION_CONTROL, array[1,2], force=5 * 240.)
                    p.setJointMotorControl2(robot, 31, p.POSITION_CONTROL, array[1,3], force=5 * 240.)
                    
                    p.setJointMotorControl2(robot, 11, p.POSITION_CONTROL, array[2,0], force=5 * 240.)
                    p.setJointMotorControl2(robot, 18, p.POSITION_CONTROL, array[2,1], force=5 * 240.)
                    p.setJointMotorControl2(robot, 25, p.POSITION_CONTROL, array[2,2], force=5 * 240.)
                    p.setJointMotorControl2(robot, 32, p.POSITION_CONTROL, array[2,3], force=5 * 240.)

                    p.setJointMotorControl2(robot, 39, p.POSITION_CONTROL, array[3,0], force=5 * 240.)
                    p.setJointMotorControl2(robot, 38, p.POSITION_CONTROL, array[3,1]*1.2, force=5 * 240.)
                    
                    p.setJointMotorControl2(robot, 36, p.POSITION_CONTROL, 1.5, force=5 * 240.)
                    p.setJointMotorControl2(robot, 37, p.POSITION_CONTROL, 0, force=5 * 240.)

                    p.setJointMotorControl2(robot,  8, p.POSITION_CONTROL, 0, force=5 * 240.)
                    p.setJointMotorControl2(robot, 15, p.POSITION_CONTROL, 0, force=5 * 240.)
                    p.setJointMotorControl2(robot, 22, p.POSITION_CONTROL, 0, force=5 * 240.)
                    p.setJointMotorControl2(robot, 29, p.POSITION_CONTROL, 0, force=5 * 240.)    
    pass


def add_fixed_constraint(body, robot, max_force = None):
    # body_pose = get_pose(body)
    finger_links = GripperCtrl(robot).gripper_joints
    finger_links = [item for sublist in finger_links for item in sublist]
    body_link = -1
    constraints = []
    for finger_link in finger_links:
        end_effector_pose = p.getLinkState(robot, finger_link)[0:2]
        # end_eff_pos, end_eff_orn = end_effector_pose

        obj_pose = p.getBasePositionAndOrientation(body)
        # obj_pos, obj_orn = obj_pose

        grasp_pose = multiply(invert(end_effector_pose))
        grasp_pos, grasp_orn = grasp_pose
        constraint = p.createConstraint(robot, finger_link, body, body_link,  # Both seem to work
                                    p.JOINT_FIXED, jointAxis=unit_point(),
                                    parentFramePosition=grasp_pos,
                                    childFramePosition=unit_point(),
                                    parentFrameOrientation=grasp_orn,
                                    childFrameOrientation=unit_quat())
        constraints.append(constraint)

        if max_force is not None:
            p.changeConstraint(constraint, maxForce=max_force)
        return constraints 
    

# new class
# class Grasp(object):
#     num = count()
#     def __init__(self, robot, body, value):
#         self.body = body
#         self.robot = robot
#         self.value = value
#         self.index = next(self.num)
        
#     def get_attachment(self):
#         # end link
#         tool_link = get_end_eff_link(self.robot)
#         return Attachment(robot, self.body, tool_link, self.value)
        
#     def __repr__(self):
#         return 'g{}'.format(self.index)
            

# class Conf(object):
#     num = count()
#     def __init__(self,body, cof, joints=None):
#         # get controllable joints configgurations
#         if joints is None:
#             joints = get_movable_joints(body)
#         if configuration is None:
#             configuration = get_joint_positions(body, joints)   

#         # change the type of parameters
#         self.body = body
#         self.joints = joints
#         self.configuration = configuration[:DOF_CONFIGURATIONS]
#         self.index = next(self.num)

#     def values(self):
#         return self.configuration
    
#     def assign(self):
#         set_joint_positions(self.body, self.joints, self.values)

#     def iterate(self):
#         yield self

#     def __repr__(self) -> str:
#         index = self.index
#         return 'q{}'.format(index)

# class Command(object):
#     def control(self, dt=0):
#         raise NotImplementedError()
#     def apply(self, state, **kwargs):
#         raise NotImplementedError()
#     def iterate(self):
#         raise NotImplementedError()


# class Commands(object):
#     num = count()
#     def __init__(self, state, savers=[], commands=[]):
#         self.state = state
#         self.savers = tuple(savers)
#         self.commands = tuple(commands)
#         self.index = next(self.num)
        
#     def assign(self):
#         for saver in self.savers:
#             saver.restore()
#         return copy.copy(self.state)
    
#     def apply(self, state, **kwargs):
#         for command in self.commands:
#             for result in command.apply(state, **kwargs):
#                 yield result
    
#     def __repe__(self):
#         return 'c{}'.format(self.index)

# # Derived class
# class Trajectory(Command):
#     _draw = False
#     def __init__(self, path):
#         self.path = tuple(path)
#         # TODO: constructor that takes in this info
#     def apply(self, state, sample=1):
#         handles = add_segments(self.to_points()) if self._draw and has_gui() else []
#         for conf in self.path[::sample]:
#             conf.assign()
#             yield
#         end_conf = self.path[-1]
#         if isinstance(end_conf, Pose):
#             state.poses[end_conf.body] = end_conf
#         for handle in handles:
#             remove_debug(handle)
#     def control(self, dt=0, **kwargs):
#         # TODO: just waypoints
#         for conf in self.path:
#             if isinstance(conf, Pose):
#                 conf = conf.to_base_conf()
#             for _ in joint_controller_hold(conf.body, conf.joints, conf.values):
#                 step_simulation()
#                 time.sleep(dt)
#     def to_points(self, link=BASE_LINK):
#         # TODO: this is computationally expensive
#         points = []
#         for conf in self.path:
#             with BodySaver(conf.body):
#                 conf.assign()
#                 #point = np.array(point_from_pose(get_link_pose(conf.body, link)))
#                 point = np.array(get_group_conf(conf.body, 'base'))
#                 point[2] = 0
#                 point += 1e-2*np.array([0, 0, 1])
#                 if not (points and np.allclose(points[-1], point, atol=1e-3, rtol=0)):
#                     points.append(point)
#         points = get_target_path(self)
#         return waypoints_from_path(points)
#     def distance(self, distance_fn=get_distance):
#         total = 0.
#         for q1, q2 in zip(self.path, self.path[1:]):
#             total += distance_fn(q1.values, q2.values)
#         return total
#     def iterate(self):
#         for conf in self.path:
#             yield conf
#     def reverse(self):
#         return Trajectory(reversed(self.path))
#     #def __repr__(self):
#     #    return 't{}'.format(id(self) % 1000)
#     def __repr__(self):
#         d = 0
#         if self.path:
#             conf = self.path[0]
#             d = 3 if isinstance(conf, Pose) else len(conf.joints)
#         return 't({},{})'.format(d, len(self.path))

# def create_trajectory(robot, joints, path):
#     return Trajectory(Conf(robot, joints, q) for q in path)

# class GripperCommand(Command):
#     def __init__(self, robot, array, teleport=False):
#         self.robot = robot
#         self.teleport = teleport
#         self.joints = np.array([
#                                    [9, 16, 23, 20],
#                                    [10, 17, 24, 31],
#                                    [11, 18, 25, 32],
#                                    [39, 38, 36, 37],
#                                    [8, 15, 22, 29]])
#         self.array = array

#     def apply(self, state, **kwargs):
#         # 规划grasp路径
#         joints = get_gripper_joints(self.robot, self.arm)
#         start_conf = get_joint_positions(self.robot, joints)
#         # end_conf = [self.position] * len(joints)
#         end_conf = ([self.position] if isinstance(self.position, float) else list(self.position)) * len(joints) 
#         if self.teleport:
#             path = [start_conf, end_conf]
#         else:
#             extend_fn = get_extend_fn(self.robot, joints)
#             path = [start_conf] + list(extend_fn(start_conf, end_conf))
#             # path = [start_conf] + list(extend_fn(start_ex_conf, end_conf))
        
#         # print("*****************paths*******************************")
#         # print("path:", path, "path_type", type(path))
            
#         for positions in path:
#             set_joint_positions(self.robot, joints,  positions)
#             yield positions

#     def control(self, **kwargs):
#         joints = self.joints
#         positions = self.array
#         for _ in joint_controller_hold(self.robot, joints, positions):
#             yield

#     def __repr__(self):
#         return '{}({},{},{})'.format(self.__class__.__name__, get_body_name(self.robot),
#                                      self.arm, self.position)


# def get_target_point(conf):
#     # TODO: use full body aabb
#     robot = conf.body
#     link = link_from_name(robot, 'torso_lift_link')
#     #link = BASE_LINK
#     # TODO: center of mass instead?
#     # TODO: look such that cone bottom touches at bottom
#     # TODO: the target isn't the center which causes it to drift
#     with BodySaver(conf.body):
#         conf.assign()
#         lower, upper = get_aabb(robot, link)
#         center = np.average([lower, upper], axis=0)
#         point = np.array(get_group_conf(conf.body, 'base'))
#         #point[2] = upper[2]
#         point[2] = center[2]
#         #center, _ = get_center_extent(conf.body)
#         return point


# def get_target_path(trajectory):
#     # TODO: only do bounding boxes for moving links on the trajectory
#     return [get_target_point(conf) for conf in trajectory.path]



# class BodyConf(object):

#     num = count()

#     def __init__(self, body, configuration=None, joints=None):
#         # get controllable joints configgurations
#         if joints is None:
#             joints = get_movable_joints(body)
#         if configuration is None:
#             configuration = get_joint_positions(body, joints)
        
#         # change the type of parameters
#         self.body = body
#         self.joints = joints
#         self.configuration = configuration[:DOF_CONFIGURATIONS]
#         self.index = next(self.num)
#         pass

#     # get the joint's current posture 
#     def values(self):
#         return self.configuration
    
#     # set the robot postures
#     def assign(self):
#         for i, joint_id in enumerate(self.joints):
#             p.resetJointState(self.body, joint_id, targetValue=self.configuration[i],
#                             targetVelocity = 0)
#         # set_joint_positions(self.body, self.joints, self.configuration)
#         return self.configuration
    
#     # define the ouput of robot posture
#     def __repr__(self) -> str:
#         index = self.index
#         return 'q{}'.format(index)
#     pass