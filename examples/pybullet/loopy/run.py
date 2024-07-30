#!/usr/bin/env python

from __future__ import print_function

from pddlstream.algorithms.meta import solve, create_parser
from examples.pybullet.utils.pybullet_tools.loopy_primitives import BodyPose, BodyConf, Command, get_grasp_gen, \
    get_stable_gen, get_ik_fn, get_free_motion_gen, \
    get_holding_motion_gen, get_cfree_obj_approach_pose_test, get_cfree_pose_pose_test,\
    get_movable_collision_test
from examples.pybullet.utils.pybullet_tools.utils import WorldSaver, connect, dump_body, get_pose, set_pose, Pose, \
    Point, set_default_camera, stable_z, \
    BLOCK_URDF, SMALL_BLOCK_URDF, get_configuration, SINK_URDF, STOVE_URDF, load_model, is_placement, get_body_name, \
    disconnect, DRAKE_IIWA_URDF, get_bodies, HideOutput, wait_for_user, KUKA_IIWA_URDF, add_data_path, load_pybullet, \
    LockRenderer, has_gui, draw_pose, draw_global_system,UR5_UNI_ARM_URDF, UR5_URDF,\
    get_movable_joints, set_joint_positions, get_link_state, CAMERA_SLOT_URDF, GAIZI_URDF,\
    Euler,TAIZI_URDF,DRILL_URDF
from pddlstream.language.generator import from_gen_fn, from_fn, empty_gen, from_test, universe_test
from pddlstream.utils import read, INF, get_file_path, find_unique, Profiler, str_from_object, negate_test
from pddlstream.language.constants import print_solution, PDDLProblem
# from examples.pybullet.tamp.streams import get_cfree_approach_pose_test, get_cfree_pose_pose_test, get_cfree_traj_pose_test, \
#     move_cost_fn, get_cfree_obj_approach_pose_test

import math

HOME_CONF=[0,-1.57,1.57,0,0,0]
# HOME_CONF = [-1.62,-1.57,-1.57,-1.35,1.62,0]
# HOME_CONF=[0,0,0,0,0,0]
# HOME_CONF=[-6.666624994380624, -6.289973104499651, -2.719662298974622, -5.458182921074028, -3.2201010639495706, -6.276911581047255]
OCCUPIED_FLAG=True

def get_fixed(robot, movable):
    rigid = [body for body in get_bodies() if body != robot]
    fixed = [body for body in rigid if body not in movable]
    return fixed

def place_movable(certified):
    placed = []
    for literal in certified:
        if literal[0] == 'not':
            fact = literal[1]
            if fact[0] == 'trajcollision':
                _, b, p = fact[1:]
                set_pose(b, p.pose)
                placed.append(b)
    return placed

def get_free_motion_synth(robot, movable=[], teleport=False):
    fixed = get_fixed(robot, movable)
    def fn(outputs, certified):
        assert(len(outputs) == 1)
        q0, _, q1 = find_unique(lambda f: f[0] == 'freemotion', certified)[1:]
        obstacles = fixed + place_movable(certified)
        free_motion_fn = get_free_motion_gen(robot, obstacles, teleport)
        return free_motion_fn(q0, q1)
    return fn

def get_holding_motion_synth(robot, movable=[], teleport=False):
    fixed = get_fixed(robot, movable)
    def fn(outputs, certified):
        assert(len(outputs) == 1)
        q0, _, q1, o, g = find_unique(lambda f: f[0] == 'holdingmotion', certified)[1:]
        obstacles = fixed + place_movable(certified)
        holding_motion_fn = get_holding_motion_gen(robot, obstacles, teleport)
        return holding_motion_fn(q0, q1, o, g)
    return fn

#######################################################

def pddlstream_from_problem(robot, goal, movable=[], fixed=[], init = [], teleport=False, grasp_name='top'):
    #assert (not are_colliding(tree, kin_cache))

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}

    stream_map = {
        'sample-pose': from_gen_fn(get_stable_gen(fixed)),
        'sample-grasp': from_gen_fn(get_grasp_gen(robot, grasp_name)),
        'inverse-kinematics': from_fn(get_ik_fn(robot, fixed, teleport)),
        'plan-free-motion': from_fn(get_free_motion_gen(robot, fixed, teleport)),
        'plan-holding-motion': from_fn(get_holding_motion_gen(robot, fixed, teleport)),

        'test-cfree-pose-pose': from_test(get_cfree_pose_pose_test()),
        'test-cfree-approach-pose': from_test(get_cfree_obj_approach_pose_test()),
        'test-cfree-traj-pose': from_test(negate_test(get_movable_collision_test())), #get_cfree_traj_pose_test()),

        'TrajCollision': get_movable_collision_test(),
    }

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)


#######################################################

def load_world():
    # TODO: store internal world info here to be reloaded
    set_default_camera()
    draw_global_system()
    with HideOutput():
        robot = load_model(UR5_URDF, fixed_base=True) # DRAKE_IIWA_URDF | KUKA_IIWA_URDF
        # floor = load_model('models/short_floor.urdf')
        floor = load_pybullet("plane.urdf")
        table = load_pybullet("table/table.urdf", scale = 2.0)
        # sink = load_model(SINK_URDF, pose=Pose(Point(x=-0.5)))
        sink = load_model(CAMERA_SLOT_URDF, pose=Pose(Point(x=-0.8, y=+0.5)))
        stove = load_model(STOVE_URDF, pose=Pose(Point(x=+0.5)))
        celery = load_model(GAIZI_URDF, fixed_base=False)
        radish = load_model(SMALL_BLOCK_URDF, fixed_base=False)
        radish_2 = load_model(SMALL_BLOCK_URDF, fixed_base=False)
        taizi = load_model(TAIZI_URDF, pose=Pose(Point(y=+0.6)))
        # drill = load_model(DRILL_URDF, pose=Pose(Point(y=+1.0)))
        drill = load_model(DRILL_URDF, fixed_base=False)
        #cup = load_model('models/dinnerware/cup/cup_small.urdf',

    body_names = {
        sink: 'sink',
        stove: 'stove',
        celery: 'celery',
        radish: 'radish',
        radish_2: 'radish_2',
        table:'table'
    }
    movable_bodies = [celery, radish, radish_2, drill]

    movable_joints = get_movable_joints(robot)
    set_joint_positions(robot, movable_joints, HOME_CONF)
    print(get_link_state(robot, 7))


    set_pose(celery, Pose(Point(x=-0.4, y=0.4, z=stable_z(celery, floor)),
                         Euler(0,0, 0)))
    set_pose(radish, Pose(Point(x = -0.35, y=-0.3, z=stable_z(radish, floor))))
    set_pose(radish_2, Pose(Point(x = -0.45, y=-0.35, z=stable_z(radish_2, floor))))

    set_pose(floor, Pose(Point(x=0, y=0, z=-2*0.625)))
    set_pose(table, Pose(Point(x=0, y=0, z=-2*0.625)))
    set_pose(robot, Pose(Point(x=0, y=0, z=0.01)))
    set_pose(sink, Pose(Point(x=-0.4 ,y=-0.3 ,z=-0.011),Euler(math.radians(90), 0, 0)))

    set_pose(taizi, Pose(Point(x=-0.9, y=0, z=stable_z(taizi, table))))
    set_pose(drill, Pose(Point(x=-0.9, y=0,z=stable_z(drill, taizi)), 
                         Euler( 90,0, 0)))

    return robot, body_names, movable_bodies

def postprocess_plan(plan):
    paths = []
    for name, args in plan:
        if name == 'place':
            paths += args[-1].reverse().body_paths
        elif name in ['move', 'move_free', 'move_holding', 'pick']:
            paths += args[-1].body_paths
    return Command(paths)

#######################################################

def get_goal(robot,movable=[],):

    print('Robot:', robot)
    conf = BodyConf(robot, get_configuration(robot))
    init = [('CanMove',),
            ('Conf', conf),
            ('AtConf', conf),
            ('HandEmpty',)]

    fixed = get_fixed(robot, movable)
    print('Movable:', movable)
    print('Fixed:', fixed)
    for body in movable:
        pose = BodyPose(body, get_pose(body))
        init += [('Graspable', body),
                 ('Pose', body, pose),
                 ('AtPose', body, pose)]
        for surface in fixed:
            init += [('Stackable', body, surface)]
            if is_placement(body, surface):
                init += [('Supported', body, pose, surface)]

    for body in fixed:
        name = get_body_name(body)
        if 'cup' in name:
            init += [('Sink', body)]
        if 'stove' in name:
            init += [('Stove', body)]
        if 'table' in name:
            init += [('Table', body)]

    # 初始化任务规划所需谓词
    init += [('Occupied', fixed[2])] 
    init += [('Block1', movable[1],), ('Block2',movable[2],)]
    init += [('Gaizi', movable[0],)]
    # init += [('Refuse_pick',)]
    # init += [('In_region', movable[1], fixed[2]),
    #          ('In_region', movable[2], fixed[2]),]
    init += [('Block1_in_region', movable[1], fixed[2]),
             ('Block2_in_region', movable[2], fixed[2])]

    body_0 = movable[0]
    body = movable[1]
    body2 = movable[2]
    body3 = movable[3]
    # print("movable:", movable)
    movable_name = [get_body_name(x) for x in movable]
    fixed_name = [get_body_name(x) for x in fixed]
    print(movable_name)
    print(fixed_name)

    print("fixed:", fixed)
    goal = ('and',
            ('AtConf', conf),
            ('Pick_gaizi', body_0, fixed[2]),
            )
    goal2 = ('and',
             ('AtConf', conf),
             ('On', body_0, fixed[2]))
    
    goal3 = ('and',
             ('AtConf', conf),
            ('On', body3, fixed[2]),
             )
    return goal, goal2, goal3, init, fixed

def main():
    parser = create_parser()
    parser.add_argument('-enable', action='store_true', help='Enables rendering during planning')
    parser.add_argument('-teleport', action='store_true', help='Teleports between configurations')
    parser.add_argument('-simulate', action='store_true', help='Simulates the system')
    args = parser.parse_args()
    print('Arguments:', args)

    connect(use_gui=True)
    robot, names, movable = load_world()
    print('Objects:', names)
    saver = WorldSaver()

    goal, goal2, goal3, init, fixed = get_goal(robot=robot, movable=movable)

    problem = pddlstream_from_problem(robot, goal=goal, movable=movable, fixed=fixed, init=init, teleport=args.teleport)
    problem2 = pddlstream_from_problem(robot, goal=goal2, movable=movable, fixed=fixed, init=init, teleport=args.teleport)
    problem3 = pddlstream_from_problem(robot, goal=goal3, movable=movable, fixed=fixed, init=init, teleport=args.teleport)
    
    _, _, _, stream_map, init, goal = problem
    _, _, _, stream_map2, init, goal2 = problem2
    _, _, _, stream_map3, init, goal3 = problem3

    print('Init:', init)
    print('Goal:', goal, goal2, goal3)
    print('Streams:', str_from_object(set(stream_map)), 
                      str_from_object(set(stream_map2)), 
                      str_from_object(set(stream_map3)))

    with Profiler():
        with LockRenderer(lock=not args.enable):
            solution = solve(problem, algorithm=args.algorithm, unit_costs=args.unit, success_cost=INF)
            solution2 = solve(problem2, algorithm=args.algorithm, unit_costs=args.unit, success_cost=INF)
            solution3 = solve(problem3, algorithm=args.algorithm, unit_costs=args.unit, success_cost=INF)
            saver.restore()
    print_solution(solution)
    plan, cost, evaluations = solution
    plan2, _, _ = solution2
    plan3, _, _ = solution3
    if (plan is None) or not has_gui():
        disconnect()
        return

    command = postprocess_plan(plan)
    command2 = postprocess_plan(plan2)
    command3 = postprocess_plan(plan3)
    # print("*************************************")
    # print(plan)
    # print(type(plan))
    # command.control()
    if args.simulate:
        wait_for_user('Simulate?')
        # command.control()
    else:
        wait_for_user('Execute?')
        command.refine(num_steps=10).execute(time_step=0.01)
        command2.refine(num_steps=10).execute(time_step=0.01)
        command3.refine(num_steps=10).execute(time_step=0.01)
    wait_for_user('Finish?')
    disconnect()

if __name__ == '__main__':
    main()