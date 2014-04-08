#!/usr/bin/env python

import argparse, numpy, openravepy, time

from HerbRobot import HerbRobot
from SimpleRobot import SimpleRobot
from HerbEnvironment import HerbEnvironment
from SimpleEnvironment import SimpleEnvironment

from AStarPlanner import AStarPlanner
from DepthFirstPlanner import DepthFirstPlanner
from BreadthFirstPlanner import BreadthFirstPlanner
from HeuristicRRTPlanner import HeuristicRRTPlanner

def main(robot, planning_env, planner, vis_short=False, bis_short=False):

    raw_input('Press any key to begin planning')

    start_config = numpy.array(robot.GetCurrentConfiguration())
    if robot.name == 'herb':
        goal_config = numpy.array([ 4.6, -1.76, 0.00, 1.96, -1.15, 0.87, -1.43] )
    else:
        goal_config = numpy.array([3.0, 0.0])

    plan = planner.Plan(start_config, goal_config)
    traj = robot.ConvertPlanToTrajectory(plan)

    if not vis_short and not bis_short:
        raw_input('Press any key to execute trajectory')
        robot.ExecuteTrajectory(traj)
    else:
        if vis_short:
            planning_env.PlotPath(plan)
        #traj = robot.ConvertPlanToTrajectory(plan)

        plan_short = planning_env.ShortenPath(plan,10.0,bis_short)
        if vis_short:
            planning_env.PlotPath(plan_short)
        traj_short = robot.ConvertPlanToTrajectory(plan_short)

        # for debugging before the execution
        #print "Showing initial path..."
        #robot.ExecuteTrajectory(traj)
        xxx = raw_input("Type any button to continue to shortened path...")
        print "Showing shortened path"
        robot.ExecuteTrajectory(traj_short)


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='script for testing planners')
    
    parser.add_argument('-r', '--robot', type=str, default='simple',
                        help='The robot to load (herb or simple)')
    parser.add_argument('-p', '--planner', type=str, default='astar',
                        help='The planner to run (astar, bfs, dfs or hrrt)')
    parser.add_argument('-v', '--visualize', action='store_true',
                        help='Enable visualization of tree growth (only applicable for simple robot)')
    parser.add_argument('--resolution', type=float, default=0.1,
                        help='Set the resolution of the grid (default: 0.1)')
    parser.add_argument('-d', '--debug', action='store_true',
                        help='Enable debug logging')
    parser.add_argument('-m', '--manip', type=str, default='right',
                        help='The manipulator to plan with (right or left) - only applicable if robot is of type herb')
    parser.add_argument('-vs', '--visualize_short', action='store_true',
                        help='Enable visualization of path shortening (only applicable for simple robot)')
    parser.add_argument('-bs', '--bisection_short', action='store_true',
                        help='Enable bisection path shortening')
    args = parser.parse_args()
    
    openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    openravepy.misc.InitOpenRAVELogging()

    if args.debug:
        openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug)

    env = openravepy.Environment()
    env.SetViewer('qtcoin')
    env.GetViewer().SetName('Homework 2 Viewer')

    # First setup the environment and the robot
    visualize = args.visualize
    visualize_short = args.visualize_short
    bisection_short = args.bisection_short
    if args.robot == 'herb':
        robot = HerbRobot(env, args.manip)
        planning_env = HerbEnvironment(robot, args.resolution)
        visualize = False
        visualize_short = False
    elif args.robot == 'simple':
        robot = SimpleRobot(env)
        planning_env = SimpleEnvironment(robot, args.resolution)
    else:
        print 'Unknown robot option: %s' % args.robot
        exit(0)

    # Next setup the planner
    if args.planner == 'astar':
        planner = AStarPlanner(planning_env, visualize)
    elif args.planner == 'bfs':
        planner = BreadthFirstPlanner(planning_env, visualize)
    elif args.planner == 'dfs':
        planner = DepthFirstPlanner(planning_env, visualize)
    elif args.planner == 'hrrt':
        planner = HeuristicRRTPlanner(planning_env, visualize)
    else:
        print 'Unknown planner option: %s' % args.planner
        exit(0)

    main(robot, planning_env, planner, visualize_short, bisection_short)
    xxx = raw_input("Type any button to exit!")
    import IPython
    IPython.embed()

        
    
