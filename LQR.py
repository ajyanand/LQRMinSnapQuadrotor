#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import String
import geometry_msgs.msg
from std_srvs.srv import Empty
import time

from pathlib import Path
import inspect
from occupancy_map import OccupancyMap
from se3_control import SE3Control
from world_traj import WorldTraj

from flightsim.world import World

#CHANGE TO ACTUAL QUADROTOR PARAMETERS LATER
from crazyflie_params import quad_params


class Planner:
    def __init__(self, quad_params):
        #file = Path(inspect.getsourcefile(lambda:0)).parent.resolve() / '..' / 'util' / mapfilename
        print("ding")
        filepath="/home/ajay/catkin_ws/src/planner_template/scripts/util/test_over_under.json"
        self.world = World.from_file(filepath)          # World boundary and obstacles.
        self.start  = self.world.world['start']          # Start point, shape=(3,)
        self.goal   = self.world.world['goal']           # Goal point, shape=(3,)

        self.controller = SE3Control(quad_params)

        t_s = time.time()
        self.LQRtraj = WorldTraj(self.world, self.start, self.goal)
        print("Planning time: ", time.time()-t_s)

        self.curr_state  = {'x': self.start,
                            'v': (0, 0, 0),
                            'q': (0, 0, 0, 1), # [i,j,k,w]
                            'w': (0, 0, 0)}
    def cmd_command(self):
        cmd = geometry_msgs.msg.PoseStamped()
        cmd.header.frame_id = "world"
    
        t = rospy.get_time()
        X = self.LQRtraj.update(t)
        print("time: ",t)
        print(X['x'])
        cmd.pose.position.x = X['x'][0]
        cmd.pose.position.y = X['x'][1]
        cmd.pose.position.z = X['x'][2]
        return cmd
    
def open_loop(LQR):
    rospy.init_node('LQR', anonymous=True)
    des_pose = rospy.Publisher('/command/pose', geometry_msgs.msg.PoseStamped,queue_size=10)
    rate = rospy.Rate(15) # 10hz
    while not rospy.is_shutdown():

        des_pose.publish(LQR.cmd_command())
        rate.sleep()

if __name__ == '__main__':
	try:
		LQR = Planner(quad_params)
		r = rospy.ServiceProxy('/gazebo/reset_simulation',Empty)
		r()
		print("Reset")
		open_loop(LQR)
	except rospy.ROSInterruptException:
		pass
