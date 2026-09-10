#!/usr/bin/env python3
"""Plan with MoveIt, execute through ros2_control, verify the physical cube.

No model teleportation, attach plugin, or real-robot SDK is used by this demo.
Success requires measured object lift AND return to the pedestal.
"""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetMotionPlan, ApplyPlanningScene
from moveit_msgs.msg import Constraints, JointConstraint, CollisionObject
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from ep_simulation.model import ARM_JOINTS, HOME, valid
from ep_simulation.scene import PICK, LIFT, CUBE_START, TABLE_HEIGHT


class Demo(Node):
    def __init__(self):
        super().__init__('ep_pick_demo')
        self.positions = {}
        self.create_subscription(JointState, '/ep_joint_states', self.joints, 10)
        self.plan = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.entity = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.execute = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.gripper = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.events = []

    def joints(self, msg):
        self.positions.update(zip(msg.name, msg.position))

    def wait(self, future, timeout=40):
        end = time.monotonic()+timeout
        while rclpy.ok() and not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise RuntimeError('Timed out waiting for ROS response')
        if future.exception():
            raise future.exception()
        return future.result()

    def pause(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def record(self, stage, **data):
        event = {'stage': stage, 'wall_time': time.time(), **data}
        self.events.append(event)
        print(json.dumps(event, ensure_ascii=False), flush=True)

    def ready(self):
        for client in [self.plan, self.scene, self.entity]:
            if not client.wait_for_service(timeout_sec=60):
                raise RuntimeError('Missing service: '+client.srv_name)
        for client in [self.execute, self.gripper]:
            if not client.wait_for_server(timeout_sec=30):
                raise RuntimeError('Missing trajectory action server')
        self.pause(1)
        if not all(j in self.positions for j in ARM_JOINTS):
            raise RuntimeError('No arm joint states received')

    def setup_scene(self):
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        for name, dims, xyz in [('pedestal', [0.08,0.12,TABLE_HEIGHT], [0.305,CUBE_START[1],TABLE_HEIGHT/2]),
                                ('floor', [2.0,2.0,0.01], [0.0,0.0,-0.005])]:
            obj = CollisionObject()
            obj.id = name; obj.header.frame_id = 'world'; obj.operation = CollisionObject.ADD
            primitive = SolidPrimitive(); primitive.type = SolidPrimitive.BOX; primitive.dimensions = dims
            pose = Pose(); pose.position.x, pose.position.y, pose.position.z = xyz; pose.orientation.w = 1.0
            obj.primitives.append(primitive); obj.primitive_poses.append(pose)
            req.scene.world.collision_objects.append(obj)
        if not self.wait(self.scene.call_async(req)).success:
            raise RuntimeError('Failed to install planning scene')

    def arm(self, name, q):
        if not valid(q):
            raise ValueError('EP joint/motor limits exceeded')
        req = GetMotionPlan.Request(); m = req.motion_plan_request
        m.group_name = 'arm'; m.planner_id = 'RRTConnectkConfigDefault'
        m.num_planning_attempts = 5; m.allowed_planning_time = 5.0
        m.max_velocity_scaling_factor = 0.35; m.max_acceleration_scaling_factor = 0.35
        m.start_state.is_diff = True
        m.workspace_parameters.header.frame_id = 'world'
        for axis in ['x','y','z']:
            setattr(m.workspace_parameters.min_corner, axis, -1.0)
            setattr(m.workspace_parameters.max_corner, axis, 1.0)
        goal = Constraints()
        for j, value in zip(ARM_JOINTS,q):
            c=JointConstraint();c.joint_name=j;c.position=float(value)
            c.tolerance_above=0.001;c.tolerance_below=0.001;c.weight=1.0
            goal.joint_constraints.append(c)
        m.goal_constraints.append(goal)
        response = self.wait(self.plan.call_async(req)).motion_plan_response
        if response.error_code.val != 1:
            raise RuntimeError('MoveIt planning failed at '+name+': '+str(response.error_code.val))
        trajectory = response.trajectory
        names = trajectory.joint_trajectory.joint_names
        indices = [names.index(n) for n in ARM_JOINTS]
        for point in trajectory.joint_trajectory.points:
            if not valid([point.positions[i] for i in indices]):
                raise RuntimeError('Planned path violates coupled EP motor limits')
        goal = ExecuteTrajectory.Goal();goal.trajectory=trajectory
        handle=self.wait(self.execute.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('Execution rejected')
        try:
            result=self.wait(handle.get_result_async(), timeout=90).result
        except Exception:
            self.wait(handle.cancel_goal_async(), timeout=5)
            raise
        if result.error_code.val != 1:
            raise RuntimeError('Trajectory execution failed: '+str(result.error_code.val))
        self.pause(0.4)
        actual=[self.positions[j] for j in ARM_JOINTS]
        if max(abs(a-b) for a,b in zip(actual,q)) > 0.02:
            raise RuntimeError('Arm endpoint joint error too large')
        self.record(name, joints=actual, trajectory_points=len(trajectory.joint_trajectory.points))

    def fingers(self, opening):
        goal=FollowJointTrajectory.Goal()
        goal.trajectory.joint_names=['left_finger_joint']
        p=JointTrajectoryPoint();p.positions=[opening];p.time_from_start.sec=2
        goal.trajectory.points=[p]
        handle=self.wait(self.gripper.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('Gripper goal rejected')
        result=self.wait(handle.get_result_async(), timeout=20).result
        if result.error_code != 0:
            raise RuntimeError('Gripper trajectory failed: '+result.error_string)
        self.pause(0.7)
        self.record('gripper', commanded_half_gap=opening)

    def cube(self):
        req=GetEntityState.Request();req.name='target_cube';req.reference_frame='world'
        result=self.wait(self.entity.call_async(req))
        if not result.success:
            raise RuntimeError('Gazebo cannot locate target_cube')
        p=result.state.pose.position
        return [p.x,p.y,p.z]

    def run(self):
        self.ready();self.setup_scene();self.fingers(0.04)
        self.arm('home',HOME);self.arm('approach',LIFT)
        initial=self.cube();self.record('cube_initial',position=initial)
        if math.dist(initial,CUBE_START)>0.008:
            raise RuntimeError('Cube is not at the pickup point; restart the simulation to reset')
        self.arm('pick',PICK);self.fingers(0.011)
        self.arm('lift',LIFT);self.pause(1)
        lifted=self.cube();lift=lifted[2]-initial[2]
        self.record('cube_lifted',position=lifted,lift_m=lift)
        if lift<0.025:
            self.fingers(0.04)
            raise RuntimeError('Physical grasp failed: cube did not lift by at least 25 mm')
        self.arm('replace',PICK);self.fingers(0.04)
        self.arm('retract',LIFT);self.arm('home_final',HOME);self.pause(1)
        final=self.cube();error=math.dist(final,initial)
        self.record('cube_final',position=final,return_error_m=error)
        if error>0.012:
            raise RuntimeError('Cube did not return to the pedestal within 12 mm')
        return {'success':True,'lift_m':lift,'return_error_m':error}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output', default='work/pick_result.json')
    args=parser.parse_args()
    rclpy.init();node=Demo();result={}
    try:
        result=node.run()
    except Exception as error:
        result={'success':False,'error':str(error)}
        print('FAILED:',error,flush=True)
    finally:
        result['events']=node.events
        output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
        node.execute.destroy();node.gripper.destroy()
        node.destroy_node();rclpy.shutdown()
    raise SystemExit(0 if result.get('success') else 1)


if __name__=='__main__':
    main()
