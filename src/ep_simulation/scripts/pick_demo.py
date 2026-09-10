#!/usr/bin/env python3
"""Plan with MoveIt, execute through ros2_control, verify the physical bottle.

No model teleportation, attach plugin, or real-robot SDK is used by this demo.
Success requires measured lift, placement at B, and upright settling on the floor.
"""
import argparse
import json
import math
import time
import sys
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetMotionPlan, ApplyPlanningScene
from moveit_msgs.msg import Constraints, JointConstraint, CollisionObject, AttachedCollisionObject
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from ep_simulation.model import ARM_JOINTS, HOME, valid, fk, ik
from ep_simulation.scene import load_config, targets, bottle_start, bottle_goal, bottle_mass


class Demo(Node):
    def __init__(self, cfg):
        super().__init__('ep_pick_demo')
        self.cfg = cfg
        self.targets = targets(cfg)
        self.positions = {}
        self.create_subscription(JointState, '/ep_joint_states', self.joints, 10)
        self.plan = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.entity = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.execute = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.gripper = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.events = []
        self.attached = False

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
        print(json.dumps({k:v for k,v in event.items() if k != 'trajectory'}, ensure_ascii=False), flush=True)

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

    def apply_scene(self, req):
        if not self.wait(self.scene.call_async(req)).success:
            raise RuntimeError('Failed to update planning scene')

    def bottle_collision(self, pose):
        obj = CollisionObject()
        obj.id = 'target_bottle'; obj.header.frame_id = 'world'; obj.operation = CollisionObject.ADD
        shape = SolidPrimitive(); shape.type = SolidPrimitive.CYLINDER
        shape.dimensions = [self.cfg['height_m'], self.cfg['diameter_m']/2]
        obj.primitives.append(shape); obj.primitive_poses.append(pose)
        return obj

    def setup_scene(self):
        req = ApplyPlanningScene.Request(); req.scene.is_diff = True
        obj = CollisionObject()
        obj.id = 'floor'; obj.header.frame_id = 'world'; obj.operation = CollisionObject.ADD
        primitive = SolidPrimitive(); primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [2.0, 2.0, 0.01]
        # 1 mm tolerance for ODE's sub-millimeter contact penetration.
        pose = Pose(); pose.position.z = -0.006; pose.orientation.w = 1.0
        obj.primitives.append(primitive); obj.primitive_poses.append(pose)
        req.scene.world.collision_objects.append(obj)
        req.scene.world.collision_objects.append(self.bottle_collision(self.bottle_pose()))
        self.apply_scene(req)

    def attach_bottle(self):
        # Planning representation only: Gazebo still uses friction, never attachment.
        pose = self.bottle_pose()
        tool = fk([self.positions[j] for j in ARM_JOINTS])
        pose.position.x -= tool[0]; pose.position.y -= tool[1]; pose.position.z -= tool[2]
        self.bottle_offset = [pose.position.x, pose.position.y, pose.position.z]
        q = pose.orientation
        cosine = max(-1, min(1, 1-2*(q.x*q.x+q.y*q.y)))
        self.bottle_half_extent = self.cfg['height_m']/2*abs(cosine)+self.cfg['diameter_m']/2*math.sqrt(max(0,1-cosine*cosine))
        attached = AttachedCollisionObject(); attached.link_name = 'tool_link'
        attached.touch_links = ['left_finger', 'right_finger']
        attached.object = self.bottle_collision(pose)
        attached.object.header.frame_id = 'tool_link'
        req = ApplyPlanningScene.Request(); req.scene.is_diff = True
        remove = CollisionObject(); remove.id = 'target_bottle'; remove.operation = CollisionObject.REMOVE
        if not self.attached:
            req.scene.world.collision_objects.append(remove)
        req.scene.robot_state.is_diff = True
        req.scene.robot_state.attached_collision_objects.append(attached)
        self.apply_scene(req)
        self.attached = True

    def detach_bottle(self):
        req = ApplyPlanningScene.Request(); req.scene.is_diff = True
        remove = AttachedCollisionObject(); remove.link_name = 'tool_link'
        remove.object.id = 'target_bottle'; remove.object.operation = CollisionObject.REMOVE
        req.scene.robot_state.is_diff = True
        req.scene.robot_state.attached_collision_objects.append(remove)
        req.scene.world.collision_objects.append(self.bottle_collision(self.bottle_pose()))
        self.apply_scene(req)
        self.attached = False

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
        if not all(math.isfinite(a) for a in actual) or max(abs(a-b) for a,b in zip(actual,q)) > 0.02:
            raise RuntimeError('Arm endpoint joint error too large')
        coupling = {
            'wrist_counter_shoulder': -actual[0],
            'endpoint_bracket_joint': -actual[1],
            'rod_counter_shoulder': actual[0],
            'rod_joint': actual[1],
        }
        if not all(j in self.positions and math.isfinite(self.positions[j]) for j in coupling):
            raise RuntimeError('Missing or nonfinite passive linkage feedback; task stopped')
        errors = {j: self.positions[j]-target for j,target in coupling.items()}
        if max(abs(e) for e in errors.values()) > 0.03:
            self.record('coupling_error', errors=errors)
            raise RuntimeError('Passive arm linkage tracking error; task stopped')
        self.record(name, joints=actual, trajectory_points=len(trajectory.joint_trajectory.points),
                    trajectory={'joint_names': list(names), 'points': [
                        {'positions': list(p.positions), 'velocities': list(p.velocities),
                         'accelerations': list(p.accelerations),
                         'time_from_start_s': p.time_from_start.sec+p.time_from_start.nanosec/1e9}
                        for p in trajectory.joint_trajectory.points]})

    def fingers(self, opening):
        goal=FollowJointTrajectory.Goal()
        goal.trajectory.joint_names=['left_finger_joint']
        p=JointTrajectoryPoint();p.positions=[opening];p.time_from_start.sec=2
        goal.trajectory.points=[p]
        handle=self.wait(self.gripper.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('Gripper goal rejected')
        try:
            result=self.wait(handle.get_result_async(), timeout=20).result
        except Exception:
            self.wait(handle.cancel_goal_async(), timeout=5)
            raise
        if result.error_code != 0:
            raise RuntimeError('Gripper trajectory failed: '+result.error_string)
        self.pause(0.7)
        self.record('gripper', commanded_half_gap=opening)

    def bottle_pose(self):
        req=GetEntityState.Request();req.name='target_bottle';req.reference_frame='world'
        result=self.wait(self.entity.call_async(req))
        if not result.success:
            raise RuntimeError('Gazebo cannot locate target_bottle')
        pose = result.state.pose
        values = [pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[3:])-1)>0.01:
            raise RuntimeError('Invalid bottle pose feedback; task stopped')
        return pose

    def bottle(self):
        pose = self.bottle_pose(); p = pose.position; q = pose.orientation
        tilt = math.acos(max(-1, min(1, 1-2*(q.x*q.x+q.y*q.y))))
        return [p.x, p.y, p.z], tilt

    def run(self):
        cfg, poses = self.cfg, self.targets
        self.ready()
        preflight, tilt = self.bottle()
        if math.dist(preflight,bottle_start(cfg))>0.012 or tilt>math.radians(10):
            raise RuntimeError('Bottle is not upright at pickup A; task stopped before motion')
        self.setup_scene();self.fingers(cfg['open_half_gap_m'])
        self.arm('home',HOME);self.arm('approach',poses['approach'])
        initial, tilt=self.bottle();self.record('bottle_initial',position=initial,tilt_rad=tilt)
        if math.dist(initial,bottle_start(cfg))>0.012 or tilt>math.radians(10):
            raise RuntimeError('Bottle is not upright at pickup A; no automatic reset is performed')
        self.arm('pick',poses['pick'])
        self.fingers(cfg['diameter_m']/2-cfg['grip_compression_m'])
        clamped, clamp_tilt = self.bottle()
        self.record('bottle_clamped', position=clamped, tilt_rad=clamp_tilt,
                    finger_positions={k:v for k,v in self.positions.items() if 'finger' in k})
        self.attach_bottle()
        self.arm('lift',poses['lift']);self.pause(1)
        lifted, tilt=self.bottle();lift=lifted[2]-initial[2]
        self.record('bottle_lifted',position=lifted,lift_m=lift,tilt_rad=tilt)
        if lift<0.025 or tilt>math.radians(15):
            raise RuntimeError('Physical grasp failed: insufficient lift or excessive bottle tilt; task stopped')
        self.attach_bottle()  # Refresh planning geometry after any settling in the jaws.
        self.arm('transfer',poses['transfer'])
        moved, tilt=self.bottle()
        if abs(moved[0]-cfg['place_x_m'])>0.02 or moved[2]-initial[2]<0.025 or tilt>math.radians(15):
            raise RuntimeError('Bottle slipped during transfer; task stopped')
        self.attach_bottle()
        # Compensate observed grip slip so the bottle is not driven into the floor.
        place = ik(cfg['place_x_m']-self.bottle_offset[0],
                   self.bottle_half_extent+cfg['release_clearance_m']-self.bottle_offset[2])
        self.record('placement_adjustment', bottle_offset=self.bottle_offset, joints=place)
        self.arm('place',place);self.fingers(cfg['open_half_gap_m'])
        self.pause(1);self.detach_bottle()
        self.arm('retract',poses['retract']);self.arm('home_final',HOME);self.pause(1)
        final, tilt=self.bottle()
        self.pause(1); settled, final_tilt=self.bottle(); drift=math.dist(final,settled)
        error=math.dist(settled,bottle_goal(cfg))
        self.record('bottle_final',position=settled,placement_error_m=error,
                    tilt_rad=final_tilt,settling_drift_m=drift)
        if error>0.012 or final_tilt>math.radians(10) or drift>0.003:
            raise RuntimeError('Bottle did not settle upright within 12 mm of placement B')
        return {'success':True,'lift_m':lift,'placement_error_m':error,
                'tilt_rad':final_tilt,'settling_drift_m':drift}



def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output', default='work/pick_result.json')
    parser.add_argument('--config', default=None)
    parser.add_argument('--cycles', type=int, default=1)
    args=parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])
    node = None; result = {}; rclpy.init()
    runs = []; resets = []
    try:
        if args.cycles < 1:
            raise ValueError('cycles must be positive')
        cfg = load_config(args.config)  # Validate before submitting any motion.
        node = Demo(cfg)
        node.record('configuration', config=cfg, total_mass_kg=bottle_mass(cfg),
                    notice='Rigid-body simulation does not validate PET deformation or EP payload')
        for cycle in range(args.cycles):
            node.record('cycle_start', cycle=cycle+1, direction='A_to_B')
            runs.append(node.run())
            if cycle+1 < args.cycles:
                # Return the bottle physically to A between scored A-to-B trials.
                node.cfg = dict(cfg, pick_x_m=cfg['place_x_m'], place_x_m=cfg['pick_x_m'])
                node.targets = targets(node.cfg)
                node.record('reset_start', direction='B_to_A')
                resets.append(node.run())
                node.cfg = cfg; node.targets = targets(cfg)
        result={'success':True, 'requested_cycles':args.cycles, 'runs':runs, 'resets':resets,
                'config':cfg, 'mass_kg':bottle_mass(cfg)}
    except Exception as error:
        result={'success':False,'error':str(error),'runs':runs,'resets':resets}
        print('FAILED:',error,flush=True)
    finally:
        result['events']=node.events if node else []
        output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
        if node:
            node.execute.destroy();node.gripper.destroy();node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if result.get('success') else 1)


if __name__=='__main__':
    main()
