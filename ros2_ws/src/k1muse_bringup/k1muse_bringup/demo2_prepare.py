"""One-shot shared Demo1/2 preparation; never submits a business goal."""
import argparse
import json
import math
import sys
import time
import uuid
from pathlib import Path

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from std_msgs.msg import String
from k1muse_interfaces.action import ArmCommand
from k1muse_interfaces.msg import ArmStatus, LiftStatus
from k1muse_interfaces.srv import WebRequest


def business_readiness(info, fetch_enabled, survey_enabled):
    """Use one existing inspect snapshot; keep each business independently usable."""
    survey = 'disabled'
    if survey_enabled:
        state = info.get('survey', {})
        survey = (state.get('admission_error') or
                  ('ready' if state.get('enabled') else 'survey_disabled'))
    drugs = [name for name, row in info.get('fetch_admission', {}).items()
             if row.get('ready')] if fetch_enabled and info.get('fetch_ready') else []
    fetch = ('ready' if info.get('fetch_ready') else info.get('fetch_blocked_reason') or 'no_fetch_targets') if fetch_enabled else 'disabled'
    return {'survey': survey, 'fetch': fetch, 'drugs': drugs}


class Preparation(Node):
    def __init__(self, config, enable_voice=False, prepare_survey=False):
        super().__init__('demo2_prepare')
        self.client_id = 'demo2-prepare-' + uuid.uuid4().hex
        self.token = ''
        self.states = {}
        self.subscriptions_ = [self.create_subscription(kind, topic,
            lambda msg, key=key: self.states.update({key: msg}), qos_profile_sensor_data)
            for key, kind, topic in [
                ('arm', ArmStatus, '/k1muse/arm/status'),
                ('lift', LiftStatus, '/k1muse/lift/status'),
                ('odom', Odometry, '/odom')]]
        self.web = self.create_client(WebRequest, '/k1muse/web/request')
        self.manager = self.create_client(WebRequest, '/k1muse/task/control')
        self.resources = self.create_client(WebRequest, '/k1muse/web/navigation/control')
        self.reset = self.create_client(Trigger, '/k1muse/chassis/reset_odometry')
        self.arm = ActionClient(self, ArmCommand, '/k1muse/arm/command')
        self.config = yaml.safe_load(Path(config).read_text(encoding='utf-8'))
        self.enable_voice = enable_voice
        self.prepare_survey = prepare_survey
        self.renew = None
        self.inference_state, self.intent_state = {}, {}
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_subscriptions = [self.create_subscription(String,
            '/k1muse/task/inference/status',
            lambda msg: self.inference_state.update(json.loads(msg.data)), status_qos)]
        if enable_voice:
            self.status_subscriptions.append(self.create_subscription(String,
                '/k1muse/voice/intent/status',
                lambda msg: self.intent_state.update(json.loads(msg.data)), status_qos))

    def wait(self, condition, seconds, label):
        end = time.monotonic() + seconds
        while rclpy.ok() and not condition():
            if time.monotonic() >= end:
                raise RuntimeError(label + ' timeout/unresolved; no automatic replay')
            rclpy.spin_once(self, timeout_sec=.05)
        if not rclpy.ok():
            raise RuntimeError('preparation interrupted')

    def response(self, future, seconds, label):
        self.wait(future.done, seconds, label)
        return future.result()

    def request(self, operation, arguments=None):
        req = WebRequest.Request()
        req.client_id, req.token = self.client_id, self.token
        req.request_id = uuid.uuid4().hex
        req.operation, req.arguments = operation, json.dumps(arguments or {})
        return req

    def rpc(self, client, operation, arguments=None):
        response = self.response(client.call_async(self.request(operation, arguments)), 16 if operation == 'set_navigation_compute' else 8, operation)
        if not response.accepted:
            raise RuntimeError(operation + ': ' + response.error)
        return response, json.loads(response.result or '{}')

    def operation(self, operation, arguments=None):
        response, result = self.rpc(self.web, operation, arguments)
        operation_id = response.operation_id
        end = time.monotonic() + 75
        while result.get('status') == 'running':
            if time.monotonic() > end:
                raise RuntimeError(operation + ' unresolved; inspect before restarting')
            # Startup-only polling of the existing single operation slot.
            until = time.monotonic() + .25
            self.wait(lambda: time.monotonic() >= until, 1, 'operation poll')
            _, result = self.rpc(self.web, 'get_operation', {'operation_id': operation_id})
        if result.get('status') != 'succeeded':
            raise RuntimeError(operation + ': ' + json.dumps(result, ensure_ascii=False))
        self.get_logger().info(operation + ' succeeded')
        return result.get('result') or {}

    def fresh(self, message, age=.5):
        stamp = message.header.stamp
        return 0 <= (self.get_clock().now().nanoseconds -
                     (stamp.sec * 1000000000 + stamp.nanosec)) * 1e-9 < age

    def idle(self):
        if len(self.states) != 3 or not all(self.fresh(m) for m in self.states.values()):
            return False
        arm, lift, odom = (self.states[k] for k in ('arm', 'lift', 'odom'))
        v = odom.twist.twist
        return (not arm.is_busy and not arm.has_fault and not lift.is_busy and
                not lift.has_fault and lift.is_homed and
                math.hypot(v.linear.x, v.linear.y) < .02 and abs(v.angular.z) < .05)

    def ready_pose(self):
        mode = self.config['grasp'].get('mode', 'front')
        if mode not in ('front', 'top'):
            raise RuntimeError('grasp.mode must be front or top')
        profile = self.config['grasp'][mode]
        if not self.prepare_survey and not profile['calibration_confirmed']:
            raise RuntimeError(f'{mode} grasp profile is not calibrated')
        pose = profile['poses']['ready']
        if len(pose) != 5 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose):
            raise RuntimeError('a measured ready pose is required')
        xyz = lambda: [self.states['arm'].current_x, self.states['arm'].current_y,
                       self.states['arm'].current_z]
        if math.dist(xyz(), pose[:3]) <= .01:
            return
        if not self.arm.wait_for_server(timeout_sec=5):
            raise RuntimeError('ARM Action unavailable')
        goal = ArmCommand.Goal(command=ArmCommand.Goal.MOVE_POSE,
                              x=float(pose[0]), y=float(pose[1]), z=float(pose[2]),
                              roll=float(pose[3]), pitch=float(pose[4]))
        sent = self.get_clock().now().nanoseconds
        handle = self.response(self.arm.send_goal_async(goal), 8, 'ARM admission')
        if not handle or not handle.accepted:
            raise RuntimeError('ARM ready pose rejected')
        reply = self.response(handle.get_result_async(), 65, 'ARM terminal')
        result = reply.result
        if reply.status != 4 or result.result != 3 or result.reason or result.fault_source:
            raise RuntimeError('ARM ready failed: ' + str(result))
        self.wait(lambda: self.idle() and
                  self.states['arm'].header.stamp.sec * 1000000000 +
                  self.states['arm'].header.stamp.nanosec >= sent, 8, 'ARM ready feedback')

    def inference_ready(self):
        state = self.inference_state
        if state.get('phase') == 'blocked':
            raise RuntimeError('inference blocked: ' + state.get('last_error', ''))
        phase = 'wake_wait' if self.enable_voice else 'idle'
        ready = state.get('phase') == phase and not state.get('work_id')
        if not self.enable_voice:
            return ready
        intent = self.intent_state
        return (ready and state.get('input_enabled') and
                intent.get('mode') in ('rules', 'local', 'cloud') and
                not intent.get('preparing') and not intent.get('restarting'))

    def run(self):
        for client in (self.web, self.manager):
            if not client.wait_for_service(timeout_sec=30):
                raise RuntimeError('required web/manager service unavailable')
        _, info = self.rpc(self.manager, 'inspect')
        if info['busy'] or info['motion_inhibited'] or info['ready']:
            raise RuntimeError('requires a fresh idle manager; stop previous Demo2 launch first')
        if not self.config.get('enabled', False) and not self.prepare_survey:
            raise RuntimeError('neither fetch nor survey is enabled')
        if (not info['warehouse']['loaded_map_id'] or info['warehouse']['load_error'] or
                (not self.prepare_survey and info['warehouse']['fetch_config_error'])):
            raise RuntimeError('warehouse/fetch configuration unavailable')
        # Existing manager initialization has 30s service discovery and up to
        # three 60s prepare stages. This subscription disappears with this helper.
        self.wait(self.inference_ready, 210, 'inference startup ready')
        map_id = info['warehouse']['loaded_map_id']
        _, claim = self.rpc(self.web, 'claim_control')
        self.token = claim['token']
        self.renew = self.create_timer(1, lambda: self.web.call_async(self.request('set_views')))
        self.rpc(self.web, 'set_teleop_enabled', {'enabled': False})
        self.operation('list_maps')
        # Start serial/odometry through their original owner before starting AMCL.
        self.operation('set_mode', {'mode': 'MANUAL'})
        self.wait(self.idle, 15, 'fresh idle/homed hardware')
        if not self.reset.wait_for_service(timeout_sec=5):
            raise RuntimeError('reset_odometry service missing; update communication package')
        sent = self.get_clock().now().nanoseconds
        reset = self.response(self.reset.call_async(Trigger.Request()), 5, 'ODOM reset')
        if not reset.success:
            raise RuntimeError(reset.message)

        def odom_reset_received():
            odom = self.states['odom']
            p, q = odom.pose.pose.position, odom.pose.pose.orientation
            stamp = odom.header.stamp.sec * 1000000000 + odom.header.stamp.nanosec
            return (self.idle() and stamp > sent and math.hypot(p.x, p.y) < .005 and
                    abs(2 * math.atan2(q.z, q.w)) < .01)
        self.wait(odom_reset_received, 5, 'post-reset zero odometry')
        self.operation('set_mode', {'mode': 'NAVIGATION', 'map_id': map_id})
        points = self.operation('get_points', {'map_id': map_id})['tasks']
        origin = points['建图起点']  # Use the saved pose, never assume old maps have zero origin.
        self.operation('set_initial_pose', origin)
        self.wait(self.idle, 8, 'idle hardware before ARM ready')
        self.ready_pose()
        self.rpc(self.manager, 'confirm_empty', {'confirmed': True})
        _, info = self.rpc(self.manager, 'inspect')
        businesses = business_readiness(info, self.config.get('enabled', False), self.prepare_survey)
        if businesses['survey'] != 'ready' and businesses['fetch'] != 'ready':
            raise RuntimeError('no business ready: ' + json.dumps(businesses, ensure_ascii=False))
        self.rpc(self.resources, 'set_navigation_compute', {
            'manager_instance': info['manager_instance'], 'context_id': info['context_id'],
            'business_id': '', 'state': 'parked'})
        # Root-owned, fixed board helper; no shell or persistent polling.
        irq_helper = Path('/usr/local/sbin/k1-resource-affinity')
        if irq_helper.is_file():
            import subprocess
            subprocess.run(['sudo', '-n', str(irq_helper)], check=True, timeout=10,
                           stdout=subprocess.DEVNULL)
        self.wait(self.inference_ready, 5, 'inference ready before handoff')
        voice = 'no voice; inference idle'
        if self.enable_voice:
            actual = self.intent_state['mode']
            requested = self.intent_state.get('active_startup_mode', actual)
            voice = 'voice wake_wait; mode=' + actual + '; requested=' + requested
            if actual != requested:
                self.get_logger().warning('voice backend fallback: requested=' + requested +
                                          ' actual=' + actual)
        self.get_logger().info('DEMO READY: ' + voice +
                               '; survey=' + businesses['survey'] +
                               '; fetch=' + businesses['fetch'] +
                               '; drugs=' + ','.join(businesses['drugs']) +
                               '; use voice or task_debug.py fetch/survey [--drug <规范药名>]')

    def close(self):
        if self.renew:
            self.renew.cancel()
        if self.token and rclpy.ok():
            try:
                self.rpc(self.web, 'release_control')
            except Exception as error:
                self.get_logger().warning(str(error))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fetch-config', required=True)
    parser.add_argument('--enable-voice', action='store_true')
    parser.add_argument('--prepare-survey', action='store_true')
    parser.add_argument('--start-at-origin-empty', action='store_true', required=True,
                        help='Confirm saved origin/heading, empty gripper and clear ARM workspace')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = None
    try:
        node = Preparation(args.fetch_config, args.enable_voice, args.prepare_survey)
        node.run()
        return 0
    except (Exception, KeyboardInterrupt) as error:
        print(str(error) + '; no fetch sent; no automatic retry', file=sys.stderr)
        return 1
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
