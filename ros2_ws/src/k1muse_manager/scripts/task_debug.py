#!/usr/bin/env python3
"""One-shot manager debugger. Never publishes motion or actuator commands."""
import argparse
import json
import sys
import time
import uuid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from k1muse_interfaces.action import FetchDrug, SurveyWarehouse
from k1muse_interfaces.srv import WebRequest
from unique_identifier_msgs.msg import UUID


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=['inspect', 'preview', 'reload', 'confirm_empty',
                                            'pause', 'next', 'continue', 'cancel', 'breakpoints', 'fetch',
                                            'survey', 'inventory_get', 'inventory_set'])
    parser.add_argument('--drug', default=None, help='One canonical drug; omitted for whole-warehouse survey')
    parser.add_argument('--map-id', default='')
    parser.add_argument('--shelf', default='')
    parser.add_argument('--layer', type=int)
    parser.add_argument('--quantity', type=int)
    parser.add_argument('--confirmed', action='store_true')
    parser.add_argument('--steps', nargs='*', default=[])
    args, ros_args = parser.parse_known_args()
    if args.drug is not None and not args.drug.strip():
        parser.error('--drug must be a nonblank canonical drug name')
    rclpy.init(args=ros_args)
    node = Node('task_debug')
    try:
        if args.operation in ('fetch', 'survey'):
            survey = args.operation == 'survey'
            if not survey and not args.drug:
                parser.error('fetch requires --drug with one canonical drug name')
            quantity = args.quantity if args.quantity is not None else 1
            if not survey and not 1 <= quantity <= 3:
                parser.error('fetch --quantity must be 1..3')
            action = SurveyWarehouse if survey else FetchDrug
            client = ActionClient(node, action, '/k1muse/task/survey' if survey else '/k1muse/task/fetch_drug')
            if not client.wait_for_server(timeout_sec=3):
                raise RuntimeError(action.__name__ + ' unavailable')
            goal = (SurveyWarehouse.Goal(map_id=args.map_id, drug_name=args.drug or '')
                    if survey else FetchDrug.Goal(drug_name=args.drug, quantity=quantity))
            goal_id = uuid.uuid4()
            print('goal_id=' + goal_id.hex, flush=True)
            started = time.monotonic()
            def feedback(message):
                f = message.feedback
                print(json.dumps({'stage': f.stage, 'shelf_id': f.shelf_id, 'layer': f.layer,
                                  'direction': f.direction, 'attempt': f.attempt,
                                  'provisional_count': f.provisional_count}, ensure_ascii=False)
                      if survey else json.dumps({'stage': f.stage, 'planned_quantity': f.planned_quantity,
                                               'fetched_quantity': f.fetched_quantity}), flush=True)
            future = client.send_goal_async(goal, goal_uuid=UUID(uuid=list(goal_id.bytes)),
                                            feedback_callback=feedback)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if not future.done():
                raise RuntimeError(action.__name__ + ' admission unresolved; goal_id=' + goal_id.hex + '; inspect before retrying')
            if not future.result() or not future.result().accepted:
                raise RuntimeError(action.__name__ + ' rejected; run task_debug.py inspect and check the rejection reason')
            print('admission_ms=%.1f' % ((time.monotonic() - started) * 1000), flush=True)
            result = future.result().get_result_async()
            rclpy.spin_until_future_complete(node, result)
            reply = result.result()
            summary = {'status': reply.status, 'message': reply.result.message}
            if survey:
                summary['summary'] = json.loads(reply.result.summary_json or '{}')
            else:
                summary.update(requested_quantity=reply.result.requested_quantity,
                               planned_quantity=reply.result.planned_quantity, fetched_quantity=reply.result.fetched_quantity)
            print(json.dumps(summary, ensure_ascii=False))
            return 0 if reply.status == GoalStatus.STATUS_SUCCEEDED else 1
        else:
            client = node.create_client(WebRequest, '/k1muse/task/control')
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError('Task control unavailable; start manager with fetch_config_file')
            req = WebRequest.Request()
            req.client_id = 'task_debug'
            req.request_id = str(time.monotonic_ns())
            req.operation = args.operation
            arguments = {'confirmed': args.confirmed, 'steps': args.steps}
            if args.drug:
                arguments['drug_name'] = args.drug
            if args.operation == 'preview' and args.quantity is not None:
                arguments['quantity'] = args.quantity
            if args.operation.startswith('inventory_'):
                arguments = {'map_id': args.map_id}
                if args.operation == 'inventory_set':
                    if not args.shelf or args.layer is None or args.quantity is None or not args.drug:
                        parser.error('inventory_set requires --map-id --shelf --layer --drug --quantity')
                    arguments.update(shelf_id=args.shelf, layer=args.layer, drug=args.drug, quantity=args.quantity)
            req.arguments = json.dumps(arguments)
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if not future.done():
                raise RuntimeError('Control acknowledgement timeout; inspect before repeating an operation')
            reply = future.result()
            print(json.dumps({'accepted': reply.accepted, 'error': reply.error,
                              'result': json.loads(reply.result or '{}')}, ensure_ascii=False, indent=2))
            return 0 if reply.accepted else 1
    except KeyboardInterrupt:
        print('Client stopped. Robot task may still be running; use cancel/inspect.', file=sys.stderr)
        return 130
    except Exception as error:
        print(str(error), file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
