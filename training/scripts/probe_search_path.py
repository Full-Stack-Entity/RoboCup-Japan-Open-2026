"""Bounded ComputePathToPose probe. No NavigateToPose or velocity publishers."""
import argparse
import json
import math
from pathlib import Path
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    for k in ('x','y','yaw'):p.add_argument('--'+k,type=float,required=True)
    a=p.parse_args()
    if not all(math.isfinite(v) for v in (a.x,a.y,a.yaw)):p.error('finite pose required')
    import rclpy
    from rclpy.action import ActionClient
    from nav2_msgs.action import ComputePathToPose
    with a.output.open('x') as stream:
        rclpy.init();node=rclpy.create_node('handyman_path_probe_readonly')
        client=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
        handle=None;report=dict(success=False,robot_control_started=False)
        def wait(future,seconds):
            end=time.monotonic()+seconds
            while rclpy.ok() and not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
            if not future.done():raise TimeoutError('planning request timed out')
            return future.result()
        try:
            if not client.wait_for_server(timeout_sec=5.):raise RuntimeError('planner unavailable')
            goal=ComputePathToPose.Goal();goal.use_start=False;goal.planner_id='GridBased'
            goal.goal.header.frame_id='map';goal.goal.header.stamp=node.get_clock().now().to_msg()
            goal.goal.pose.position.x=a.x;goal.goal.pose.position.y=a.y
            goal.goal.pose.orientation.z=math.sin(a.yaw/2);goal.goal.pose.orientation.w=math.cos(a.yaw/2)
            handle=wait(client.send_goal_async(goal),5.)
            if not handle.accepted:raise RuntimeError('planning rejected')
            result=wait(handle.get_result_async(),10.)
            path=result.result.path;points=[[v.pose.position.x,v.pose.position.y] for v in path.poses]
            report.update(action_status=result.status,frame_id=path.header.frame_id,path_xy=points,
                path_length_m=sum(math.dist(x,y) for x,y in zip(points,points[1:])),
                endpoint_error_m=math.dist(points[-1],[a.x,a.y]) if points else None,
                success=result.status==4 and bool(points) and path.header.frame_id=='map',
                target=dict(x=a.x,y=a.y,yaw=a.yaw),start_source='current_robot_tf',
                scope='global static/inflation costmap only; no local dynamic obstacle or visibility proof')
        except Exception as exc:
            report['error']=str(exc)
            if handle is not None and handle.accepted:
                try:wait(handle.cancel_goal_async(),2.)
                except Exception:pass
        finally:
            json.dump(report,stream,indent=2,allow_nan=False)
            print(json.dumps({k:v for k,v in report.items() if k!='path_xy'}))
            client.destroy();node.destroy_node();rclpy.try_shutdown()
        if not report['success']:raise SystemExit(1)


if __name__=='__main__':main()
