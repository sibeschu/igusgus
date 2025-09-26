#!/usr/bin/env python3
import rclpy
import numpy as np

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint, BoundingVolume,
    MotionPlanRequest, PlanningOptions
)
from std_srvs.srv import Trigger

PLANNING_GROUP = "igus_rebel_arm"     # SRDF group
ENDEFFECTOR_LINK = "link6"                # end-effector link
PLANNING_FRAME = "world"         # usually base link
POSE_TOLERANCE_POSITION_MM = 0.005                # 5 mm positional tolerance
POSE_TOLERANCE_ROTATION_RAD = 0.02               # ~1.1 deg per axis
PRE_POSE_SPHERE_RADIUS = 0.05

TARGET_XYZ = (0.4, 0.0, 0.40)      # goal position in PLANNING_FRAME
TARGET_QWXYZ = (0.0, 0.99, 0.0, 0.0)   # goal orientation as (w,x,y,z)

class MoveGroupPoseClient(Node):
    def __init__(self):
        super().__init__("ctrl_rob")
        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.sphere_radius_m = PRE_POSE_SPHERE_RADIUS

        self.client = ActionClient(self, MoveGroup, "/move_action", callback_group=self.callback_group)

        # latest tool pose cache
        self.latest_tool_pose: PoseStamped | None = None

        # subs
        self.tool_pose_subscriber = self.create_subscription(
            PoseStamped, "/tool_coord", self._on_tool_pose, 10
        )

        # service
        self.go_to_tool_service = self.create_service(
            Trigger, "/go_to_tool", self._on_go_to_tool, callback_group=self.callback_group
        )

        self.go_to_default_service = self.create_service(
            Trigger, "/go_to_default", self._on_go_to_default, callback_group=self.callback_group
        )


    def _on_go_to_default(self, _req, _res):
        res = Trigger.Response()

        # Build the default target pose
        default_pose = PoseStamped()
        default_pose.header.frame_id = PLANNING_FRAME
        default_pose.pose.position.x, default_pose.pose.position.y, default_pose.pose.position.z = TARGET_XYZ
        qw, qx, qy, qz = TARGET_QWXYZ
        default_pose.pose.orientation.w = qw
        default_pose.pose.orientation.x = qx
        default_pose.pose.orientation.y = qy
        default_pose.pose.orientation.z = qz

        # Build constraints and motion request
        goal_constraints = self._make_pose_goal_constraints(default_pose)
        motion_request = MotionPlanRequest()
        motion_request.group_name = PLANNING_GROUP
        motion_request.goal_constraints = [goal_constraints]
        planning_options = PlanningOptions(plan_only=False)

        move_group_goal = MoveGroup.Goal(
            request=motion_request,
            planning_options=planning_options
        )

        # Send to move_group action server
        if not self.client.wait_for_server(timeout_sec=2.0):
            res.success = False
            res.message = "MoveGroup action server (/move_action) not available."
            return res

        send_future = self.client.send_goal_async(move_group_goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            res.success = False
            res.message = "MoveGroup goal rejected."
            return res

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        res.success = (result.error_code.val == 1)  # 1 == SUCCESS
        res.message = f"MoveGroup result code: {result.error_code.val}"
        return res


    def _on_tool_pose(self, msg: PoseStamped):
        self.latest_tool_pose = msg

    def _on_go_to_tool(self, _req, _res):
        res = Trigger.Response()
        if self.latest_tool_pose is None:
            res.success = False
            res.message = "No tool pose received on /tool_coord."
            return res

        tool_pose = self.latest_tool_pose
        if tool_pose.header.frame_id != PLANNING_FRAME:
            tf = self.tf_buffer.lookup_transform(
                PLANNING_FRAME, tool_pose.header.frame_id, rclpy.time.Time(), timeout=Duration(seconds=0.5)
            )
            tool_pose = do_transform_pose(tool_pose, tf)

        pregrasp_pose = self._compute_spherical_pregrasp(tool_pose, self.sphere_radius_m)

        goal_constraints = self._make_pose_goal_constraints(pregrasp_pose)
        motion_request = MotionPlanRequest()
        motion_request.group_name = PLANNING_GROUP
        motion_request.goal_constraints = [goal_constraints]

        planning_options = PlanningOptions(plan_only=False)  # plan+execute on server

        move_group_goal = MoveGroup.Goal(
            request=motion_request,
            planning_options=planning_options
        )

        if not self.client.wait_for_server(timeout_sec=2.0):
            res.success = False
            res.message = "MoveGroup action server (/move_action) not available."
            return res

        send_future = self.client.send_goal_async(move_group_goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            res.success = False
            res.message = "MoveGroup goal rejected."
            return res

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        # SUCCESS = 1 per moveit_msgs/MoveItErrorCodes
        ok = (result.error_code.val == 1)

        res.success = ok
        res.message = f"MoveGroup result code: {result.error_code.val}"
        return res

    def _lookup_current_ee_pose(self) -> PoseStamped:
        tf = self.tf_buffer.lookup_transform(
            PLANNING_FRAME, ENDEFFECTOR_LINK, rclpy.time.Time(), timeout=Duration(seconds=0.5)
        )
        ee = PoseStamped()
        ee.header.frame_id = PLANNING_FRAME
        ee.pose.position.x = tf.transform.translation.x
        ee.pose.position.y = tf.transform.translation.y
        ee.pose.position.z = tf.transform.translation.z
        ee.pose.orientation = tf.transform.rotation
        return ee

    def _compute_spherical_pregrasp(self, tool_pose: PoseStamped, radius_m: float) -> PoseStamped:
        ee_pose = self._lookup_current_ee_pose()
        p_ee = np.array([ee_pose.pose.position.x, ee_pose.pose.position.y, ee_pose.pose.position.z], dtype=float)
        p_tool = np.array([tool_pose.pose.position.x, tool_pose.pose.position.y, tool_pose.pose.position.z], dtype=float)

        v = p_tool - p_ee
        dist = np.linalg.norm(v)
        if dist < 1e-6:
            d_hat = np.array([0.0, 0.0, 1.0])  # fallback direction
            dist = 1.0
        else:
            d_hat = v / dist

        if dist <= radius_m + 1e-3:
            stop_dist = max(0.0, dist * 0.5)   # conservative if already inside the sphere
        else:
            stop_dist = radius_m

        p_pre = p_tool - stop_dist * d_hat

        pre = PoseStamped()
        pre.header.frame_id = tool_pose.header.frame_id
        pre.pose.position.x, pre.pose.position.y, pre.pose.position.z = p_pre.tolist()
        # keep current EE orientation for stability; change if you prefer tool orientation
        pre.pose.orientation = ee_pose.pose.orientation
        return pre


    def _make_pose_goal_constraints(self, pose_stamped: PoseStamped) -> Constraints:
        # Position constraint: sphere of radius POSE_TOLERANCE_POSITION_MM centered at target
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[POSE_TOLERANCE_POSITION_MM])
        region_pose = pose_stamped.pose  # center at target pose
        position_bounding_volume = BoundingVolume(primitives=[sphere], primitive_poses=[region_pose])
        position_constraint = PositionConstraint()
        position_constraint.header = pose_stamped.header
        position_constraint.link_name = ENDEFFECTOR_LINK
        position_constraint.constraint_region = position_bounding_volume
        position_constraint.weight = 1.0

        # Orientation constraint: absolute XYZ-angle tolerances around target orientation
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose_stamped.header
        orientation_constraint.link_name = ENDEFFECTOR_LINK
        orientation_constraint.orientation = pose_stamped.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = POSE_TOLERANCE_ROTATION_RAD
        orientation_constraint.absolute_y_axis_tolerance = POSE_TOLERANCE_ROTATION_RAD
        orientation_constraint.absolute_z_axis_tolerance = POSE_TOLERANCE_ROTATION_RAD
        orientation_constraint.weight = 1.0

        return Constraints(position_constraints=[position_constraint], orientation_constraints=[orientation_constraint])

    def send_pose_goal(self):
        # 1) Build target pose in planning frame
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = PLANNING_FRAME
        goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z = TARGET_XYZ
        qw, qx, qy, qz = TARGET_QWXYZ
        goal_pose.pose.orientation.w = qw; goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy; goal_pose.pose.orientation.z = qz

        # 2) Convert to MoveIt constraints
        goal_constraints = self._make_pose_goal_constraints(goal_pose)

        # 3) Form MotionPlanRequest (plan+execute on server side)
        req = MotionPlanRequest()
        req.group_name = PLANNING_GROUP
        req.goal_constraints = [goal_constraints]

        options = PlanningOptions(plan_only=False)  # set True for plan-only
        goal = MoveGroup.Goal(request=req, planning_options=options)

        # 4) Send to /move_action (move_group)
        self.client.wait_for_server()
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        result_future = send_future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f"MoveGroup result: {result_future.result()}")

def main():
    rclpy.init()
    node = MoveGroupPoseClient()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
