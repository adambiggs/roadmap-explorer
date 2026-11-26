import contextlib
import os
import signal
import subprocess
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

import domain_coordinator
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from roadmap_explorer_msgs.action import Explore
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class TestExploreCancel(unittest.TestCase):
    def setUp(self):
        settings = contextlib.ExitStack()
        self.addCleanup(settings.close)
        domain = settings.enter_context(domain_coordinator.domain_id())
        settings.enter_context(patch.dict(os.environ, {
            "ROS_DOMAIN_ID": str(domain), "ROS_LOCALHOST_ONLY": "1",
        }))
        package = "roadmap_explorer"
        share = Path(get_package_share_directory(package))
        executable = Path(get_package_prefix(package), "lib", package, "roadmap_exploration_server")
        self.explorer = subprocess.Popen([
            str(executable), "--ros-args", "--params-file", str(share / "params/exploration_params.yaml"),
            "-p", "localisation_only_mode:=false", "-p", "explorationBT.bt_sleep_ms:=20",
        ])
        self.addCleanup(self.stop_explorer)
        rclpy.init()
        self.node = rclpy.create_node("explore_cancel_test")
        self.planner = rclpy.create_node("planner_server")
        self.planner.declare_parameter("GridBased.allow_unknown", True)
        self.reject_allow_unknown = False
        self.planner.add_on_set_parameters_callback(
            lambda _: SetParametersResult(successful=not self.reject_allow_unknown))
        self.costmap = rclpy.create_node("global_costmap", namespace="/global_costmap")
        self.costmap.declare_parameter("robot_radius", 0.1)
        self.parent = ActionClient(self.node, Explore, "roadmap_explorer")
        self.child_requested = threading.Event()
        self.release_response = threading.Event()
        self.child_cancelled = threading.Event()
        self.release_result = threading.Event()
        self.child_terminal = threading.Event()
        self.accept = True
        self.child = ActionServer(
            self.node, NavigateToPose, "navigate_to_pose", self.execute_child,
            goal_callback=self.child_request, cancel_callback=self.child_cancel,
            callback_group=ReentrantCallbackGroup(),
        )
        self.executor = MultiThreadedExecutor(num_threads=4)
        for node in (self.node, self.planner, self.costmap):
            self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin)
        self.thread.start()
        self.addCleanup(self.stop_fixture)

        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        transform.transform.rotation.w = 1.0
        self.tf = StaticTransformBroadcaster(self.node)
        self.tf.sendTransform(transform)

        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.height = grid.info.width = 160
        grid.info.resolution = 0.05
        grid.info.origin.position.x = grid.info.origin.position.y = -4.0
        grid.info.origin.orientation.w = 1.0
        # A known room opening onto unexplored space to the east.
        grid.data = [(0 if col < 108 else -1) for _ in range(160) for col in range(160)]
        self.map_pub = self.node.create_publisher(OccupancyGrid, "explored_map", 1)
        self.map_timer = self.node.create_timer(0.2, lambda: self.map_pub.publish(grid))

        state = self.node.create_client(ChangeState, "/roadmap_explorer_node/change_state")
        self.assertTrue(state.wait_for_service(timeout_sec=10))
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request()
            request.transition.id = transition
            self.assertTrue(self.await_result(state.call_async(request)).success)
        self.assertTrue(self.parent.wait_for_server(timeout_sec=10))

    def stop_explorer(self):
        self.explorer.send_signal(signal.SIGINT)
        try:
            self.explorer.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.explorer.kill()
            self.explorer.wait()

    def stop_fixture(self):
        self.release_response.set()
        self.release_result.set()
        self.executor.shutdown()
        self.thread.join()
        self.child.destroy()
        for node in (self.costmap, self.planner, self.node):
            node.destroy_node()
        rclpy.shutdown()

    def await_result(self, future):
        ready = threading.Event()
        future.add_done_callback(lambda _: ready.set())
        self.assertTrue(ready.wait(10), "ROS response timed out")
        return future.result()

    def child_request(self, goal):
        self.child_requested.set()
        if not self.release_response.wait(15):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT

    def child_cancel(self, goal):
        self.cancel_id = bytes(goal.goal_id.uuid)
        self.child_cancelled.set()
        return CancelResponse.ACCEPT

    def execute_child(self, goal):
        if self.release_result.wait(15) and goal.is_cancel_requested:
            goal.canceled()
        else:
            goal.abort()
        self.child_id = bytes(goal.goal_id.uuid)
        self.child_terminal.set()
        return NavigateToPose.Result()

    def cancel_while_child_is_pending(self, accept):
        self.accept = accept
        parent = self.await_result(self.parent.send_goal_async(Explore.Goal()))
        self.assertTrue(parent.accepted)
        result = parent.get_result_async()
        parent_terminal = threading.Event()
        result.add_done_callback(lambda _: parent_terminal.set())
        self.assertTrue(self.child_requested.wait(20), "no NavigateToPose request")
        cancel = self.await_result(parent.cancel_goal_async())
        self.assertEqual([bytes(g.goal_id.uuid) for g in cancel.goals_canceling],
                         [bytes(parent.goal_id.uuid)])
        self.assert_parent_pending(result, parent_terminal, "child response")
        self.release_response.set()
        return result, parent_terminal

    def assert_parent_pending(self, result, parent_terminal, before):
        if parent_terminal.wait(0.5):
            self.fail(f"Explore returned status {result.result().status} before {before}")

    def test_parent_waits_for_accepted_child_result(self):
        result, parent_terminal = self.cancel_while_child_is_pending(accept=True)
        self.assertTrue(self.child_cancelled.wait(5), "child was never cancelled")
        self.assert_parent_pending(result, parent_terminal, "child terminal result")
        self.release_result.set()
        self.assertEqual(self.await_result(result).status, GoalStatus.STATUS_CANCELED)
        self.assertTrue(self.child_terminal.is_set())
        self.assertEqual(self.cancel_id, self.child_id)

    def test_rejected_child_releases_parent(self):
        result, _ = self.cancel_while_child_is_pending(accept=False)
        self.assertEqual(self.await_result(result).status, GoalStatus.STATUS_CANCELED)
        self.assertFalse(self.child_cancelled.is_set())
        self.assertFalse(self.child_terminal.is_set())

    def test_failed_initialization_aborts_and_releases_the_server(self):
        self.reject_allow_unknown = True
        failed = self.await_result(self.parent.send_goal_async(Explore.Goal()))
        self.assertTrue(failed.accepted)
        self.assertEqual(self.await_result(failed.get_result_async()).status,
                         GoalStatus.STATUS_ABORTED)
        self.reject_allow_unknown = False
        retry = self.await_result(self.parent.send_goal_async(Explore.Goal()))
        self.assertTrue(retry.accepted)
        self.assertTrue(self.child_requested.wait(20), "retry never initialized")

    def test_second_parent_is_rejected_until_the_first_ends(self):
        first = self.await_result(self.parent.send_goal_async(Explore.Goal()))
        self.assertTrue(first.accepted)
        first_result = first.get_result_async()
        first_terminal = threading.Event()
        first_result.add_done_callback(lambda _: first_terminal.set())
        self.assertTrue(self.child_requested.wait(20), "no NavigateToPose request")
        other = ActionClient(self.node, Explore, "roadmap_explorer")
        self.assertTrue(other.wait_for_server(timeout_sec=10))
        self.assertFalse(self.await_result(other.send_goal_async(Explore.Goal())).accepted)
        self.assert_parent_pending(first_result, first_terminal, "its own cancel")
        self.await_result(first.cancel_goal_async())
        self.release_response.set()
        self.release_result.set()
        self.assertEqual(self.await_result(first_result).status, GoalStatus.STATUS_CANCELED)
        self.assertTrue(self.await_result(other.send_goal_async(Explore.Goal())).accepted)
