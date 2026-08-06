#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Timed Explore Guard

Runs alongside explore_lite: tracks block detections, auto-stops
exploration after a configurable duration and returns to a fixed home position.
"""

import json
import math
import subprocess
import sys
import os
import threading

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

try:
    import actionlib
except ImportError:
    actionlib = None

try:
    import tf
except ImportError:
    tf = None


class TimedExploreGuard:
    def __init__(self):
        rospy.init_node("timed_explore_guard")

        # Config
        self.duration = float(rospy.get_param("~duration", 180.0))
        self.detection_topic = rospy.get_param("~detection_topic",
                                               "/coverage_block_search/detection")
        self.goal_topic = rospy.get_param("~goal_topic",
                                          "/move_base_simple/goal")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.explore_node = rospy.get_param("~explore_node", "/explore")

        # Fixed home position (no TF recording needed)
        self.home_x = float(rospy.get_param("~home_x", 0.0))
        self.home_y = float(rospy.get_param("~home_y", 0.0))
        self.home_yaw = float(rospy.get_param("~home_yaw", 0.0))

        # State
        self.found_blocks = []
        self.start_time = None

        # Publishers / Subscribers
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=True)
        rospy.Subscriber(self.detection_topic, String, self.detection_callback, queue_size=10)

        if tf is not None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = None

        rospy.loginfo("Timed explore guard ready: duration=%.0fs home=(%.2f, %.2f, %.1fdeg)",
                      self.duration, self.home_x, self.home_y, math.degrees(self.home_yaw))

    # ------------------------------------------------------------------
    # Voice
    # ------------------------------------------------------------------

    COLOR_CN = {
        "red":    "红色",
        "blue":   "蓝色",
        "green":  "绿色",
        "yellow": "黄色",
        "orange": "橙色",
        "purple": "紫色",
        "white":  "白色",
        "black":  "黑色",
    }

    def speak(self, text):
        """Async TTS via spd-say."""
        def _run():
            try:
                subprocess.run(
                    ["spd-say", "-l", "zh", "--", text],
                    timeout=10,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except Exception:
                pass
        t = threading.Thread(target=_run, daemon=True)
        t.start()

    # ------------------------------------------------------------------
    # Robot position (for block dedup)
    # ------------------------------------------------------------------

    def _get_robot_xy(self):
        """Get current robot x,y in map frame. Returns (0,0) on failure."""
        if self.tf_listener is None:
            return 0.0, 0.0
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(
                self.map_frame, self.base_frame, now, rospy.Duration(0.2))
            trans, _ = self.tf_listener.lookupTransform(
                self.map_frame, self.base_frame, now)
            return trans[0], trans[1]
        except Exception:
            return 0.0, 0.0

    # ------------------------------------------------------------------
    # Block detection
    # ------------------------------------------------------------------

    def detection_callback(self, msg):
        try:
            det = json.loads(msg.data)
        except ValueError:
            return

        color = det.get("color", "unknown")
        rx, ry = self._get_robot_xy()

        # Dedup: same color + robot nearby last sighting → same block
        min_dist = rospy.get_param("~block_dedup_distance", 1.0)
        for block in self.found_blocks:
            if block["color"] != color:
                continue
            bx = block.get("robot_x", float("inf"))
            by = block.get("robot_y", float("inf"))
            if math.hypot(rx - bx, ry - by) < min_dist:
                rospy.logdebug("Dedup %s: robot at (%.2f,%.2f) near previous (%.2f,%.2f)",
                               color, rx, ry, bx, by)
                return

        block = {
            "id": len(self.found_blocks) + 1,
            "color": color,
            "area": float(det.get("area", 0.0)),
            "center_x": float(det.get("center_x", 0.0)),
            "center_y": float(det.get("center_y", 0.0)),
            "robot_x": rx,
            "robot_y": ry,
            "stamp": rospy.Time.now().to_sec(),
        }
        self.found_blocks.append(block)
        rospy.loginfo("FOUND block #%d color=%s area=%.0f pixel=(%d,%d) robot=(%.2f,%.2f)",
                      block["id"], block["color"], block["area"],
                      int(block["center_x"]), int(block["center_y"]),
                      rx, ry)

        color_cn = self.COLOR_CN.get(block["color"], block["color"])
        self.speak("发现%s方块" % color_cn)

    # ------------------------------------------------------------------
    # Return to home
    # ------------------------------------------------------------------

    def return_to_home(self):
        """Return to fixed home position using move_base action client."""
        x, y, yaw = self.home_x, self.home_y, self.home_yaw

        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = self.map_frame
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        rospy.loginfo("Return home: x=%.3f y=%.3f yaw=%.1fdeg",
                      x, y, math.degrees(yaw))

        # Clear costmaps
        rospy.loginfo("Clearing costmaps before return...")
        try:
            import rosservice
            rosservice.call_service('/move_base/clear_costmaps', [])
        except Exception as exc:
            rospy.logwarn("Could not clear costmaps: %s (continuing anyway)", exc)

        # Publish latched goal as backup
        self.goal_pub.publish(goal_pose)
        rospy.sleep(0.5)

        if actionlib is None:
            rospy.logwarn("actionlib not available; using topic goal only")
            return True

        action_name = rospy.get_param("~move_base_action", "move_base")
        max_retries = 3
        for attempt in range(max_retries):
            try:
                client = actionlib.SimpleActionClient(action_name, MoveBaseAction)
                if not client.wait_for_server(rospy.Duration(5.0)):
                    rospy.logwarn("move_base action server not available (attempt %d/%d)",
                                  attempt + 1, max_retries)
                    rospy.sleep(1.0)
                    continue

                client.cancel_all_goals()
                rospy.sleep(0.5)

                goal = MoveBaseGoal()
                goal.target_pose = goal_pose
                client.send_goal(goal)

                rospy.loginfo("Waiting for return (attempt %d/%d, max 120s)...",
                              attempt + 1, max_retries)
                finished = client.wait_for_result(rospy.Duration(120.0))

                if finished:
                    state = client.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Return to home succeeded!")
                        return True
                    else:
                        state_names = {0: "PENDING", 1: "ACTIVE", 2: "PREEMPTED",
                                       3: "SUCCEEDED", 4: "ABORTED", 5: "REJECTED",
                                       6: "PREEMPTING", 7: "RECALLING", 8: "RECALLED",
                                       9: "LOST"}
                        rospy.logwarn("Return attempt %d/%d: state=%d (%s)",
                                      attempt + 1, max_retries, state,
                                      state_names.get(state, "UNKNOWN"))
                else:
                    client.cancel_goal()
                    rospy.logwarn("Return attempt %d/%d timed out", attempt + 1, max_retries)

            except Exception as exc:
                rospy.logwarn("Return attempt %d/%d exception: %s", attempt + 1, max_retries, exc)

            if attempt < max_retries - 1:
                rospy.loginfo("Clearing costmaps and retrying in 2s...")
                try:
                    import rosservice
                    rosservice.call_service('/move_base/clear_costmaps', [])
                except Exception:
                    pass
                rospy.sleep(2.0)

        rospy.logerr("Return failed after %d attempts", max_retries)
        return False

    # ------------------------------------------------------------------
    # Main
    # ------------------------------------------------------------------

    def run(self):
        self.start_time = rospy.Time.now()

        rate = rospy.Rate(10)
        printed_warning = False

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - self.start_time).to_sec()

            if elapsed > self.duration - 30 and not printed_warning:
                rospy.logwarn("30 seconds remaining before return")
                printed_warning = True

            if elapsed >= self.duration:
                break

            rate.sleep()

        rospy.loginfo("Exploration time expired (%.0fs). Stopping explore and returning.",
                      self.duration)

        # Kill explore_lite
        try:
            rospy.loginfo("Killing node: %s", self.explore_node)
            os.system("rosnode kill %s 2>/dev/null" % self.explore_node)
        except Exception as exc:
            rospy.logwarn("Could not kill explore node: %s", exc)

        rospy.sleep(1.0)

        # Return to fixed home
        self.return_to_home()

        # Print summary
        rospy.loginfo("=== Exploration Summary ===")
        rospy.loginfo("Duration: %.0fs", self.duration)
        rospy.loginfo("Blocks found: %d", len(self.found_blocks))
        for block in self.found_blocks:
            rospy.loginfo("  #%d %s area=%.0f pixel=(%d,%d)",
                          block["id"], block["color"], block["area"],
                          int(block["center_x"]), int(block["center_y"]))


def main():
    node = TimedExploreGuard()
    node.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
