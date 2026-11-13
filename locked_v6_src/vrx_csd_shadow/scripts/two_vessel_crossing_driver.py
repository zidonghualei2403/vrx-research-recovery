#!/usr/bin/python3
import csv
import math
import os
from dataclasses import dataclass, field

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


@dataclass
class Vessel:
    name: str
    goal: np.ndarray
    position: np.ndarray = field(default_factory=lambda: np.zeros(2))
    heading: float = 0.0
    stamp: float = -math.inf
    initialized: bool = False
    reached: bool = False


class CrossingDriver(Node):
    """Deterministic route driver used only to generate a repeatable encounter."""

    def __init__(self):
        super().__init__("two_vessel_crossing_driver")
        self.declare_parameter("log_dir", "/tmp/vrx_two_vessel_crossing")
        self.declare_parameter("cruise_thrust", 340.0)
        self.declare_parameter("max_turn", 150.0)
        self.declare_parameter("yield_distance", 25.0)
        self.declare_parameter("resume_distance", 29.0)
        self.declare_parameter("goal_tolerance", 4.5)

        self.log_dir = str(self.get_parameter("log_dir").value)
        self.cruise = float(self.get_parameter("cruise_thrust").value)
        self.max_turn = float(self.get_parameter("max_turn").value)
        self.yield_distance = float(self.get_parameter("yield_distance").value)
        self.resume_distance = float(self.get_parameter("resume_distance").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.vessels = [
            Vessel("wamv_1", np.array([35.0, 0.0])),
            Vessel("wamv_2", np.array([0.0, 35.0])),
        ]
        self.yielding = False
        self.thrust_publishers = {}

        for index, vessel in enumerate(self.vessels):
            for topic in (f"/{vessel.name}/pose", f"/model/{vessel.name}/pose"):
                self.create_subscription(
                    TFMessage, topic,
                    lambda msg, i=index: self.pose_callback(msg, i), 10,
                )
            self.thrust_publishers[vessel.name] = (
                self.create_publisher(Float64, f"/{vessel.name}/thrusters/left/thrust", 10),
                self.create_publisher(Float64, f"/{vessel.name}/thrusters/right/thrust", 10),
            )

        os.makedirs(self.log_dir, exist_ok=True)
        self.file = open(os.path.join(self.log_dir, "crossing_driver.csv"), "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "time", "vessel", "x", "y", "heading", "goal_distance",
            "inter_vessel_distance", "yielding", "left_thrust", "right_thrust",
        ])
        self.timer = self.create_timer(0.1, self.step)
        self.get_logger().info(
            "Two-vessel crossing driver active: wamv_1 has right of way; wamv_2 yields."
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def pose_callback(self, msg, index):
        transform = next((
            item for item in msg.transforms
            if item.child_frame_id.endswith("/base_link")
            or item.child_frame_id.endswith("base_link")
        ), None)
        if transform is None:
            return
        vessel = self.vessels[index]
        translation = transform.transform.translation
        vessel.position = np.array([translation.x, translation.y], dtype=float)
        vessel.heading = yaw_from_quaternion(transform.transform.rotation)
        vessel.stamp = self.now_seconds()
        vessel.initialized = True

    def publish(self, vessel, left, right):
        left_msg, right_msg = Float64(), Float64()
        left_msg.data, right_msg.data = float(left), float(right)
        self.thrust_publishers[vessel.name][0].publish(left_msg)
        self.thrust_publishers[vessel.name][1].publish(right_msg)

    def command(self, vessel, speed_scale):
        offset = vessel.goal - vessel.position
        distance = float(np.linalg.norm(offset))
        if distance <= self.goal_tolerance:
            vessel.reached = True
            return 0.0, 0.0, distance
        desired = math.atan2(offset[1], offset[0])
        error = wrap_angle(desired - vessel.heading)
        forward_scale = max(0.12, math.cos(error))
        forward = self.cruise * speed_scale * forward_scale
        turn = float(np.clip(210.0 * error, -self.max_turn, self.max_turn))
        return forward - turn, forward + turn, distance

    def step(self):
        now = self.now_seconds()
        if not all(v.initialized and now - v.stamp < 1.0 for v in self.vessels):
            for vessel in self.vessels:
                self.publish(vessel, 0.0, 0.0)
            return

        separation = float(np.linalg.norm(self.vessels[1].position - self.vessels[0].position))
        if self.yielding:
            self.yielding = separation < self.resume_distance
        elif separation < self.yield_distance and not self.vessels[0].reached:
            self.yielding = True

        for index, vessel in enumerate(self.vessels):
            scale = 0.0 if index == 1 and self.yielding else 1.0
            left, right, goal_distance = self.command(vessel, scale)
            self.publish(vessel, left, right)
            self.writer.writerow([
                f"{now:.3f}", vessel.name, f"{vessel.position[0]:.3f}",
                f"{vessel.position[1]:.3f}", f"{vessel.heading:.5f}",
                f"{goal_distance:.3f}", f"{separation:.3f}", int(self.yielding),
                f"{left:.3f}", f"{right:.3f}",
            ])
        self.file.flush()

    def destroy_node(self):
        for vessel in self.vessels:
            self.publish(vessel, 0.0, 0.0)
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CrossingDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
