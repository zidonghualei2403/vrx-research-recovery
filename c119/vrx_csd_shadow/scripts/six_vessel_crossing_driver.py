#!/usr/bin/python3
import csv
import json
import math
import os
import time
from dataclasses import dataclass, field

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64, Float64MultiArray


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class DeterministicSACActor:
    """Dependency-free NumPy inference for an exported SB3 SAC actor."""

    def __init__(self, path):
        with open(path, "r", encoding="utf-8") as stream:
            payload = json.load(stream)
        if payload.get("format") not in {
                "vrx_deterministic_sac_actor_v1",
                "vrx_deterministic_sac_actor_v2"}:
            raise ValueError(f"Unsupported SAC actor format in {path}")
        self.format = payload["format"]
        self.observation_schema = payload.get("observation_schema", "legacy")
        self.deployment_gate = payload.get("deployment_gate", "legacy")
        self.observation_dim = int(payload["observation_dim"])
        self.action_dim = int(payload["action_dim"])
        self.heading_scale = float(payload["heading_scale_rad"])
        self.speed_scale = float(payload["speed_scale_mps"])
        self.layers = [
            (np.asarray(layer["weight"], dtype=float),
             np.asarray(layer["bias"], dtype=float))
            for layer in payload["layers"]
        ]

    def predict(self, observation):
        value = np.asarray(observation, dtype=float)
        if value.shape != (self.observation_dim,):
            raise ValueError(
                f"SAC observation has shape {value.shape}, expected "
                f"({self.observation_dim},)")
        for weight, bias in self.layers[:-1]:
            value = np.maximum(0.0, weight @ value + bias)
        weight, bias = self.layers[-1]
        action = np.tanh(weight @ value + bias)
        if action.shape != (self.action_dim,):
            raise ValueError("Exported SAC actor produced an invalid action")
        return action.reshape(4, 2)


@dataclass
class Vessel:
    name: str
    goal: np.ndarray
    expected_start: np.ndarray
    position: np.ndarray = field(default_factory=lambda: np.zeros(2))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(2))
    heading: float = 0.0
    yaw_rate: float = 0.0
    stamp: float = -math.inf
    initialized: bool = False
    gps_ready: bool = False
    imu_ready: bool = False
    reached: bool = False
    valid_start_samples: int = 0


class SixVesselCrossingDriver(Node):
    """Repeatable six-vessel encounter driver, separate from the learned graph."""

    def __init__(self):
        super().__init__("six_vessel_crossing_driver")
        self.declare_parameter("log_dir", "/tmp/vrx_six_vessel_crossing")
        self.declare_parameter("cruise_thrust", 300.0)
        self.declare_parameter("max_turn", 145.0)
        self.declare_parameter("yield_distance", 28.0)
        self.declare_parameter("resume_distance", 32.0)
        self.declare_parameter("goal_tolerance", 5.0)
        self.declare_parameter("goal_slowdown_radius", 14.0)
        self.declare_parameter("patrol_endpoint_buffer", 5.5)
        self.declare_parameter("origin_latitude", -33.724223)
        self.declare_parameter("origin_longitude", 150.679736)
        self.declare_parameter("control_mode", "baseline")
        self.declare_parameter("graph_mode", "conditional_csd")
        self.declare_parameter("graph_timeout", 0.8)
        self.declare_parameter("prediction_horizon", 12.0)
        self.declare_parameter("combined_safety_radius", 8.0)
        self.declare_parameter("encounter_mode", "offset")
        self.declare_parameter("residence_radius", 6.0)
        self.declare_parameter("required_residence", 10.0)
        self.declare_parameter("post_arrival_observation", 10.0)
        self.declare_parameter("max_experiment_duration", 300.0)
        self.declare_parameter("start_position_tolerance", 4.0)
        self.declare_parameter("required_start_samples", 5)
        self.declare_parameter("startup_timeout", 30.0)
        self.declare_parameter("pose_freshness_timeout", 3.0)
        self.declare_parameter("environment_uncertainty", 0.0)
        self.declare_parameter("cooperative_count", 6)
        self.declare_parameter("residual_sac_model_path", "")
        self.declare_parameter("sea_state", "calm")
        self.declare_parameter("residual_sac_authority", 0.35)

        self.log_dir = str(self.get_parameter("log_dir").value)
        self.cruise = float(self.get_parameter("cruise_thrust").value)
        self.max_turn = float(self.get_parameter("max_turn").value)
        self.yield_distance = float(self.get_parameter("yield_distance").value)
        self.resume_distance = float(self.get_parameter("resume_distance").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.goal_slowdown_radius = float(
            self.get_parameter("goal_slowdown_radius").value)
        self.patrol_endpoint_buffer = float(
            self.get_parameter("patrol_endpoint_buffer").value)
        self.origin_latitude = float(self.get_parameter("origin_latitude").value)
        self.origin_longitude = float(self.get_parameter("origin_longitude").value)
        self.control_mode = str(self.get_parameter("control_mode").value)
        self.graph_mode = str(self.get_parameter("graph_mode").value)
        self.graph_timeout = float(self.get_parameter("graph_timeout").value)
        self.prediction_horizon = float(self.get_parameter("prediction_horizon").value)
        self.combined_safety_radius = float(
            self.get_parameter("combined_safety_radius").value)
        self.encounter_mode = str(self.get_parameter("encounter_mode").value)
        self.environment_uncertainty = float(
            self.get_parameter("environment_uncertainty").value)
        self.cooperative_count = int(
            self.get_parameter("cooperative_count").value)
        self.sea_state = str(self.get_parameter("sea_state").value)
        self.residual_sac_authority = float(np.clip(
            self.get_parameter("residual_sac_authority").value, 0.0, 1.0))
        self.residual_sac_actor = None
        if self.graph_mode == "gnn_gru_residual_sac":
            actor_path = str(self.get_parameter("residual_sac_model_path").value)
            if not actor_path or not os.path.isfile(actor_path):
                raise FileNotFoundError(
                    f"Residual SAC actor model not found: {actor_path}")
            self.residual_sac_actor = DeterministicSACActor(actor_path)
        self.residual_authority = float(np.clip(
            (0.90 - self.environment_uncertainty) / 0.15, 0.0, 1.0))
        self.residence_radius = float(self.get_parameter("residence_radius").value)
        self.required_residence = float(
            self.get_parameter("required_residence").value)
        self.post_arrival_observation = float(
            self.get_parameter("post_arrival_observation").value)
        self.max_experiment_duration = float(
            self.get_parameter("max_experiment_duration").value)
        self.start_position_tolerance = float(
            self.get_parameter("start_position_tolerance").value)
        self.required_start_samples = int(
            self.get_parameter("required_start_samples").value)
        self.startup_timeout = float(
            self.get_parameter("startup_timeout").value)
        self.pose_freshness_timeout = float(
            self.get_parameter("pose_freshness_timeout").value)
        if self.control_mode not in ("baseline", "csd_safe"):
            raise ValueError(f"Unsupported control_mode={self.control_mode}")
        if self.encounter_mode not in ("offset", "central", "harbor"):
            raise ValueError(f"Unsupported encounter_mode={self.encounter_mode}")

        if self.encounter_mode == "central":
            starts = [
                (45.0, 0.0), (22.5, 38.97114317),
                (-22.5, 38.97114317), (-45.0, 0.0),
                (-22.5, -38.97114317), (22.5, -38.97114317),
            ]
            goals = [
                (-45.0, 0.0), (-22.5, -38.97114317),
                (22.5, -38.97114317), (45.0, 0.0),
                (22.5, 38.97114317), (-22.5, 38.97114317),
            ]
        elif self.encounter_mode == "harbor":
            starts = [
                (-48.0, -22.0), (-48.0, 22.0),
                (48.0, -8.0), (48.0, 8.0),
                (-25.0, -20.0), (25.0, 20.0),
            ]
            goals = [
                (48.0, 22.0), (48.0, -22.0),
                (-48.0, 8.0), (-48.0, -8.0),
                (-25.0, 20.0), (25.0, -20.0),
            ]
        else:
            starts = [
                (38.97114317, -22.5), (38.97114317, 22.5), (0.0, 45.0),
                (-38.97114317, 22.5), (-38.97114317, -22.5), (0.0, -45.0),
            ]
            goals = [
                (-38.97114317, 22.5), (-38.97114317, -22.5), (0.0, -45.0),
                (38.97114317, -22.5), (38.97114317, 22.5), (0.0, 45.0),
            ]
        self.vessels = [
            Vessel(
                f"wamv_{i + 1}",
                np.asarray(goal, dtype=float),
                np.asarray(starts[i], dtype=float),
            )
            for i, goal in enumerate(goals)
        ]
        self.patrol_direction = np.ones(len(self.vessels), dtype=float)
        self.patrol_filtered_speed = np.zeros(len(self.vessels), dtype=float)
        self.patrol_forward_thrust = np.zeros(len(self.vessels), dtype=float)
        self.patrol_axes = []
        for vessel in self.vessels:
            axis = vessel.goal - vessel.expected_start
            axis /= max(float(np.linalg.norm(axis)), 1e-9)
            self.patrol_axes.append(axis)
        self.static_rectangles = (
            ((-11.0, 13.0), (7.0, 3.0)),
            ((12.0, -13.0), (7.0, 3.0)),
        ) if self.encounter_mode == "harbor" else ()
        # Candidate C01: compensate only the static-wall prediction mismatch.
        # Collision auditing still uses the original physical wall geometry.
        self.static_wall_prediction_padding = np.array([0.35, 0.35])
        # Nominal center-corridor routes used by the MATLAB harbor scenario.
        # The learned graph may locally deviate for collision avoidance, but it
        # must return to these routes instead of escaping around the map sides.
        self.harbor_reference_routes = (
            ((-24.0, -11.0), (0.0, -4.0), (24.0, 8.0)),
            ((-30.0, 25.5), (-14.0, 25.5), (7.0, 25.5),
             (16.0, 21.0), (23.0, 13.0), (31.0, 5.5), (38.0, 0.0)),
            ((24.0, -5.0), (0.0, 2.0), (-24.0, 4.0)),
            ((24.0, 8.0), (0.0, 4.0), (-24.0, -4.0)),
        ) if self.encounter_mode == "harbor" else ()
        self.harbor_route_index = np.zeros(4, dtype=int)
        # C64: remember that wamv_1 has cleared a real route conflict.  This
        # enables a small progress-recovery bias only after the conflict has
        # disappeared; active safety penalties remain unchanged.
        self.wamv1_route_conflict_seen = False
        # C93: reserve wamv_1's final crossing of the vertical shuttle lane.
        # The shuttle has right of way; once a safe trailing gap opens, wamv_1
        # commits long enough to clear the lane instead of negotiating again
        # while it is inside the conflict area.
        self.wamv1_shuttle_commit_until = -math.inf
        # C119: the final wamv_6 crossing is a spatial state machine. Once a
        # live relative-motion check grants the gap, keep moving horizontally
        # until wamv_2's complete hull has cleared the shuttle lane. There is
        # deliberately no wall-clock expiry for this commitment.
        self.wamv2_final_horizontal_committed = False
        self.wamv2_final_hold_position = None
        # C94: the earlier C79 gate covers only the final crossing with
        # wamv_6.  Reserve wamv_2's initial crossing with wamv_5 separately,
        # using measured shuttle motion rather than launch-time phase.
        self.wamv2_initial_commit_until = -math.inf
        # C98: once wamv_2 starts this crossing, do not send it back to the
        # old waiting point when the short C94 timer expires. The latch is
        # released naturally after the complete vessel clears x=-18 m.
        self.wamv2_initial_crossing_committed = False
        self.yielding = np.zeros(len(self.vessels), dtype=bool)
        self.previous_avoid_offset = np.zeros(len(self.vessels), dtype=float)
        self.residual_eligible = np.zeros(len(self.vessels), dtype=bool)
        self.residual_accepted = np.zeros(len(self.vessels), dtype=bool)
        self.residual_heading_delta = np.zeros(len(self.vessels), dtype=float)
        self.residual_speed_delta = np.zeros(len(self.vessels), dtype=float)
        self.last_predicted_clearance = np.full(len(self.vessels), math.nan)
        self.emergency_braking = np.zeros(len(self.vessels), dtype=bool)
        self.graph_edges = {i: [] for i in range(len(self.vessels))}
        self.graph_stamp = -math.inf
        self.control_announced = False
        self.experiment_start = None
        self.residence_start = np.full(len(self.vessels), np.nan)
        self.residence_qualified = np.zeros(len(self.vessels), dtype=bool)
        self.arrival_qualified = np.zeros(len(self.vessels), dtype=bool)
        self.first_arrival_time = np.full(len(self.vessels), np.nan)
        self.all_arrived_time = math.nan
        self.startup_wall_time = time.monotonic()
        self.startup_failed = False
        self.thrust_publishers = {}
        for index, vessel in enumerate(self.vessels):
            self.create_subscription(
                NavSatFix, f"/{vessel.name}/sensors/gps/gps/fix",
                lambda msg, i=index: self.gps_callback(msg, i), 10,
            )
            self.create_subscription(
                Imu, f"/{vessel.name}/sensors/imu/imu/data",
                lambda msg, i=index: self.imu_callback(msg, i), 10,
            )
            self.thrust_publishers[vessel.name] = (
                self.create_publisher(Float64, f"/{vessel.name}/thrusters/left/thrust", 10),
                self.create_publisher(Float64, f"/{vessel.name}/thrusters/right/thrust", 10),
            )
        self.create_subscription(
            Float64MultiArray, "/csd/active_edges", self.graph_callback, 10)

        os.makedirs(self.log_dir, exist_ok=True)
        self.file = open(os.path.join(self.log_dir, "six_vessel_driver.csv"), "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            "time", "vessel", "x", "y", "heading", "goal_x", "goal_y",
            "goal_distance", "nearest_vessel_distance", "yielding", "left_thrust",
            "right_thrust", "reached",
            "control_mode", "graph_mode", "graph_fresh", "active_edge_count",
            "graph_max_risk", "graph_min_clearance", "avoid_offset_rad",
            "command_speed_scale", "arrival_qualified", "residence_qualified",
            "residual_eligible", "residual_accepted",
            "residual_heading_delta_rad", "residual_speed_delta",
            "environment_uncertainty", "residual_authority",
            "patrol_direction", "harbor_route_stage",
        ])
        self.timer = self.create_timer(0.1, self.step)
        self.get_logger().info(
            f"Six physical WAM-V vessels active; control_mode={self.control_mode}, "
            f"graph_mode={self.graph_mode}, encounter_mode={self.encounter_mode}."
        )
        if self.residual_sac_actor is not None:
            self.get_logger().info(
                f"Residual SAC actor active; sea_state={self.sea_state}, "
                f"authority={self.residual_sac_authority:.2f}")

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def announce_pose_if_ready(self, vessel):
        was_initialized = vessel.initialized
        vessel.initialized = vessel.gps_ready and vessel.imu_ready
        if vessel.initialized and not was_initialized:
            self.get_logger().info(
                f"Pose locked: {vessel.name} at "
                f"({vessel.position[0]:.2f}, {vessel.position[1]:.2f})"
            )

    def gps_callback(self, msg, index):
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        vessel = self.vessels[index]
        earth_radius = 6378137.0
        x = math.radians(msg.longitude - self.origin_longitude) * earth_radius
        x *= math.cos(math.radians(self.origin_latitude))
        y = math.radians(msg.latitude - self.origin_latitude) * earth_radius
        position = np.array([x, y], dtype=float)
        now = self.now_seconds()
        if not vessel.initialized:
            start_error = float(np.linalg.norm(position - vessel.expected_start))
            if start_error > self.start_position_tolerance:
                vessel.valid_start_samples = 0
                vessel.gps_ready = False
                return
            vessel.valid_start_samples += 1
            if vessel.valid_start_samples < self.required_start_samples:
                vessel.position = position
                vessel.stamp = now
                return
        if vessel.gps_ready:
            dt = max(now - vessel.stamp, 1e-3)
            measured_velocity = (position - vessel.position) / dt
            vessel.velocity = 0.65 * vessel.velocity + 0.35 * measured_velocity
        vessel.position = position
        vessel.stamp = now
        vessel.gps_ready = True
        self.announce_pose_if_ready(vessel)

    def imu_callback(self, msg, index):
        vessel = self.vessels[index]
        vessel.heading = yaw_from_quaternion(msg.orientation)
        measured_yaw_rate = float(msg.angular_velocity.z)
        if math.isfinite(measured_yaw_rate):
            vessel.yaw_rate = measured_yaw_rate
        vessel.imu_ready = True
        self.announce_pose_if_ready(vessel)

    def publish(self, vessel, left, right):
        left_msg, right_msg = Float64(), Float64()
        left_msg.data, right_msg.data = float(left), float(right)
        self.thrust_publishers[vessel.name][0].publish(left_msg)
        self.thrust_publishers[vessel.name][1].publish(right_msg)

    def graph_callback(self, msg):
        data = list(msg.data)
        stride = 11 if self.graph_mode in (
            "gnn_residual_csd", "gnn_uncertainty_residual_csd") else 8
        if len(data) % stride != 0:
            self.get_logger().warning(
                f"Ignored malformed CSD edge message with {len(data)} values")
            return
        edges = {i: [] for i in range(len(self.vessels))}
        for offset in range(0, len(data), stride):
            ego = int(round(data[offset]))
            neighbor = int(round(data[offset + 1]))
            if ego not in edges or not 0 <= neighbor < len(self.vessels):
                continue
            edge = {
                "neighbor": neighbor,
                "risk": float(data[offset + 2]),
                "clearance": float(data[offset + 3]),
                "lower_bound": float(data[offset + 4]),
                "hard_safety": bool(round(data[offset + 5])),
                "tcpa": float(data[offset + 6]),
                "critical_probability": float(data[offset + 7]),
            }
            if stride == 11:
                edge.update({
                    "advice_sin": float(data[offset + 8]),
                    "advice_cos": float(data[offset + 9]),
                    "advice_speed": float(data[offset + 10]),
                })
            edges[ego].append(edge)
        self.graph_edges = edges
        self.graph_stamp = self.now_seconds()

    def command(self, vessel, speed_scale):
        offset = vessel.goal - vessel.position
        distance = float(np.linalg.norm(offset))
        if distance <= self.goal_tolerance:
            vessel.reached = True
            return 0.0, 0.0, distance
        desired = math.atan2(offset[1], offset[0])
        error = wrap_angle(desired - vessel.heading)
        approach_scale = float(np.clip(
            (distance - self.goal_tolerance)
            / max(self.goal_slowdown_radius - self.goal_tolerance, 1e-6),
            0.12,
            1.0,
        ))
        forward = (
            self.cruise * speed_scale * approach_scale
            * max(0.10, math.cos(error))
        )
        turn = float(np.clip(205.0 * error, -self.max_turn, self.max_turn))
        turn *= max(0.30, approach_scale)
        return forward - turn, forward + turn, distance

    def startup_hold_command(self, index):
        """Keep each ready vessel near its spawn while other bridges lock."""
        vessel = self.vessels[index]
        offset = vessel.expected_start - vessel.position
        distance = float(np.linalg.norm(offset))
        nominal_axis = self.patrol_axes[index]
        if distance > 0.30:
            desired_heading = math.atan2(offset[1], offset[0])
            forward_scale = float(np.clip(distance / 4.0, 0.12, 0.45))
        else:
            desired_heading = math.atan2(nominal_axis[1], nominal_axis[0])
            forward_scale = 0.0
        heading_error = wrap_angle(desired_heading - vessel.heading)
        forward = (
            self.cruise * forward_scale * max(0.08, math.cos(heading_error)))
        turn = float(np.clip(
            170.0 * heading_error, -0.70 * self.max_turn, 0.70 * self.max_turn))
        return forward - turn, forward + turn

    def patrol_command(self, index):
        """Track the ideal shuttle route without reacting to raw GPS jitter."""
        vessel = self.vessels[index]
        axis = self.patrol_axes[index]
        start = vessel.expected_start
        end = vessel.goal
        direction = self.patrol_direction[index]
        target = end if direction > 0.0 else start
        target_distance = float(np.linalg.norm(target - vessel.position))
        route_length = float(np.linalg.norm(end - start))
        progress = float(np.dot(vessel.position - start, axis))

        # Switch before the endpoint so the real hull has room to arrest its
        # longitudinal momentum. Overshoot checks keep the patrol recoverable.
        if ((direction > 0.0 and (
                target_distance <= self.patrol_endpoint_buffer
                or progress >= route_length))
                or (direction < 0.0 and (
                    target_distance <= self.patrol_endpoint_buffer
                    or progress <= 0.0))):
            direction *= -1.0
            self.patrol_direction[index] = direction
            target = end if direction > 0.0 else start
            target_distance = float(np.linalg.norm(target - vessel.position))
            self.get_logger().info(
                f"{vessel.name} patrol thrust reversed; direction={direction:+.0f}")

        # Match the ideal MATLAB shuttle while respecting the real WAM-V's
        # inertia. Raw GPS differentiation is too noisy for a high-gain speed
        # loop, so use a slow speed estimate and a slew-limited feed-forward
        # command. Cross-track feedback keeps the vessel on its prescribed
        # line in wind and waves without changing its traffic role.
        measured_along_speed = float(np.dot(vessel.velocity, axis))
        filtered_speed = (
            0.92 * self.patrol_filtered_speed[index]
            + 0.08 * measured_along_speed
        )
        self.patrol_filtered_speed[index] = filtered_speed

        normal = np.array([-axis[1], axis[0]], dtype=float)
        cross_track = float(np.dot(vessel.position - start, normal))
        cross_track_heading = float(np.clip(-0.045 * cross_track, -0.24, 0.24))
        desired_heading = math.atan2(axis[1], axis[0]) + direction * cross_track_heading
        heading_error = wrap_angle(desired_heading - vessel.heading)
        desired_along_speed = 1.30 * direction
        target_forward = float(np.clip(
            0.46 * self.cruise * direction
            + 28.0 * (desired_along_speed - filtered_speed),
            -0.62 * self.cruise,
            0.62 * self.cruise,
        ))
        previous_forward = self.patrol_forward_thrust[index]
        max_thrust_step = 18.0
        forward = previous_forward + float(np.clip(
            target_forward - previous_forward,
            -max_thrust_step,
            max_thrust_step,
        ))
        self.patrol_forward_thrust[index] = forward
        turn = float(np.clip(
            150.0 * heading_error, -0.60 * self.max_turn, 0.60 * self.max_turn))
        vessel.reached = False
        return forward - turn, forward + turn, target_distance

    def command_heading(self, vessel, desired, speed_scale, allow_goal_evasion=False):
        distance = float(np.linalg.norm(vessel.goal - vessel.position))
        if distance <= self.goal_tolerance and not allow_goal_evasion:
            vessel.reached = True
            return 0.0, 0.0, distance
        error = wrap_angle(desired - vessel.heading)
        approach_scale = float(np.clip(
            (distance - self.goal_tolerance)
            / max(self.goal_slowdown_radius - self.goal_tolerance, 1e-6),
            0.12,
            1.0,
        ))
        forward = (
            self.cruise * speed_scale * approach_scale
            * max(0.10, math.cos(error))
        )
        turn = float(np.clip(205.0 * error, -self.max_turn, self.max_turn))
        turn *= max(0.30, approach_scale)
        return forward - turn, forward + turn, distance

    def learned_graph_shield_command(self, index, threats):
        """Match the MATLAB graph controller used by the learned GNN-GRU method."""
        vessel = self.vessels[index]
        goal_offset = vessel.goal - vessel.position
        goal_distance = float(np.linalg.norm(goal_offset))
        if goal_distance <= self.goal_tolerance:
            vessel.reached = True
            return 0.0, 0.0, goal_distance, 0.0, 0.0

        goal_heading = math.atan2(goal_offset[1], goal_offset[0])
        goal_direction = goal_offset / max(goal_distance, 1e-9)
        right_direction = np.array([goal_direction[1], -goal_direction[0]])
        desired_vector = goal_direction.copy()
        speed_scale = 1.0

        for threat in threats:
            neighbor = self.vessels[threat["neighbor"]]
            relative = neighbor.position - vessel.position
            distance = max(float(np.linalg.norm(relative)), 1e-6)
            bearing = relative / distance
            tcpa = max(0.0, float(threat["tcpa"]))
            urgency = float(threat["risk"]) * (
                0.45 + 0.55 * math.exp(-tcpa / 5.0))
            urgency = min(1.35, 1.45 * urgency)

            # All vessels use the same maritime starboard convention. This is
            # the symmetry breaker used by the MATLAB controller: reciprocal
            # encounters circulate around the conflict region instead of all
            # selecting zero speed.
            desired_vector += urgency * (2.20 * right_direction - 0.80 * bearing)
            speed_scale = min(speed_scale, max(0.18, 1.0 - 0.82 * urgency))

        # The static harbor walls are known geometry. They enter the same
        # directional ship-domain shield as graph threats, while moving
        # vessels remain represented by learned graph edges.
        for center, half_size in self.static_rectangles:
            center = np.asarray(center, dtype=float)
            half_size = np.asarray(half_size, dtype=float)
            predicted = vessel.position + 3.0 * vessel.velocity
            nearest = np.clip(predicted, center - half_size, center + half_size)
            away = predicted - nearest
            distance = float(np.linalg.norm(away))
            if distance < 1e-6:
                away = predicted - center
                distance = max(float(np.linalg.norm(away)), 1e-6)
            wall_clearance = distance - 3.4
            if wall_clearance < 12.0:
                urgency = float(np.clip((12.0 - wall_clearance) / 12.0, 0.0, 1.4))
                desired_vector += 2.8 * urgency * (away / distance)
                desired_vector += 0.45 * urgency * right_direction
                speed_scale = min(speed_scale, max(0.20, 1.0 - 0.70 * urgency))

        norm = float(np.linalg.norm(desired_vector))
        if norm < 1e-9:
            desired_heading = goal_heading
        else:
            desired_vector /= norm
            desired_heading = math.atan2(desired_vector[1], desired_vector[0])

        avoid_offset = wrap_angle(desired_heading - goal_heading)
        self.previous_avoid_offset[index] = avoid_offset
        left, right, distance = self.command_heading(
            vessel, desired_heading, speed_scale)
        return left, right, distance, avoid_offset, speed_scale

    @staticmethod
    def rectangle_signed_distance(point, center, half_size):
        delta = np.abs(point - center) - half_size
        outside = float(np.linalg.norm(np.maximum(delta, 0.0)))
        inside = min(max(float(delta[0]), float(delta[1])), 0.0)
        return outside + inside

    def harbor_reference_target(self, index):
        """Advance after entering or safely crossing a route waypoint gate."""
        vessel = self.vessels[index]
        route = self.harbor_reference_routes[index]
        route_index = int(self.harbor_route_index[index])
        while route_index < len(route):
            target = np.asarray(route[route_index], dtype=float)
            previous = (
                vessel.expected_start if route_index == 0
                else np.asarray(route[route_index - 1], dtype=float))
            leg = target - previous
            leg_length = max(float(np.linalg.norm(leg)), 1e-9)
            leg_direction = leg / leg_length
            target_delta = vessel.position - target
            passed_gate = float(np.dot(target_delta, leg_direction)) >= 0.0
            cross_track = abs(float(
                leg_direction[0] * target_delta[1]
                - leg_direction[1] * target_delta[0]))
            # The first harbor turn sits beside a wall corner. A 6 m gate let
            # crosswind-displaced vessels turn before reaching the safe side
            # of that corner. Require an accurate first approach; retain the
            # wider gate later, where it prevents waypoint-return loops.
            if index == 1 and route_index <= 1:
                # Preserve the gradual route around both wall corners instead
                # of cutting a bend through a wide generic waypoint gate.
                gate_radius = 2.5
                gate_cross_track = 3.0
            elif index == 1:
                # Once both wall-corner gates are behind wamv_2, a safety
                # detour may cross a later gate several metres off the nominal
                # line. Never make it turn back toward an already passed point.
                gate_radius = 2.5
                gate_cross_track = 14.0
            else:
                gate_radius = 3.0 if route_index == 0 else 6.0
                gate_cross_track = 4.0 if route_index == 0 else 18.0
            entered = float(np.linalg.norm(target_delta)) <= gate_radius
            # wamv_2 must first climb to the fixed upper-corridor height before
            # it is allowed to turn east.  The old x-only shortcut also applied
            # to waypoint zero, so wind-driven x progress could release the
            # turn while the vessel was still below the safe line.
            upper_turn_aligned = (
                index == 1 and route_index == 0
                and float(vessel.position[1]) >= float(target[1]) - 0.25)
            # C103: the first waypoint is a one-way entrance to the upper
            # corridor. Wind can carry wamv_2 just below its small waypoint
            # disk after it has already made enough eastward progress. Do not
            # send it back through the wamv_5 lane to hit that disk again.
            first_corridor_entered = (
                index == 1 and route_index == 0
                and float(vessel.position[0]) >= float(target[0]) - 3.0
                and float(vessel.position[1]) >= float(target[1]) - 2.5)
            upper_corridor_cleared = (
                index == 1 and 1 <= route_index <= 2
                and float(vessel.position[0]) >= float(target[0]))
            # A safety manoeuvre may legitimately miss the waypoint disk. The
            # perpendicular gate prevents the controller from turning back to
            # an already passed point and entering a limit cycle.
            if index == 1 and route_index == 0:
                # This is a hard phase boundary: no generic passed-waypoint
                # test may bypass the fixed-y turn latch.
                if (not entered and not upper_turn_aligned
                        and not first_corridor_entered):
                    return self.harbor_smoothed_reference_target(
                        index, route_index, previous, target)
            elif index == 0 and route_index == 0 and passed_gate:
                # Once wamv_1 crosses its first waypoint gate, never command it
                # back toward that point after wind or avoidance displacement.
                pass
            elif not entered and not upper_corridor_cleared and not (
                    passed_gate and cross_track <= gate_cross_track):
                return self.harbor_smoothed_reference_target(
                    index, route_index, previous, target)
            route_index += 1
            self.harbor_route_index[index] = route_index
            self.get_logger().info(
                f"{vessel.name} cleared harbor waypoint "
                f"{route_index}/{len(route)}")
        return vessel.goal

    def harbor_smoothed_reference_target(
            self, index, route_index, previous, target):
        """Use a moving lookahead on wamv_2's active route leg."""
        if index != 1 or route_index == 0:
            return target

        vessel = self.vessels[index]
        leg = target - previous
        leg_length = max(float(np.linalg.norm(leg)), 1e-9)
        leg_direction = leg / leg_length
        along_track = float(np.dot(vessel.position - previous, leg_direction))
        lookahead = 11.0 if route_index <= 2 else 7.0
        target_along = float(np.clip(
            along_track + lookahead, 0.35 * leg_length, leg_length))
        return previous + target_along * leg_direction

    def predicted_patrol_position(self, index, prediction_time):
        """Predict a shuttle vessel including its scheduled thrust reversal."""
        vessel = self.vessels[index]
        start = vessel.expected_start
        end = vessel.goal
        axis = self.patrol_axes[index]
        route_length = float(np.linalg.norm(end - start))
        low = self.patrol_endpoint_buffer
        high = route_length - self.patrol_endpoint_buffer
        scalar = float(np.dot(vessel.position - start, axis))
        lateral = vessel.position - (start + scalar * axis)
        direction = float(self.patrol_direction[index])
        travel = 1.15 * prediction_time
        while travel > 1e-9:
            boundary = high if direction > 0.0 else low
            distance = abs(boundary - scalar)
            if travel <= distance:
                scalar += direction * travel
                travel = 0.0
            else:
                scalar = boundary
                travel -= distance
                direction *= -1.0
        return start + scalar * axis + lateral

    def harbor_forward_gap_open(self, index, neighbor_index,
                                reference_target, desired_speed):
        """Return true when the first harbor crossing can be cleared first.

        This is deliberately narrow: only wamv_3 approaching its first route
        gate against the vertical wamv_6 shuttle may claim the gap. The test
        includes acceleration, the complete ego hull passing the conflict
        point, and a time buffer before the shuttle enters that point.
        """
        if (self.encounter_mode != "harbor" or index != 2
                or neighbor_index != 5
                or int(self.harbor_route_index[index]) != 0
                or desired_speed < 1.9):
            return False

        vessel = self.vessels[index]
        shuttle = self.vessels[neighbor_index]
        ego_start = vessel.position
        ego_leg = np.asarray(reference_target, dtype=float) - ego_start
        shuttle_leg = shuttle.goal - shuttle.expected_start

        def cross(first, second):
            return float(first[0] * second[1] - first[1] * second[0])

        denominator = cross(ego_leg, shuttle_leg)
        if abs(denominator) < 1e-6:
            return False
        relative_start = shuttle.expected_start - ego_start
        ego_fraction = cross(relative_start, shuttle_leg) / denominator
        shuttle_fraction = cross(relative_start, ego_leg) / denominator
        if not (0.0 <= ego_fraction <= 1.15
                and -0.10 <= shuttle_fraction <= 1.10):
            return False

        conflict = ego_start + ego_fraction * ego_leg
        # The stern must clear the crossing, not merely the vessel center.
        required_travel = float(np.linalg.norm(conflict - ego_start)) + 3.5
        hull_forward = np.array([
            math.cos(vessel.heading), math.sin(vessel.heading)])
        speed = max(0.0, float(np.dot(vessel.velocity, hull_forward)))
        elapsed = 0.0
        travelled = 0.0
        integration_step = 0.10
        while elapsed < 35.0 and travelled < required_travel:
            speed = min(float(desired_speed), speed + 0.65 * integration_step)
            travelled += speed * integration_step
            elapsed += integration_step
        if travelled < required_travel:
            return False
        ego_clear_time = elapsed

        shuttle_entry_time = math.inf
        for prediction_time in np.arange(0.0, 35.0 + 1e-9, 0.25):
            shuttle_position = self.predicted_patrol_position(
                neighbor_index, float(prediction_time))
            if float(np.linalg.norm(shuttle_position - conflict)) <= 5.0:
                shuttle_entry_time = float(prediction_time)
                break

        return ego_clear_time + 1.0 < shuttle_entry_time

    def harbor_wamv2_crossing_gap_open(self, reference_target, desired_speed):
        """Check whether wamv_2 can clear wamv_6's lane before it arrives.

        This applies only to wamv_2's final horizontal crossing segment. It is a
        scheduling decision, not a collision override: the full OBB rollout
        below still rejects every candidate that physically overlaps a ship.
        """
        index = 1
        neighbor_index = 5
        route_stage = int(self.harbor_route_index[index])
        if (self.encounter_mode != "harbor"
                or route_stage not in (4, 5)
                or desired_speed < 1.65):
            return False

        vessel = self.vessels[index]
        shuttle = self.vessels[neighbor_index]
        ego_start = vessel.position
        # C119: evaluate the exact direct motion that will be executed.
        # The previous implementation evaluated the obsolete diagonal route
        # endpoint (31, 5.5), then executed toward (31, 13), so its permission
        # did not describe the real crossing geometry.
        crossing_target = np.asarray((31.0, 13.0), dtype=float)
        ego_leg = crossing_target - ego_start
        shuttle_leg = shuttle.goal - shuttle.expected_start

        def cross(first, second):
            return float(first[0] * second[1] - first[1] * second[0])

        denominator = cross(ego_leg, shuttle_leg)
        if abs(denominator) < 1e-6:
            return False
        relative_start = shuttle.expected_start - ego_start
        ego_fraction = cross(relative_start, shuttle_leg) / denominator
        shuttle_fraction = cross(relative_start, ego_leg) / denominator
        if not (0.0 <= ego_fraction <= 1.20
                and -0.10 <= shuttle_fraction <= 1.10):
            return False

        conflict = ego_start + ego_fraction * ego_leg
        required_travel = float(np.linalg.norm(conflict - ego_start)) + 3.5
        hull_forward = np.array([
            math.cos(vessel.heading), math.sin(vessel.heading)])
        speed = max(0.0, float(np.dot(vessel.velocity, hull_forward)))
        elapsed = 0.0
        travelled = 0.0
        while elapsed < 20.0 and travelled < required_travel:
            speed = min(float(desired_speed), speed + 0.65 * 0.10)
            travelled += speed * 0.10
            elapsed += 0.10
        if travelled < required_travel:
            return False

        shuttle_entry_time = math.inf
        for prediction_time in np.arange(0.0, 20.0 + 1e-9, 0.25):
            shuttle_position = self.predicted_patrol_position(
                neighbor_index, float(prediction_time))
            if float(np.linalg.norm(shuttle_position - conflict)) <= 5.0:
                shuttle_entry_time = float(prediction_time)
                break
        # Preserve a complete-hull time window. The former 1.25 s threshold
        # admitted the 20261101 crossing with only a fraction of a second of
        # realized clearance after turn and acceleration lag.
        return elapsed + 4.0 < shuttle_entry_time

    def harbor_wamv1_shuttle_right_of_way(self):
        """Schedule wamv_1 on either safe side of wamv_6's passage."""
        if (self.encounter_mode != "harbor"
                or int(self.harbor_route_index[0]) < 2
                or float(self.vessels[0].position[0]) >= 31.0):
            return "none"

        now = self.now_seconds()
        vessel = self.vessels[0]
        shuttle = self.vessels[5]
        # Once the bow has entered the crossing approach, stopping or turning
        # back is less safe than clearing the narrow lane. Before that point a
        # short permit is re-evaluated from the live shuttle phase.
        if float(vessel.position[0]) >= 20.0:
            return "commit"
        if now < self.wamv1_shuttle_commit_until:
            return "commit"

        lane_x = 25.0
        crossing_y = 8.0
        approach_speed = 1.85
        lane_entry_eta = float(np.clip(
            (lane_x - float(vessel.position[0])) / approach_speed,
            0.0, 8.0))
        predicted_shuttle_y = float(self.predicted_patrol_position(
            5, lane_entry_eta)[1])
        shuttle_velocity_y = float(shuttle.velocity[1])
        moving_upward = (
            float(self.patrol_direction[5]) < 0.0
            and shuttle_velocity_y > 0.20)
        moving_downward = (
            float(self.patrol_direction[5]) > 0.0
            and shuttle_velocity_y < -0.20)
        predicted_leading_gap = (
            moving_upward
            and predicted_shuttle_y >= crossing_y + 7.5)
        predicted_trailing_gap = (
            moving_downward
            and predicted_shuttle_y <= crossing_y - 7.5)
        complete_hull_already_clear = (
            float(self.patrol_direction[5]) > 0.0
            and float(shuttle.position[1]) <= 0.0)
        safe_trailing_gap = (
            predicted_trailing_gap or complete_hull_already_clear)
        if predicted_leading_gap or safe_trailing_gap:
            # Four seconds is enough to reach the monotonic x=20 commitment
            # boundary, but unlike the old 14 s latch it cannot preserve a
            # stale permit through a complete shuttle phase change.
            self.wamv1_shuttle_commit_until = now + 4.0
            return "commit"
        return "yield"

    def harbor_wamv2_initial_shuttle_right_of_way(self):
        """Keep wamv_2 out of wamv_5's upper turnaround until a real gap."""
        vessel = self.vessels[1]
        if (self.encounter_mode != "harbor"
                or int(self.harbor_route_index[1]) > 1
                or float(vessel.position[0]) >= -18.0):
            return "none"

        now = self.now_seconds()
        if self.wamv2_initial_crossing_committed:
            return "commit"

        shuttle = self.vessels[4]
        shuttle_y = float(shuttle.position[1])
        shuttle_vy = float(shuttle.velocity[1])

        # Two valid windows exist.  Before arrival, wamv_5 must still be far
        # enough below the crossing for wamv_2 to clear it.  After turnaround,
        # its complete hull must be moving down and already below the lane.
        gap_before_arrival = shuttle_y <= 6.0 and shuttle_vy >= -0.10
        gap_after_departure = shuttle_y <= 15.0 and shuttle_vy <= -0.20
        if gap_before_arrival or gap_after_departure:
            self.wamv2_initial_commit_until = now + 12.0
            self.wamv2_initial_crossing_committed = True
            return "commit"
        return "yield"

    @staticmethod
    def oriented_rectangle_clearance(first_position, first_heading,
                                     first_half_size, second_position,
                                     second_heading, second_half_size):
        """Conservative signed SAT clearance for two oriented rectangles."""
        first_forward = np.array([
            math.cos(first_heading), math.sin(first_heading)])
        first_lateral = np.array([-first_forward[1], first_forward[0]])
        second_forward = np.array([
            math.cos(second_heading), math.sin(second_heading)])
        second_lateral = np.array([-second_forward[1], second_forward[0]])
        relative = second_position - first_position
        gaps = []
        for axis in (first_forward, first_lateral,
                     second_forward, second_lateral):
            first_radius = (
                first_half_size[0] * abs(float(np.dot(first_forward, axis)))
                + first_half_size[1] * abs(float(np.dot(first_lateral, axis))))
            second_radius = (
                second_half_size[0] * abs(float(np.dot(second_forward, axis)))
                + second_half_size[1] * abs(float(np.dot(second_lateral, axis))))
            gaps.append(abs(float(np.dot(relative, axis)))
                        - first_radius - second_radius)
        return max(gaps)

    def command_heading_speed(self, vessel, desired_heading, desired_speed):
        """Track the MATLAB heading-speed action using measured VRX feedback."""
        goal_distance = float(np.linalg.norm(vessel.goal - vessel.position))
        if goal_distance <= self.goal_tolerance:
            vessel.reached = True
            return 0.0, 0.0, goal_distance
        hull_forward = np.array([
            math.cos(vessel.heading), math.sin(vessel.heading)])
        # Differential thrust controls surge. Cross-current drift must not be
        # mistaken for useful forward speed by the longitudinal controller.
        speed = max(0.0, float(np.dot(vessel.velocity, hull_forward)))
        heading_error = wrap_angle(desired_heading - vessel.heading)
        desired_speed = float(np.clip(desired_speed, 0.0, 2.0))
        feedforward = self.cruise * desired_speed / 2.0
        forward = float(np.clip(
            feedforward + 85.0 * (desired_speed - speed), 0.0, self.cruise))
        forward *= max(0.08, math.cos(heading_error))
        turn = float(np.clip(
            220.0 * heading_error, -self.max_turn, self.max_turn))
        return forward - turn, forward + turn, goal_distance

    def harbor_residual_observation(self, plans):
        """Reproduce the V6 80-D observation used during SAC training."""
        cooperative = self.vessels[:self.cooperative_count]
        dynamic = self.vessels[
            self.cooperative_count:self.cooperative_count + 2]
        entity_positions = [v.position for v in cooperative]
        entity_positions.extend(
            np.asarray(center, dtype=float)
            for center, _ in self.static_rectangles)
        entity_positions.extend(v.position for v in dynamic)
        entity_positions = np.asarray(entity_positions, dtype=float)

        edge_probability = np.zeros((4, 8), dtype=float)
        for ego in range(min(4, self.cooperative_count)):
            for edge in self.graph_edges.get(ego, []):
                neighbor = int(edge["neighbor"])
                if neighbor < self.cooperative_count:
                    column = neighbor
                else:
                    column = 6 + neighbor - self.cooperative_count
                if 0 <= column < 8:
                    edge_probability[ego, column] = max(
                        float(edge.get("critical_probability", 0.0)),
                        float(edge.get("risk", 0.0)),
                    )

        sea_noise = {
            "calm": 0.015,
            "medium_wind_wave": 0.06,
            "moderate": 0.06,
            "strong_wind_wave": 0.12,
            "strong": 0.12,
            "extreme_wind_wave": 0.20,
            "extreme": 0.20,
        }.get(
            self.sea_state,
            0.20 * float(np.clip(self.environment_uncertainty, 0.0, 1.0)),
        )

        rows = []
        for index, vessel in enumerate(cooperative[:4]):
            base_heading, base_speed = plans[index][5], plans[index][6]
            c, s = math.cos(vessel.heading), math.sin(vessel.heading)
            world_to_body = np.array([[c, s], [-s, c]])
            goal_body = world_to_body @ (vessel.goal - vessel.position)
            target_body = world_to_body @ (
                self.harbor_reference_target(index) - vessel.position)
            hull_half = np.array([2.7, 1.35])
            pair_clearance = math.inf
            obstacle_clearance = math.inf
            for other_index, other in enumerate(cooperative[:4]):
                if other_index == index:
                    continue
                pair_clearance = min(
                    pair_clearance,
                    self.oriented_rectangle_clearance(
                        vessel.position, vessel.heading, hull_half,
                        other.position, other.heading, hull_half))
            for center, half_size in self.static_rectangles:
                obstacle_clearance = min(
                    obstacle_clearance,
                    self.oriented_rectangle_clearance(
                        vessel.position, vessel.heading, hull_half,
                        np.asarray(center, dtype=float), 0.0,
                        np.asarray(half_size, dtype=float)))
            for other in dynamic:
                obstacle_clearance = min(
                    obstacle_clearance,
                    self.oriented_rectangle_clearance(
                        vessel.position, vessel.heading, hull_half,
                        other.position, other.heading, hull_half))
            speed = float(np.linalg.norm(vessel.velocity))
            rows.extend([
                goal_body[0] / 100.0,
                goal_body[1] / 100.0,
                target_body[0] / 100.0,
                target_body[1] / 100.0,
                speed / 2.4,
                math.sin(wrap_angle(base_heading - vessel.heading)),
                base_speed / 2.4,
                float(np.clip(pair_clearance, -10.0, 50.0)) / 50.0,
                float(np.clip(obstacle_clearance, -10.0, 50.0)) / 50.0,
                float(np.max(edge_probability[index])),
                float(vessel.reached),
                sea_noise / 0.20,
            ])
        rows.extend(edge_probability.reshape(-1).tolist())
        return np.asarray(rows, dtype=float)

    def harbor_v6_shield_threshold(self):
        noise = {
            "calm": 0.015,
            "medium_wind_wave": 0.06,
            "moderate": 0.06,
            "strong_wind_wave": 0.12,
            "strong": 0.12,
            "extreme_wind_wave": 0.20,
            "extreme": 0.20,
        }.get(self.sea_state, 0.20 * self.environment_uncertainty)
        extreme_tail = 0.9 if self.sea_state in {
            "extreme", "extreme_wind_wave"} else 0.0
        return 0.8 + 4.0 * noise + extreme_tail

    def harbor_v6_predict_joint_minimum(self, commands):
        """V6 joint hull / wall / shuttle prediction used by the safety gate."""
        cooperative = self.vessels[:4]
        positions = np.stack([v.position for v in cooperative]).astype(float)
        headings = np.asarray([v.heading for v in cooperative], dtype=float)
        speeds = np.asarray([
            np.linalg.norm(v.velocity) for v in cooperative], dtype=float)
        commands = np.asarray(commands, dtype=float)
        hull_half = np.array([2.7, 1.35])
        minimum = math.inf
        for step in range(1, 13):
            heading_error = np.arctan2(
                np.sin(commands[:, 0] - headings),
                np.cos(commands[:, 0] - headings))
            headings += np.clip(heading_error, -0.19, 0.19)
            headings = np.arctan2(np.sin(headings), np.cos(headings))
            speeds += np.clip(commands[:, 1] - speeds, -0.22, 0.16)
            positions += 0.5 * speeds[:, None] * np.c_[
                np.cos(headings), np.sin(headings)]
            prediction_time = 0.5 * step
            for first in range(4):
                for second in range(first + 1, 4):
                    minimum = min(
                        minimum,
                        self.oriented_rectangle_clearance(
                            positions[first], headings[first], hull_half,
                            positions[second], headings[second], hull_half))
                for center, half_size in self.static_rectangles:
                    predicted_half_size = (
                        np.asarray(half_size, dtype=float)
                        + self.static_wall_prediction_padding)
                    minimum = min(
                        minimum,
                        self.oriented_rectangle_clearance(
                            positions[first], headings[first], hull_half,
                            np.asarray(center, dtype=float), 0.0,
                            predicted_half_size))
                for other_index in range(4, min(6, len(self.vessels))):
                    other = self.vessels[other_index]
                    predicted = self.predicted_patrol_position(
                        other_index, prediction_time)
                    minimum = min(
                        minimum,
                        self.oriented_rectangle_clearance(
                            positions[first], headings[first], hull_half,
                            predicted, other.heading, hull_half))
        return float(minimum)

    def harbor_v6_command_score(self, commands, predicted_clearance=None):
        commands = np.asarray(commands, dtype=float)
        positions = np.stack([v.position for v in self.vessels[:4]])
        headings = np.asarray([v.heading for v in self.vessels[:4]])
        speeds = np.asarray([
            np.linalg.norm(v.velocity) for v in self.vessels[:4]])
        targets = np.stack([
            self.harbor_reference_target(i) for i in range(4)])
        target_headings = np.arctan2(
            targets[:, 1] - positions[:, 1],
            targets[:, 0] - positions[:, 0])
        errors = np.arctan2(
            np.sin(commands[:, 0] - target_headings),
            np.cos(commands[:, 0] - target_headings))
        active = np.asarray([not v.reached for v in self.vessels[:4]])
        progress = commands[active, 1] * np.cos(errors[active])
        if progress.size == 0:
            return 0.0
        if predicted_clearance is None:
            predicted_clearance = self.harbor_v6_predict_joint_minimum(commands)
        clearance_credit = min(
            predicted_clearance, self.harbor_v6_shield_threshold() + 4.0)
        heading_delta = np.arctan2(
            np.sin(commands[:, 0] - headings),
            np.cos(commands[:, 0] - headings))
        return float(
            np.mean(progress) + 0.50 * np.min(progress)
            + 0.12 * clearance_credit
            - 0.08 * np.mean(np.abs(heading_delta))
            - 0.04 * np.mean(np.abs(commands[:, 1] - speeds)))

    def harbor_v6_project_residual(self, plans, actions):
        """Accept a residual only when joint safety and predicted value improve."""
        base = np.asarray([
            [plans[i][5], plans[i][6]] for i in range(4)], dtype=float)
        residual = np.c_[
            self.residual_sac_actor.heading_scale * actions[:, 0],
            self.residual_sac_actor.speed_scale * actions[:, 1],
        ]
        if float(np.linalg.norm(residual)) <= 1e-7:
            return base, 0.0, 0.0
        base_score = self.harbor_v6_command_score(base)
        threshold = self.harbor_v6_shield_threshold()
        for scale in (1.0, 0.75, 0.5, 0.25, 0.0):
            candidate = base.copy()
            candidate[:, 0] += scale * residual[:, 0]
            candidate[:, 0] = np.arctan2(
                np.sin(candidate[:, 0]), np.cos(candidate[:, 0]))
            candidate[:, 1] = np.clip(
                candidate[:, 1] + scale * residual[:, 1], 0.0, 2.0)
            for index, vessel in enumerate(self.vessels[:4]):
                if vessel.reached:
                    candidate[index, 1] = 0.0
            minimum = self.harbor_v6_predict_joint_minimum(candidate)
            advantage = self.harbor_v6_command_score(
                candidate, minimum) - base_score
            if minimum >= threshold and (
                    scale <= 1e-6 or advantage >= 0.01):
                return candidate, scale, max(0.0, advantage)
        return base, 0.0, 0.0

    def residual_safety_scale(self, index, predicted_clearance, threats):
        """Fade SAC authority near hull, wall, or graph safety limits."""
        vessel = self.vessels[index]
        hull_half_size = np.array([2.7, 1.35])
        clearance = float(predicted_clearance)
        for other_index, other in enumerate(self.vessels):
            if other_index == index:
                continue
            clearance = min(clearance, self.oriented_rectangle_clearance(
                vessel.position, vessel.heading, hull_half_size,
                other.position, other.heading, hull_half_size))
        for center, half_size in self.static_rectangles:
            predicted_half_size = (
                np.asarray(half_size, dtype=float)
                + self.static_wall_prediction_padding)
            clearance = min(clearance, self.oriented_rectangle_clearance(
                vessel.position, vessel.heading, hull_half_size,
                np.asarray(center, dtype=float), 0.0,
                predicted_half_size))
        if any(bool(edge.get("hard_safety", False)) for edge in threats):
            return 0.0
        return float(np.clip((clearance + 0.5) / 3.5, 0.0, 1.0))

    def harbor_graph_shield_command(self, index, threats):
        """Port the MATLAB GNN-GRU plus finite-horizon expert safety actor."""
        vessel = self.vessels[index]
        goal_offset = vessel.goal - vessel.position
        goal_distance = float(np.linalg.norm(goal_offset))
        if goal_distance <= self.goal_tolerance:
            vessel.reached = True
            return (0.0, 0.0, goal_distance, 0.0, 0.0,
                    vessel.heading, 0.0)

        reference_target = self.harbor_reference_target(index)
        reference_offset = reference_target - vessel.position
        reference_distance = float(np.linalg.norm(reference_offset))
        reference_heading = math.atan2(reference_offset[1], reference_offset[0])
        reference_direction = reference_offset / max(reference_distance, 1e-9)
        right_direction = np.array([
            reference_direction[1], -reference_direction[0]])
        graph_vector = reference_direction.copy()
        graph_speed = 1.0
        now = self.now_seconds()
        wamv1_shuttle_mode = (
            self.harbor_wamv1_shuttle_right_of_way()
            if index == 0 else "none")
        wamv2_initial_shuttle_mode = (
            self.harbor_wamv2_initial_shuttle_right_of_way()
            if index == 1 else "none")
        if wamv1_shuttle_mode == "yield":
            # Wait outside the shuttle lane rather than braking in its path.
            # A low-speed station correction counters wind drift while keeping
            # the hull near the safe holding point west of the crossing.
            holding_point = np.array([15.0, 8.0])
            holding_offset = holding_point - vessel.position
            holding_distance = float(np.linalg.norm(holding_offset))
            holding_heading = (
                math.atan2(holding_offset[1], holding_offset[0])
                if holding_distance > 0.35 else vessel.heading)
            holding_speed = 0.0 if holding_distance <= 1.25 else min(
                0.65, 0.20 + 0.12 * holding_distance)
            left, right, distance = self.command_heading_speed(
                vessel, holding_heading, holding_speed)
            self.yielding[index] = True
            self.emergency_braking[index] = True
            self.last_predicted_clearance[index] = math.nan
            return (left, right, distance,
                    wrap_angle(holding_heading - reference_heading),
                    holding_speed / 2.0, holding_heading, holding_speed)

        if wamv2_initial_shuttle_mode == "yield":
            # Hold west of wamv_5's patrol lane and above the static wall.
            # Do not retreat to an older waypoint after entering the hold.
            holding_point = np.array([-30.5, 25.5])
            holding_offset = holding_point - vessel.position
            holding_distance = float(np.linalg.norm(holding_offset))
            holding_heading = (
                math.atan2(holding_offset[1], holding_offset[0])
                if holding_distance > 0.35 else vessel.heading)
            holding_speed = 0.0 if holding_distance <= 1.0 else min(
                0.60, 0.18 + 0.12 * holding_distance)
            left, right, distance = self.command_heading_speed(
                vessel, holding_heading, holding_speed)
            self.yielding[index] = True
            self.emergency_braking[index] = True
            self.last_predicted_clearance[index] = math.nan
            return (left, right, distance,
                    wrap_angle(holding_heading - reference_heading),
                    holding_speed / 2.0, holding_heading, holding_speed)

        wamv1_shuttle_commit = wamv1_shuttle_mode == "commit"
        wamv2_initial_shuttle_commit = (
            wamv2_initial_shuttle_mode == "commit")
        # C103: after a safe initial gap has been accepted, crossing the
        # wamv_5 lane is monotonic. The old unrestricted heading search could
        # choose a large port turn while the shuttle closed from astern,
        # leaving both hulls travelling together instead of clearing the
        # crossing. Aim at the next upper-corridor point and release this lock
        # as soon as the complete vessel has crossed x=-18 m.
        wamv2_initial_escape = (
            index == 1
            and wamv2_initial_shuttle_commit
            and int(self.harbor_route_index[index]) <= 1
            and float(vessel.position[0]) < -18.0
        )
        # C119: make a one-way spatial transition into the crossing. No fixed
        # staging waypoint or elapsed-time window is involved. If the live
        # gap is closed, station-keep at the position where this phase began;
        # this can never command a return toward an older point. Once the gap
        # opens, travel directly to the lane-clear point and remain latched
        # until the complete hull clears.
        wamv2_final_approach = (
            index == 1
            and int(self.harbor_route_index[index]) in (4, 5)
            and float(vessel.position[0]) < 31.0
        )
        if wamv2_final_approach and not self.wamv2_final_horizontal_committed:
            if self.wamv2_final_hold_position is None:
                self.wamv2_final_hold_position = vessel.position.copy()
            holding_point = self.wamv2_final_hold_position
            holding_offset = holding_point - vessel.position
            holding_distance = float(np.linalg.norm(holding_offset))
            if self.harbor_wamv2_crossing_gap_open(
                    np.asarray((31.0, 13.0), dtype=float), 2.0):
                self.wamv2_final_horizontal_committed = True
                self.wamv2_final_hold_position = None
            else:
                holding_heading = (
                    math.atan2(holding_offset[1], holding_offset[0])
                    if holding_distance > 0.35 else vessel.heading)
                holding_speed = (
                    0.0 if holding_distance <= 0.75 else
                    min(0.75, 0.20 + 0.14 * holding_distance))
                left, right, distance = self.command_heading_speed(
                    vessel, holding_heading, holding_speed)
                self.yielding[index] = True
                self.emergency_braking[index] = True
                self.last_predicted_clearance[index] = math.nan
                return (left, right, distance,
                        wrap_angle(holding_heading - reference_heading),
                        holding_speed / 2.0, holding_heading, holding_speed)
        wamv2_crossing_commit = (
            index == 1
            and self.wamv2_final_horizontal_committed
            and float(vessel.position[0]) < 31.0
        )
        for threat in threats:
            # Candidate C49: do not let a distant predicted shuttle conflict
            # suppress the only speed that can clear the first crossing. The
            # candidate-level hull simulation below still rejects overlap.
            if self.harbor_forward_gap_open(
                    index, int(threat["neighbor"]), reference_target, 2.0):
                continue
            if (wamv2_crossing_commit
                    and int(threat["neighbor"]) == 5):
                continue
            if (wamv1_shuttle_commit
                    and int(threat["neighbor"]) == 5):
                continue
            if (wamv2_initial_shuttle_commit
                    and int(threat["neighbor"]) == 4):
                continue
            neighbor = self.vessels[threat["neighbor"]]
            relative = neighbor.position - vessel.position
            separation = max(float(np.linalg.norm(relative)), 1e-6)
            bearing = relative / separation
            urgency = float(threat["risk"]) * (
                0.45 + 0.55 * math.exp(-max(0.0, threat["tcpa"]) / 5.0))
            urgency = min(1.4, 1.5 * urgency)
            graph_vector += urgency * (2.2 * right_direction - 0.8 * bearing)
            graph_speed = min(graph_speed, max(0.15, 1.0 - 0.85 * urgency))
        graph_heading = math.atan2(graph_vector[1], graph_vector[0])
        graph_offset = wrap_angle(graph_heading - reference_heading)

        maximum_route_risk = max(
            (float(threat["risk"]) for threat in threats), default=0.0)
        hard_route_threat = any(
            bool(threat.get("hard_safety", False)) for threat in threats)
        if (index == 0 and int(self.harbor_route_index[index]) >= 1
                and maximum_route_risk >= 0.55):
            self.wamv1_route_conflict_seen = True
        wamv1_post_conflict_recovery = (
            index == 0
            and int(self.harbor_route_index[index]) >= 1
            and self.wamv1_route_conflict_seen
            and maximum_route_risk < 0.30
            and not hard_route_threat
        )

        # Match the MATLAB safety actor: 21 heading choices, six speed
        # choices, finite turn rate / acceleration and a 14 s horizon.
        candidate_headings = [
            wrap_angle(reference_heading + math.radians(value))
            for value in range(-100, 101, 10)
        ]
        candidate_speeds = (0.0, 0.45, 0.85, 1.25, 1.65, 2.0)
        if wamv2_initial_escape:
            escape_target = np.array([-14.0, 25.5])
            escape_offset = escape_target - vessel.position
            escape_heading = math.atan2(escape_offset[1], escape_offset[0])
            candidate_headings = [
                wrap_angle(escape_heading + math.radians(value))
                for value in (-15, -10, -5, 0, 5, 10, 15)
            ]
            candidate_speeds = (1.65, 2.0)
        times = np.arange(0.0, 14.0 + 1e-9, 0.75)
        current_speed = min(2.4, float(np.linalg.norm(vessel.velocity)))
        max_turn_rate = math.radians(22.0)
        max_accel = 0.65
        hull_half_size = np.array([2.7, 1.35])
        candidates = []

        for heading in candidate_headings:
            offset = wrap_angle(heading - reference_heading)
            for desired_speed in candidate_speeds:
                minimum_clearance = math.inf
                integrated_violation = 0.0
                collision_penalty = 0.0
                wall_comfort_penalty = 0.0
                wamv2_outer_corridor_penalty = 0.0
                wamv1_commit_lane_penalty = 0.0
                final_position = vessel.position.copy()
                open_gap_neighbors = {
                    other_index
                    for other_index in range(
                        self.cooperative_count, len(self.vessels))
                    if self.harbor_forward_gap_open(
                        index, other_index, reference_target, desired_speed)
                }
                if wamv2_crossing_commit and desired_speed >= 1.65:
                    open_gap_neighbors.add(5)
                if wamv1_shuttle_commit and desired_speed >= 1.65:
                    open_gap_neighbors.add(5)
                if wamv2_initial_shuttle_commit and desired_speed >= 1.65:
                    open_gap_neighbors.add(4)

                # Candidate C05: preserve C01's pair prediction and decision
                # logic, but project static-wall clearance with measured sway
                # and yaw dynamics. This captures the stern sweep between the
                # commanded heading and the hull's actual turning response.
                wall_position = vessel.position.copy()
                wall_velocity = vessel.velocity.copy()
                wall_heading = vessel.heading
                wall_yaw_rate = vessel.yaw_rate
                wall_speed = current_speed
                previous_wall_time = 0.0

                for prediction_time in times:
                    requested_turn = wrap_angle(heading - vessel.heading)
                    realized_turn = float(np.clip(
                        requested_turn,
                        -max_turn_rate * prediction_time,
                        max_turn_rate * prediction_time))
                    predicted_heading = wrap_angle(vessel.heading + realized_turn)
                    speed_change = float(np.clip(
                        desired_speed - current_speed,
                        -max_accel * prediction_time,
                        max_accel * prediction_time))
                    predicted_speed = float(np.clip(
                        current_speed + speed_change, 0.0, 2.4))
                    mean_heading = wrap_angle(vessel.heading + 0.5 * realized_turn)
                    mean_speed = 0.5 * (current_speed + predicted_speed)
                    predicted_position = vessel.position + prediction_time * mean_speed * np.array([
                        math.cos(mean_heading), math.sin(mean_heading)])
                    final_position = predicted_position

                    wall_dt = float(prediction_time - previous_wall_time)
                    if wall_dt > 0.0:
                        wall_heading_error = wrap_angle(heading - wall_heading)
                        desired_wall_yaw_rate = float(np.clip(
                            0.9 * wall_heading_error,
                            -max_turn_rate, max_turn_rate))
                        yaw_response = 1.0 - math.exp(-wall_dt / 0.8)
                        wall_yaw_rate += yaw_response * (
                            desired_wall_yaw_rate - wall_yaw_rate)
                        wall_heading = wrap_angle(
                            wall_heading + wall_dt * wall_yaw_rate)
                        wall_speed += float(np.clip(
                            desired_speed - wall_speed,
                            -max_accel * wall_dt,
                            max_accel * wall_dt))
                        wall_speed = float(np.clip(wall_speed, 0.0, 2.4))
                        wall_commanded_velocity = wall_speed * np.array([
                            math.cos(wall_heading), math.sin(wall_heading)])
                        velocity_response = 1.0 - math.exp(-wall_dt / 2.0)
                        wall_velocity += velocity_response * (
                            wall_commanded_velocity - wall_velocity)
                        wall_position += wall_dt * wall_velocity
                    previous_wall_time = float(prediction_time)

                    if wamv1_shuttle_commit:
                        # C99: preserve the C95/C98 release timing, but reject
                        # inward sag toward wamv_6 while wamv_1 is committed
                        # to the final crossing. Open water above y=8 remains
                        # unpenalized; all physical collision checks remain.
                        inward_lane_error = max(
                            0.0, 8.0 - float(wall_position[1]))
                        wamv1_commit_lane_penalty += (
                            math.exp(-0.12 * prediction_time)
                            * inward_lane_error ** 2)

                    clearance_margins = []
                    for other_index, other in enumerate(self.vessels):
                        if other_index == index:
                            continue
                        if other_index >= self.cooperative_count:
                            other_position = self.predicted_patrol_position(
                                other_index, prediction_time)
                            # A verified forward gap relaxes only the early
                            # comfort margin. Negative physical hull clearance
                            # still receives the full collision penalty.
                            safety_margin = (
                                1.0 if other_index in open_gap_neighbors
                                else 4.5)
                        else:
                            other_position = (
                                other.position + prediction_time * other.velocity)
                            safety_margin = 3.0
                        clearance = self.oriented_rectangle_clearance(
                            predicted_position, predicted_heading, hull_half_size,
                            other_position, other.heading, hull_half_size)
                        clearance_margins.append((clearance, safety_margin))
                    for center, half_size in self.static_rectangles:
                        predicted_half_size = (
                            np.asarray(half_size, dtype=float)
                            + self.static_wall_prediction_padding)
                        if index == 0 and float(center[1]) < 0.0:
                            # C117: the second C116 repeat showed one 8.9 cm
                            # stern-corner overlap here. Add only a local
                            # prediction margin; physical geometry and routes
                            # remain unchanged.
                            predicted_half_size += np.array([0.18, 0.18])
                        clearance = self.oriented_rectangle_clearance(
                            wall_position, wall_heading, hull_half_size,
                            np.asarray(center, dtype=float), 0.0,
                            predicted_half_size)
                        clearance_margins.append((clearance, 3.0))
                        if index == 1:
                            # The upper-left route is geometrically valid but
                            # previously treated a 3 m wall pass as equally
                            # attractive as open water. Add a soft continuous
                            # comfort cost without changing the hard wall or
                            # the route used by the other vessels.
                            comfort_violation = max(0.0, 4.8 - clearance)
                            wall_comfort_penalty += (
                                math.exp(-0.12 * prediction_time)
                                * comfort_violation ** 2)
                            if (int(self.harbor_route_index[index]) <= 2
                                    and float(center[1]) > 0.0):
                                # Keep the complete oriented hull above the
                                # wall with a real 3 m buffer. The threshold
                                # adapts to heading because a turning hull has
                                # a larger vertical projection than a straight
                                # one. This is one-sided: being farther out is
                                # never penalized.
                                lateral_extent = (
                                    abs(math.sin(wall_heading))
                                    * float(hull_half_size[0])
                                    + abs(math.cos(wall_heading))
                                    * float(hull_half_size[1]))
                                safe_center_y = (
                                    float(center[1]) + float(half_size[1])
                                    + lateral_extent + 3.0)
                                outward_deficit = max(
                                    0.0, safe_center_y
                                    - float(wall_position[1]))
                                wamv2_outer_corridor_penalty += (
                                    math.exp(-0.12 * prediction_time)
                                    * outward_deficit ** 2)

                                # Track the fixed y=25.5 corridor after the
                                # corner turn.  Inward error (toward the wall)
                                # is deliberately much more expensive than an
                                # equal outward error, so the optimizer cannot
                                # shorten the path by leaning back into the
                                # obstacle while carrying crosswind momentum.
                                if int(self.harbor_route_index[index]) >= 1:
                                    corridor_y = 25.5
                                    inward_error = max(
                                        0.0, corridor_y
                                        - float(wall_position[1]))
                                    outward_error = max(
                                        0.0, float(wall_position[1])
                                        - corridor_y)
                                    wamv2_outer_corridor_penalty += (
                                        math.exp(-0.12 * prediction_time)
                                        * (1.5 * inward_error ** 2
                                           + 0.12 * outward_error ** 2))

                    step_clearance = min(
                        clearance for clearance, _ in clearance_margins)
                    minimum_clearance = min(minimum_clearance, step_clearance)
                    for clearance, safety_margin in clearance_margins:
                        violation = max(0.0, safety_margin - clearance)
                        integrated_violation += (
                            math.exp(-0.16 * prediction_time) * violation ** 2)
                        if clearance < 0.0:
                            collision_penalty += (
                                2500.0 * math.exp(-0.20 * prediction_time))

                final_goal_distance = float(np.linalg.norm(
                    vessel.goal - final_position))
                target_distance = float(np.linalg.norm(
                    reference_target - final_position))
                route_progress = float(np.dot(
                    final_position - vessel.position, reference_direction))
                reverse_progress_penalty = 40.0 * max(0.0, -route_progress) ** 2
                turn = abs(wrap_angle(heading - vessel.heading))
                port_turn = max(0.0, wrap_angle(heading - reference_heading))
                low_speed_penalty = 7.0 * max(0.0, 2.0 - desired_speed)
                graph_disagreement = abs(wrap_angle(heading - graph_heading))
                post_conflict_recovery_cost = 0.0
                if wamv1_post_conflict_recovery:
                    # Once a conflict has cleared, prefer prompt forward
                    # progress along the unchanged C60 route.  Collision,
                    # wall and finite-horizon costs still dominate this soft
                    # term, so it cannot authorize an unsafe candidate.
                    speed_deficit = max(0.0, 1.65 - desired_speed)
                    post_conflict_recovery_cost = (
                        8.0 * speed_deficit ** 2
                        + 1.8 * abs(offset) ** 2
                        - 0.9 * route_progress
                    )
                wamv2_crossing_commit_cost = 0.0
                if wamv2_crossing_commit:
                    # Commit to clearing the lane promptly. Real overlap and
                    # wall penalties remain orders of magnitude larger, so an
                    # unsafe action still loses to a safe slower alternative.
                    speed_deficit = max(0.0, 1.65 - desired_speed)
                    wamv2_crossing_commit_cost = (
                        36.0 * speed_deficit ** 2
                        + 3.0 * abs(offset) ** 2
                        - 1.4 * route_progress
                    )
                wamv1_crossing_commit_cost = 0.0
                if wamv1_shuttle_commit:
                    speed_deficit = max(0.0, 1.65 - desired_speed)
                    wamv1_crossing_commit_cost = (
                        30.0 * speed_deficit ** 2
                        + 2.5 * abs(offset) ** 2
                        - 1.2 * route_progress
                    )
                wamv2_initial_commit_cost = 0.0
                if wamv2_initial_shuttle_commit:
                    speed_deficit = max(0.0, 1.65 - desired_speed)
                    wamv2_initial_commit_cost = (
                        32.0 * speed_deficit ** 2
                        + 2.5 * abs(offset) ** 2
                        - 1.3 * route_progress
                    )
                wamv2_initial_escape_cost = 0.0
                if wamv2_initial_escape:
                    east_progress = float(
                        final_position[0] - vessel.position[0])
                    escape_error = abs(wrap_angle(
                        heading - escape_heading))
                    speed_deficit = max(0.0, 1.85 - desired_speed)
                    wamv2_initial_escape_cost = (
                        24.0 * escape_error ** 2
                        + 28.0 * speed_deficit ** 2
                        - 2.0 * east_progress
                    )
                score = (
                    collision_penalty + 22.0 * integrated_violation
                    + 0.20 * final_goal_distance + 0.85 * target_distance
                    + reverse_progress_penalty
                    + 1.2 * turn ** 2 + 0.35 * port_turn ** 2
                    + low_speed_penalty + 0.30 * graph_disagreement ** 2
                    + 2.0 * wall_comfort_penalty
                    + 18.0 * wamv2_outer_corridor_penalty
                    + 1.5 * wamv1_commit_lane_penalty
                    + post_conflict_recovery_cost
                    + wamv2_crossing_commit_cost
                    + wamv1_crossing_commit_cost
                    + wamv2_initial_commit_cost
                    + wamv2_initial_escape_cost
                    - 0.35 * minimum_clearance
                )
                candidates.append({
                    "heading": heading,
                    "offset": offset,
                    "desired_speed": desired_speed,
                    "minimum_clearance": minimum_clearance,
                    "score": score,
                })

        choice = min(candidates, key=lambda candidate: candidate["score"])
        self.last_predicted_clearance[index] = choice["minimum_clearance"]
        self.emergency_braking[index] = choice["minimum_clearance"] < 1.5

        self.previous_avoid_offset[index] = choice["offset"]
        left, right, distance = self.command_heading_speed(
            vessel, choice["heading"], choice["desired_speed"])
        speed_scale = choice["desired_speed"] / 2.0
        return (left, right, distance, choice["offset"], speed_scale,
                choice["heading"], choice["desired_speed"])

    def counterfactual_safe_command(self, index, threats):
        vessel = self.vessels[index]
        goal_offset = vessel.goal - vessel.position
        goal_distance = float(np.linalg.norm(goal_offset))
        coordinated = self.graph_mode in (
            "gnn_coordinated_csd", "learned_gnn_gru_graph_shield")
        terminal_protected = self.graph_mode in (
            "gnn_coordinated_csd", "gnn_residual_csd",
            "gnn_uncertainty_residual_csd", "learned_gnn_gru_graph_shield")
        if goal_distance <= self.goal_tolerance:
            vessel.reached = True
            if terminal_protected:
                urgent = []
                for threat in threats:
                    neighbor = self.vessels[threat["neighbor"]]
                    separation = float(np.linalg.norm(
                        vessel.position - neighbor.position))
                    if (threat["hard_safety"] or threat["risk"] >= 0.65) \
                            and separation < 10.0:
                        urgent.append((separation, neighbor, threat))
                if urgent:
                    separation, neighbor, threat = min(urgent, key=lambda item: item[0])
                    away = vessel.position - neighbor.position
                    away_heading = math.atan2(away[1], away[0])
                    speed_scale = float(np.clip(
                        0.25 + 0.25 * (10.0 - separation) / 5.0, 0.25, 0.50))
                    left, right, distance = self.command_heading(
                        vessel, away_heading, speed_scale,
                        allow_goal_evasion=True)
                    offset = wrap_angle(away_heading - vessel.heading)
                    self.previous_avoid_offset[index] = offset
                    return left, right, distance, offset, speed_scale
            return 0.0, 0.0, goal_distance, 0.0, 0.0

        goal_heading = math.atan2(goal_offset[1], goal_offset[0])
        offsets = np.radians([0.0, -25.0, 25.0, -45.0, 45.0, -70.0, 70.0])
        close_hard_risk = coordinated and any(
            threat["hard_safety"]
            and np.linalg.norm(
                self.vessels[threat["neighbor"]].position - vessel.position
            ) < 10.0
            for threat in threats
        )
        speed_scales = (
            (0.50, 0.25, 0.0) if close_hard_risk
            else (1.0, 0.75, 0.50, 0.25, 0.0)
        )
        times = np.arange(0.0, self.prediction_horizon + 1e-9, 0.5)
        candidates = []
        nominal_speed = max(1.8, min(2.8, float(np.linalg.norm(vessel.velocity))))

        for heading_offset in offsets:
            heading = goal_heading + float(heading_offset)
            direction = np.array([math.cos(heading), math.sin(heading)])
            for speed_scale in speed_scales:
                ego_velocity = nominal_speed * speed_scale * direction
                ego_future = vessel.position[None, :] + times[:, None] * ego_velocity[None, :]
                minimum_clearance = math.inf
                for threat in threats:
                    neighbor = self.vessels[threat["neighbor"]]
                    neighbor_future = (
                        neighbor.position[None, :]
                        + times[:, None] * neighbor.velocity[None, :]
                    )
                    separation = np.linalg.norm(neighbor_future - ego_future, axis=1)
                    minimum_clearance = min(
                        minimum_clearance,
                        float(np.min(separation - self.combined_safety_radius)),
                    )
                progress = speed_scale * max(0.0, math.cos(float(heading_offset)))
                smoothness = abs(float(heading_offset) - self.previous_avoid_offset[index])
                candidates.append({
                    "heading": heading,
                    "offset": float(heading_offset),
                    "speed_scale": float(speed_scale),
                    "minimum_clearance": minimum_clearance,
                    "progress": progress,
                    "smoothness": smoothness,
                })

        safe = [candidate for candidate in candidates
                if candidate["minimum_clearance"] >= 2.0]
        if close_hard_risk:
            persistent = [
                candidate for candidate in safe
                if abs(candidate["offset"]) >= math.radians(25.0) - 1e-6
            ]
            if persistent:
                safe = persistent
        if safe:
            choice = max(
                safe,
                key=lambda candidate: (
                    candidate["progress"]
                    - 0.08 * abs(candidate["offset"])
                    - 0.04 * candidate["smoothness"],
                    candidate["minimum_clearance"],
                ),
            )
        else:
            choice = max(
                candidates,
                key=lambda candidate: (
                    candidate["minimum_clearance"],
                    candidate["progress"] - 0.04 * candidate["smoothness"],
                ),
            )

        if self.graph_mode in (
                "gnn_residual_csd", "gnn_uncertainty_residual_csd") and threats:
            advice = threats[0]
            graph_is_low_risk = (
                not any(threat["hard_safety"] for threat in threats)
                and max(threat["risk"] for threat in threats) < 0.65
                and all(key in advice for key in (
                    "advice_sin", "advice_cos", "advice_speed"))
            )
            if graph_is_low_risk:
                self.residual_eligible[index] = True
                authority = (
                    self.residual_authority
                    if self.graph_mode == "gnn_uncertainty_residual_csd"
                    else 1.0
                )
                learned_offset = math.atan2(
                    advice["advice_sin"], advice["advice_cos"])
                learned_speed_scale = float(np.clip(
                    advice["advice_speed"] * 2.4 / nominal_speed, 0.0, 1.0))
                residual_offset = float(np.clip(
                    learned_offset - choice["offset"],
                    -math.radians(12.0), math.radians(12.0)))
                residual_speed = float(np.clip(
                    learned_speed_scale - choice["speed_scale"], -0.15, 0.15))
                residual_offset *= authority
                residual_speed *= authority
                proposed_offset = choice["offset"] + residual_offset
                proposed_speed = float(np.clip(
                    choice["speed_scale"] + residual_speed, 0.0, 1.0))
                proposed_heading = goal_heading + proposed_offset
                proposed_direction = np.array([
                    math.cos(proposed_heading), math.sin(proposed_heading)])
                proposed_velocity = nominal_speed * proposed_speed * proposed_direction
                ego_future = (
                    vessel.position[None, :]
                    + times[:, None] * proposed_velocity[None, :])
                proposed_clearance = math.inf
                for threat in threats:
                    neighbor = self.vessels[threat["neighbor"]]
                    neighbor_future = (
                        neighbor.position[None, :]
                        + times[:, None] * neighbor.velocity[None, :])
                    separation = np.linalg.norm(
                        neighbor_future - ego_future, axis=1)
                    proposed_clearance = min(
                        proposed_clearance,
                        float(np.min(separation - self.combined_safety_radius)),
                    )
                required_clearance = max(
                    2.0, choice["minimum_clearance"] - 0.10)
                if authority > 1e-6 and proposed_clearance >= required_clearance:
                    self.residual_accepted[index] = True
                    self.residual_heading_delta[index] = residual_offset
                    self.residual_speed_delta[index] = residual_speed
                    choice = {
                        **choice,
                        "heading": proposed_heading,
                        "offset": proposed_offset,
                        "speed_scale": proposed_speed,
                        "minimum_clearance": proposed_clearance,
                    }

        self.previous_avoid_offset[index] = choice["offset"]
        left, right, distance = self.command_heading(
            vessel, choice["heading"], choice["speed_scale"])
        return left, right, distance, choice["offset"], choice["speed_scale"]

    def update_yield_state(self, positions):
        count = len(self.vessels)
        nearest = np.full(count, math.inf)
        requested = np.zeros(count, dtype=bool)
        for i in range(count):
            if self.vessels[i].reached:
                continue
            for j in range(i + 1, count):
                if self.vessels[j].reached:
                    continue
                separation = float(np.linalg.norm(positions[j] - positions[i]))
                nearest[i] = min(nearest[i], separation)
                nearest[j] = min(nearest[j], separation)
                threshold = self.resume_distance if self.yielding[j] else self.yield_distance
                if separation < threshold:
                    requested[j] = True
        self.yielding = requested
        return nearest

    def step(self):
        now = self.now_seconds()
        poses_fresh = all(
            v.initialized and now - v.stamp < self.pose_freshness_timeout
            for v in self.vessels)
        if self.experiment_start is None and not poses_fresh:
            for index, vessel in enumerate(self.vessels):
                if (vessel.initialized
                        and now - vessel.stamp < self.pose_freshness_timeout):
                    left, right = self.startup_hold_command(index)
                    self.publish(vessel, left, right)
                else:
                    self.publish(vessel, 0.0, 0.0)
            if (not self.startup_failed
                    and time.monotonic() - self.startup_wall_time >= self.startup_timeout):
                self.startup_failed = True
                details = ", ".join(
                    f"{v.name}:samples={v.valid_start_samples}/"
                    f"{self.required_start_samples},initialized={int(v.initialized)},"
                    f"age={now - v.stamp:.2f}s"
                    for v in self.vessels)
                self.get_logger().error(
                    "STARTUP_VALIDATION_FAILED: did not receive clean poses at "
                    f"the configured starts within {self.startup_timeout:.1f} s; "
                    f"samples={details}")
                rclpy.shutdown()
            return

        if self.experiment_start is not None and not poses_fresh:
            stale = ", ".join(
                f"{v.name}:{now - v.stamp:.2f}s"
                for v in self.vessels
                if now - v.stamp >= self.pose_freshness_timeout)
            self.get_logger().warning(
                f"Runtime pose data stale ({stale}); holding current headings.")
            for index, vessel in enumerate(self.vessels):
                left, right = self.startup_hold_command(index)
                self.publish(vessel, left, right)
            return

        if not self.control_announced:
            self.get_logger().info("All six poses locked; propulsion control is active.")
            self.control_announced = True
            self.experiment_start = now

        positions = np.stack([v.position for v in self.vessels])
        nearest = self.update_yield_state(positions)
        graph_fresh = now - self.graph_stamp <= self.graph_timeout
        cooperative_mission_complete = bool(np.all(
            self.arrival_qualified[:self.cooperative_count]))
        for index, vessel in enumerate(self.vessels):
            self.residual_eligible[index] = False
            self.residual_accepted[index] = False
            self.residual_heading_delta[index] = 0.0
            self.residual_speed_delta[index] = 0.0
            self.last_predicted_clearance[index] = math.nan
            self.emergency_braking[index] = False

        harbor_plans = {}
        residual_actions = None
        residual_commands = None
        residual_joint_scale = 0.0
        residual_predicted_advantage = 0.0
        if (self.encounter_mode == "harbor"
                and self.control_mode == "csd_safe"
                and self.graph_mode == "gnn_gru_residual_sac"):
            for index in range(self.cooperative_count):
                threats = self.graph_edges[index] if graph_fresh else []
                harbor_plans[index] = self.harbor_graph_shield_command(
                    index, threats)
            observation = self.harbor_residual_observation(harbor_plans)
            residual_actions = self.residual_sac_actor.predict(observation)
            if self.residual_sac_actor.format == "vrx_deterministic_sac_actor_v2":
                (residual_commands, residual_joint_scale,
                 residual_predicted_advantage) = self.harbor_v6_project_residual(
                    harbor_plans, residual_actions)

        for index, vessel in enumerate(self.vessels):
            threats = self.graph_edges[index] if graph_fresh else []
            if index >= self.cooperative_count:
                # Prescribed noncooperative traffic never reacts to graph
                # advice; it shuttles along one crossing line without turning
                # its hull around at either endpoint. Once every cooperative
                # vessel has arrived, stop this traffic during the final
                # observation window so the completed scene is unambiguous.
                if cooperative_mission_complete:
                    left, right = 0.0, 0.0
                    goal_distance = float(np.linalg.norm(
                        vessel.goal - vessel.position))
                else:
                    left, right, goal_distance = self.patrol_command(index)
                avoid_offset = 0.0
                command_scale = (
                    0.0 if cooperative_mission_complete
                    else 0.55 * self.patrol_direction[index])
                yielding = False
            elif self.control_mode == "csd_safe" and graph_fresh:
                # In the harbor benchmark every graph construction method uses
                # the same route tracker and finite-horizon hull / wall safety
                # actor. Only the selected graph edges differ between methods.
                if self.encounter_mode == "harbor":
                    plan = harbor_plans.get(index)
                    if plan is None:
                        plan = self.harbor_graph_shield_command(index, threats)
                    (left, right, goal_distance, avoid_offset, command_scale,
                     base_heading, base_speed) = plan
                    if (residual_commands is not None and not vessel.reached
                            and not self.emergency_braking[index]):
                        desired_heading = float(residual_commands[index, 0])
                        desired_speed = float(residual_commands[index, 1])
                        left, right, goal_distance = self.command_heading_speed(
                            vessel, desired_heading, desired_speed)
                        avoid_offset = wrap_angle(
                            desired_heading - math.atan2(
                                vessel.goal[1] - vessel.position[1],
                                vessel.goal[0] - vessel.position[0]))
                        command_scale = desired_speed / 2.0
                        heading_delta = wrap_angle(
                            desired_heading - base_heading)
                        speed_delta = desired_speed - base_speed
                        self.residual_eligible[index] = True
                        self.residual_accepted[index] = bool(
                            residual_joint_scale > 1e-6 and (
                                abs(heading_delta) > 1e-5
                                or abs(speed_delta) > 1e-5))
                        self.residual_heading_delta[index] = heading_delta
                        self.residual_speed_delta[index] = speed_delta
                    elif (residual_actions is not None and not vessel.reached
                            and not self.emergency_braking[index]):
                        safety_scale = self.residual_safety_scale(
                            index, self.last_predicted_clearance[index], threats)
                        authority = self.residual_sac_authority * safety_scale
                        heading_delta = (
                            float(residual_actions[index, 0])
                            * self.residual_sac_actor.heading_scale * authority)
                        speed_delta = (
                            float(residual_actions[index, 1])
                            * self.residual_sac_actor.speed_scale * authority)
                        desired_heading = wrap_angle(
                            base_heading + heading_delta)
                        desired_speed = float(np.clip(
                            base_speed + speed_delta, 0.0, 2.2))
                        left, right, goal_distance = self.command_heading_speed(
                            vessel, desired_heading, desired_speed)
                        avoid_offset = wrap_angle(
                            desired_heading - math.atan2(
                                vessel.goal[1] - vessel.position[1],
                                vessel.goal[0] - vessel.position[0]))
                        command_scale = desired_speed / 2.0
                        self.residual_eligible[index] = safety_scale > 1e-6
                        self.residual_accepted[index] = bool(
                            abs(heading_delta) > 1e-5
                            or abs(speed_delta) > 1e-5)
                        self.residual_heading_delta[index] = heading_delta
                        self.residual_speed_delta[index] = speed_delta
                    # C115: keep C105's original upper-corridor leg, but add
                    # measured-yaw damping so wamv_2 does not overshoot east
                    # and swing its bow toward the wall. A hard vessel threat
                    # retains full authority through C105's safety command.
                    if (index == 1
                            and int(self.harbor_route_index[index]) == 2
                            and not any(bool(threat.get("hard_safety", False))
                                        for threat in threats)):
                        corridor_target = np.asarray((7.0, 25.5), dtype=float)
                        corridor_offset = corridor_target - vessel.position
                        desired_heading = math.atan2(
                            corridor_offset[1], corridor_offset[0])
                        desired_speed = 1.65
                        hull_forward = np.array([
                            math.cos(vessel.heading),
                            math.sin(vessel.heading)])
                        measured_speed = max(0.0, float(np.dot(
                            vessel.velocity, hull_forward)))
                        heading_error = wrap_angle(
                            desired_heading - vessel.heading)
                        feedforward = self.cruise * desired_speed / 2.0
                        forward = float(np.clip(
                            feedforward
                            + 85.0 * (desired_speed - measured_speed),
                            0.0, self.cruise))
                        forward *= max(0.08, math.cos(heading_error))
                        turn_limit = 0.62 * self.max_turn
                        turn = float(np.clip(
                            120.0 * heading_error - 110.0 * vessel.yaw_rate,
                            -turn_limit, turn_limit))
                        left, right = forward - turn, forward + turn
                        goal_distance = float(np.linalg.norm(
                            vessel.goal - vessel.position))
                        avoid_offset = wrap_angle(
                            desired_heading - math.atan2(
                                vessel.goal[1] - vessel.position[1],
                                vessel.goal[0] - vessel.position[0]))
                        command_scale = desired_speed / 2.0
                        self.yielding[index] = False
                    if index == 1:
                        if (self.wamv2_final_horizontal_committed
                                and float(vessel.position[0]) < 31.0):
                            clearance_target = np.asarray(
                                (31.0, 13.0), dtype=float)
                            clearance_offset = (
                                clearance_target - vessel.position)
                            desired_heading = math.atan2(
                                clearance_offset[1], clearance_offset[0])
                            desired_speed = 1.65
                            left, right, goal_distance = (
                                self.command_heading_speed(
                                    vessel, desired_heading, desired_speed))
                            avoid_offset = wrap_angle(
                                desired_heading - math.atan2(
                                    vessel.goal[1] - vessel.position[1],
                                    vessel.goal[0] - vessel.position[0]))
                            command_scale = desired_speed / 2.0
                            self.yielding[index] = False
                        elif (self.wamv2_final_horizontal_committed
                              and float(vessel.position[0]) >= 31.0):
                            self.wamv2_final_horizontal_committed = False
                            self.wamv2_final_hold_position = None
                            self.harbor_route_index[index] = max(
                                6, int(self.harbor_route_index[index]))
                elif threats:
                    if self.graph_mode == "learned_gnn_gru_graph_shield":
                        left, right, goal_distance, avoid_offset, command_scale = (
                            self.learned_graph_shield_command(index, threats))
                    else:
                        left, right, goal_distance, avoid_offset, command_scale = (
                            self.counterfactual_safe_command(index, threats))
                else:
                    left, right, goal_distance = self.command(vessel, 1.0)
                    avoid_offset = 0.0
                    command_scale = 1.0
                    self.previous_avoid_offset[index] *= 0.5
                yielding = bool(self.yielding[index])
            else:
                scale = 0.0 if self.yielding[index] else 1.0
                left, right, goal_distance = self.command(vessel, scale)
                avoid_offset = 0.0
                command_scale = scale
                yielding = bool(self.yielding[index])
                self.previous_avoid_offset[index] *= 0.5
            self.publish(vessel, left, right)
            if index < self.cooperative_count and goal_distance <= self.residence_radius:
                if not self.arrival_qualified[index]:
                    self.arrival_qualified[index] = True
                    self.first_arrival_time[index] = now
                if not math.isfinite(self.residence_start[index]):
                    self.residence_start[index] = now
                # Half a 10 Hz sample avoids rejecting an exact 10 s dwell due
                # to floating-point timestamp representation.
                if (now - self.residence_start[index]
                        >= self.required_residence - 0.05):
                    self.residence_qualified[index] = True
            elif not self.residence_qualified[index]:
                self.residence_start[index] = math.nan
            max_risk = max((edge["risk"] for edge in threats), default=math.nan)
            min_clearance = min(
                (edge["clearance"] for edge in threats), default=math.nan)
            self.writer.writerow([
                f"{now:.3f}", vessel.name, f"{vessel.position[0]:.3f}",
                f"{vessel.position[1]:.3f}", f"{vessel.heading:.5f}",
                f"{vessel.goal[0]:.3f}", f"{vessel.goal[1]:.3f}",
                f"{goal_distance:.3f}", f"{nearest[index]:.3f}",
                int(yielding), f"{left:.3f}", f"{right:.3f}",
                int(vessel.reached),
                self.control_mode, self.graph_mode, int(graph_fresh), len(threats),
                f"{max_risk:.6f}", f"{min_clearance:.6f}",
                f"{avoid_offset:.6f}", f"{command_scale:.3f}",
                int(self.arrival_qualified[index]),
                int(self.residence_qualified[index]),
                int(self.residual_eligible[index]),
                int(self.residual_accepted[index]),
                f"{self.residual_heading_delta[index]:.6f}",
                f"{self.residual_speed_delta[index]:.6f}",
                f"{self.environment_uncertainty:.6f}",
                f"{self.residual_authority:.6f}",
                f"{self.patrol_direction[index]:.0f}",
                int(self.harbor_route_index[index])
                if index < self.cooperative_count
                and self.encounter_mode == "harbor" else -1,
            ])
        self.file.flush()

        elapsed = now - self.experiment_start
        cooperative_arrivals = self.arrival_qualified[:self.cooperative_count]
        cooperative_residence = self.residence_qualified[:self.cooperative_count]
        if bool(np.all(cooperative_arrivals)) and not math.isfinite(self.all_arrived_time):
            self.all_arrived_time = now
            self.get_logger().info(
                f"ARRIVAL: all {self.cooperative_count} cooperative vessels entered the "
                f"{self.residence_radius:.1f} m "
                f"goal region by {elapsed:.1f} s; observing for "
                f"{self.post_arrival_observation:.1f} s.")

        observation_complete = (
            math.isfinite(self.all_arrived_time)
            and now - self.all_arrived_time >= self.post_arrival_observation - 0.05
        )
        if observation_complete:
            held = int(np.sum(cooperative_residence))
            self.get_logger().info(
                f"SUCCESS: all cooperative vessels arrived; "
                f"{held}/{self.cooperative_count} also held within "
                f"{self.residence_radius:.1f} m for {self.required_residence:.1f} s; "
                f"stopping at {elapsed:.1f} s.")
            rclpy.shutdown()
        elif elapsed >= self.max_experiment_duration:
            arrived = int(np.sum(cooperative_arrivals))
            held = int(np.sum(cooperative_residence))
            self.get_logger().warning(
                f"TIMEOUT: {arrived}/{self.cooperative_count} cooperative vessels "
                f"arrived and {held}/{self.cooperative_count} met the "
                f"residence criterion after {elapsed:.1f} s; stopping experiment.")
            rclpy.shutdown()

    def destroy_node(self):
        if rclpy.ok():
            for vessel in self.vessels:
                self.publish(vessel, 0.0, 0.0)
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SixVesselCrossingDriver()
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
