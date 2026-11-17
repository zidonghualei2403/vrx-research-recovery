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


def disturbance_tube_radius(prediction_time, environment_uncertainty):
    """Growing position uncertainty used by the finite-horizon safety shield."""
    time_value = max(0.0, float(prediction_time))
    uncertainty = float(np.clip(environment_uncertainty, 0.0, 1.0))
    return (0.08 + 0.55 * uncertainty) * math.sqrt(time_value)


def graph_speed_limit(threats, environment_uncertainty):
    """Convert selected graph-edge risk into a physical forward-speed cap."""
    if not threats:
        return 2.0
    cap = 2.0
    uncertainty = float(np.clip(environment_uncertainty, 0.0, 1.0))
    for threat in threats:
        risk = max(
            float(threat.get("risk", 0.0)),
            float(threat.get("critical_probability", 0.0)),
        )
        clearance = float(threat.get("clearance", math.inf))
        tcpa = max(0.0, float(threat.get("tcpa", math.inf)))
        hard_safety = bool(threat.get("hard_safety", False))
        if hard_safety or clearance <= 3.0 or (risk >= 0.90 and tcpa <= 4.0):
            local_cap = 0.30
        elif clearance <= 5.5 or risk >= 0.78:
            local_cap = 0.55
        elif clearance <= 8.0 or risk >= 0.62:
            local_cap = 0.90
        elif clearance <= 11.0 or risk >= 0.45:
            local_cap = 1.30
        else:
            local_cap = 1.70
        cap = min(cap, local_cap)
    return max(0.25, cap * (1.0 - 0.12 * uncertainty))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class DeterministicSACActor:
    """Dependency-free NumPy inference for the exported SB3 SAC policy."""

    def __init__(self, path):
        with open(path, "r", encoding="utf-8") as stream:
            payload = json.load(stream)
        if payload.get("format") not in (
                "vrx_deterministic_sac_actor_v1",
                "vrx_deterministic_sac_actor_critic_v2"):
            raise ValueError(f"Unsupported SAC actor format in {path}")
        self.observation_dim = int(payload["observation_dim"])
        self.action_dim = int(payload["action_dim"])
        self.heading_scale = float(payload["heading_scale_rad"])
        self.speed_scale = float(payload["speed_scale_mps"])
        self.layers = [
            (np.asarray(layer["weight"], dtype=float),
             np.asarray(layer["bias"], dtype=float))
            for layer in payload["layers"]
        ]
        self.critics = [[
            (np.asarray(layer["weight"], dtype=float),
             np.asarray(layer["bias"], dtype=float))
            for layer in network
        ] for network in payload.get("critics", [])]

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
        if self.action_dim % 2:
            raise ValueError("Residual SAC action dimension must be even")
        return action.reshape(self.action_dim // 2, 2)

    def q_value(self, observation, action):
        if not self.critics:
            return math.nan
        value_input = np.concatenate([
            np.asarray(observation, dtype=float),
            np.asarray(action, dtype=float).reshape(-1),
        ])
        estimates = []
        for network in self.critics:
            value = value_input
            for weight, bias in network[:-1]:
                value = np.maximum(0.0, weight @ value + bias)
            weight, bias = network[-1]
            estimates.append(float((weight @ value + bias)[0]))
        return min(estimates)

    def marginal_advantages(self, observation, actions):
        """Twin-critic value gain of each vessel's residual over zero residual."""
        flat_actions = np.asarray(actions, dtype=float).reshape(-1)
        zero_action = np.zeros_like(flat_actions)
        baseline = self.q_value(observation, zero_action)
        if not math.isfinite(baseline):
            return np.full(flat_actions.size // 2, math.inf)
        advantages = []
        for index in range(flat_actions.size // 2):
            candidate = zero_action.copy()
            candidate[2 * index:2 * index + 2] = \
                flat_actions[2 * index:2 * index + 2]
            advantages.append(self.q_value(observation, candidate) - baseline)
        return np.asarray(advantages, dtype=float)


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
        self.declare_parameter("goal_tolerance", 6.0)
        self.declare_parameter("goal_slowdown_radius", 14.0)
        self.declare_parameter("patrol_endpoint_buffer", 4.0)
        self.declare_parameter("patrol_mode", "shuttle")
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
        self.declare_parameter("environment_uncertainty", 0.0)
        self.declare_parameter("cooperative_count", 6)
        self.declare_parameter("drift_filter_alpha", 0.10)
        self.declare_parameter("selected_edge_margin_gain", 2.0)
        self.declare_parameter("residual_sac_model_path", "")
        self.declare_parameter("sea_state", "calm")
        self.declare_parameter("residual_sac_authority", 1.0)
        self.declare_parameter("residual_advantage_margin", 0.0)
        self.declare_parameter("maneuver_commitment_duration", 2.5)

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
        self.patrol_mode = str(self.get_parameter("patrol_mode").value)
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
        self.drift_filter_alpha = float(np.clip(
            self.get_parameter("drift_filter_alpha").value, 0.01, 1.0))
        self.selected_edge_margin_gain = float(max(
            0.0, self.get_parameter("selected_edge_margin_gain").value))
        self.sea_state = str(self.get_parameter("sea_state").value)
        self.residual_sac_authority = float(np.clip(
            self.get_parameter("residual_sac_authority").value, 0.0, 1.0))
        self.residual_advantage_margin = float(
            self.get_parameter("residual_advantage_margin").value)
        self.maneuver_commitment_duration = float(max(
            0.0, self.get_parameter("maneuver_commitment_duration").value))
        self.residual_sac_actor = None
        if self.graph_mode == "gnn_gru_residual_sac":
            actor_path = str(self.get_parameter("residual_sac_model_path").value)
            if not actor_path or not os.path.isfile(actor_path):
                raise FileNotFoundError(
                    f"Residual SAC actor model not found: {actor_path}")
            self.residual_sac_actor = DeterministicSACActor(actor_path)
            if self.encounter_mode == "central" and (
                    self.residual_sac_actor.observation_dim != 108
                    or self.residual_sac_actor.action_dim != 12):
                raise ValueError("Six-vessel SAC requires a 108-D observation and 12-D action")
        self.central_joint_scale = 0.0
        self.central_predicted_advantage = 0.0
        self.central_rejection_reason = "disabled"
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
        if self.control_mode not in ("baseline", "csd_safe"):
            raise ValueError(f"Unsupported control_mode={self.control_mode}")
        if self.encounter_mode not in ("offset", "central", "harbor"):
            raise ValueError(f"Unsupported encounter_mode={self.encounter_mode}")
        if self.patrol_mode not in ("single_pass", "shuttle"):
            raise ValueError(f"Unsupported patrol_mode={self.patrol_mode}")

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
        self.patrol_axes = []
        for vessel in self.vessels:
            axis = vessel.goal - vessel.expected_start
            axis /= max(float(np.linalg.norm(axis)), 1e-9)
            self.patrol_axes.append(axis)
        self.static_rectangles = (
            ((-11.0, 13.0), (7.0, 3.0)),
            ((12.0, -13.0), (7.0, 3.0)),
        ) if self.encounter_mode == "harbor" else ()
        # Nominal center-corridor routes used by the MATLAB harbor scenario.
        # The learned graph may locally deviate for collision avoidance, but it
        # must return to these routes instead of escaping around the map sides.
        self.harbor_reference_routes = (
            ((-24.0, -11.0), (0.0, -4.0), (24.0, 8.0)),
            ((-24.0, 10.0), (0.0, -4.0), (24.0, -9.0)),
            ((24.0, -5.0), (0.0, 2.0), (-24.0, 4.0)),
            ((24.0, 8.0), (0.0, 4.0), (-24.0, -4.0)),
        ) if self.encounter_mode == "harbor" else ()
        self.harbor_route_index = np.zeros(4, dtype=int)
        self.yielding = np.zeros(len(self.vessels), dtype=bool)
        self.previous_avoid_offset = np.zeros(len(self.vessels), dtype=float)
        self.residual_eligible = np.zeros(len(self.vessels), dtype=bool)
        self.residual_accepted = np.zeros(len(self.vessels), dtype=bool)
        self.residual_heading_delta = np.zeros(len(self.vessels), dtype=float)
        self.residual_speed_delta = np.zeros(len(self.vessels), dtype=float)
        self.filtered_drift = np.zeros((len(self.vessels), 2), dtype=float)
        self.last_graph_speed_cap = np.full(len(self.vessels), 2.0)
        self.last_predicted_clearance = np.full(len(self.vessels), math.nan)
        self.emergency_braking = np.zeros(len(self.vessels), dtype=bool)
        self.committed_heading = np.full(len(self.vessels), np.nan)
        self.committed_speed = np.full(len(self.vessels), np.nan)
        self.commitment_until = np.full(len(self.vessels), -math.inf)
        self.graph_edges = {i: [] for i in range(len(self.vessels))}
        self.graph_probabilities = np.zeros(
            (len(self.vessels), len(self.vessels)), dtype=float)
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
            "drift_x", "drift_y", "graph_speed_cap",
            "predicted_min_clearance", "emergency_braking",
            "central_joint_scale", "central_predicted_advantage",
            "central_rejection_reason",
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
            if vessel.imu_ready:
                forward_axis = np.array([
                    math.cos(vessel.heading), math.sin(vessel.heading)])
                along_speed = float(np.dot(vessel.velocity, forward_axis))
                lateral_drift = vessel.velocity - along_speed * forward_axis
                lateral_norm = float(np.linalg.norm(lateral_drift))
                if lateral_norm > 1.8:
                    lateral_drift *= 1.8 / lateral_norm
                alpha = self.drift_filter_alpha
                self.filtered_drift[index] = (
                    (1.0 - alpha) * self.filtered_drift[index]
                    + alpha * lateral_drift)
        vessel.position = position
        vessel.stamp = now
        vessel.gps_ready = True
        self.announce_pose_if_ready(vessel)

    def imu_callback(self, msg, index):
        vessel = self.vessels[index]
        vessel.heading = yaw_from_quaternion(msg.orientation)
        vessel.yaw_rate = float(msg.angular_velocity.z)
        vessel.imu_ready = True
        self.announce_pose_if_ready(vessel)

    def publish(self, vessel, left, right):
        left_msg, right_msg = Float64(), Float64()
        left_msg.data, right_msg.data = float(left), float(right)
        self.thrust_publishers[vessel.name][0].publish(left_msg)
        self.thrust_publishers[vessel.name][1].publish(right_msg)

    def graph_callback(self, msg):
        data = list(msg.data)
        unified_gnn = self.graph_mode in (
            "learned_gnn_gru_graph_shield", "gnn_gru_residual_sac")
        stride = 12 if unified_gnn else 11 if self.graph_mode in (
            "gnn_residual_csd", "gnn_uncertainty_residual_csd",
        ) else 8
        if len(data) % stride != 0:
            self.get_logger().warning(
                f"Ignored malformed CSD edge message with {len(data)} values")
            return
        edges = {i: [] for i in range(len(self.vessels))}
        probabilities = np.zeros_like(self.graph_probabilities)
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
            if stride in (11, 12):
                edge.update({
                    "advice_sin": float(data[offset + 8]),
                    "advice_cos": float(data[offset + 9]),
                    "advice_speed": float(data[offset + 10]),
                })
            if unified_gnn:
                probabilities[ego, neighbor] = edge["critical_probability"]
                if round(data[offset + 11]):
                    edges[ego].append(edge)
            else:
                edges[ego].append(edge)
        self.graph_edges = edges
        if unified_gnn:
            self.graph_probabilities = probabilities
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

    def patrol_command(self, index):
        """Drive prescribed traffic in the configured single-pass/shuttle mode."""
        vessel = self.vessels[index]
        axis = self.patrol_axes[index]
        start = vessel.expected_start
        end = vessel.goal
        direction = self.patrol_direction[index]
        target = end if direction > 0.0 else start
        target_distance = float(np.linalg.norm(target - vessel.position))
        route_length = float(np.linalg.norm(end - start))
        progress = float(np.dot(vessel.position - start, axis))

        # The trained MATLAB scenario uses a single crossing followed by a
        # stop. Shuttle mode is retained only for a separately trained model.
        if ((direction > 0.0 and (
                target_distance <= self.patrol_endpoint_buffer
                or progress >= route_length))
                or (direction < 0.0 and (
                    target_distance <= self.patrol_endpoint_buffer
                    or progress <= 0.0))):
            if self.patrol_mode == "single_pass":
                measured_along_speed = float(np.dot(vessel.velocity, axis))
                if abs(measured_along_speed) <= 0.12:
                    self.patrol_direction[index] = 0.0
                    return 0.0, 0.0, target_distance
                brake = float(np.clip(
                    -150.0 * measured_along_speed,
                    -0.55 * self.cruise, 0.55 * self.cruise))
                return brake, brake, target_distance
            direction *= -1.0
            self.patrol_direction[index] = direction
            target = end if direction > 0.0 else start
            target_distance = float(np.linalg.norm(target - vessel.position))
            self.get_logger().info(
                f"{vessel.name} patrol thrust reversed; direction={direction:+.0f}")

        desired_heading = math.atan2(axis[1], axis[0])
        heading_error = wrap_angle(desired_heading - vessel.heading)
        measured_along_speed = float(np.dot(vessel.velocity, axis))
        desired_along_speed = 1.15 * direction
        feedforward = 0.55 * self.cruise * direction
        forward = float(np.clip(
            feedforward + 75.0 * (desired_along_speed - measured_along_speed),
            -0.72 * self.cruise, 0.72 * self.cruise))
        turn = float(np.clip(
            190.0 * heading_error, -0.75 * self.max_turn, 0.75 * self.max_turn))
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
            entered = float(np.linalg.norm(target_delta)) <= 6.0
            # A safety manoeuvre may legitimately miss the waypoint disk. The
            # perpendicular gate prevents the controller from turning back to
            # an already passed point and entering a limit cycle.
            if not entered and not (passed_gate and cross_track <= 18.0):
                return target
            route_index += 1
            self.harbor_route_index[index] = route_index
            self.get_logger().info(
                f"{vessel.name} cleared harbor waypoint "
                f"{route_index}/{len(route)}")
        return vessel.goal

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
        if self.patrol_mode == "single_pass":
            if direction == 0.0:
                return vessel.position.copy()
            scalar = min(high, scalar + 1.15 * prediction_time)
            return start + scalar * axis + lateral
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
        # Lateral wave drift must not be mistaken for useful forward speed.
        vessel_forward = np.array([
            math.cos(vessel.heading), math.sin(vessel.heading)])
        speed = max(0.0, float(np.dot(vessel.velocity, vessel_forward)))
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
        """Reproduce the 80-D observation used by residual SAC training."""
        cooperative = self.vessels[:self.cooperative_count]
        dynamic = self.vessels[self.cooperative_count:self.cooperative_count + 2]
        entity_positions = [v.position for v in cooperative]
        entity_positions.extend(
            np.asarray(center, dtype=float) for center, _ in self.static_rectangles)
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

        sea_encoding = {
            "calm": 0.0,
            "medium_wind_wave": 0.33,
            "moderate": 0.33,
            "strong_wind_wave": 0.67,
            "strong": 0.67,
            "extreme_wind_wave": 1.0,
            "extreme": 1.0,
        }.get(self.sea_state, float(np.clip(self.environment_uncertainty, 0.0, 1.0)))

        rows = []
        for index, vessel in enumerate(cooperative[:4]):
            base_heading, base_speed = plans[index][5], plans[index][6]
            c, s = math.cos(vessel.heading), math.sin(vessel.heading)
            world_to_body = np.array([[c, s], [-s, c]])
            goal_body = world_to_body @ (vessel.goal - vessel.position)
            clearances = [
                float(np.linalg.norm(position - vessel.position)) - 5.4
                for entity_index, position in enumerate(entity_positions)
                if entity_index != index
            ]
            speed = float(np.linalg.norm(vessel.velocity))
            rows.extend([
                goal_body[0] / 100.0,
                goal_body[1] / 100.0,
                float(np.linalg.norm(goal_body)) / 120.0,
                speed / 2.4,
                math.sin(wrap_angle(base_heading - vessel.heading)),
                math.cos(wrap_angle(base_heading - vessel.heading)),
                base_speed / 2.4,
                float(np.clip(min(clearances), -10.0, 50.0)) / 50.0,
                float(np.max(edge_probability[index])),
                float(np.sum(edge_probability[index])) / 4.0,
                float(vessel.reached),
                sea_encoding,
            ])
        rows.extend(edge_probability.reshape(-1).tolist())
        return np.asarray(rows, dtype=float)

    def central_gnn_gru_base_plan(self, index, threats):
        """Reproduce the frozen six-vessel GNN-GRU expert command in VRX."""
        vessel = self.vessels[index]
        offset = vessel.goal - vessel.position
        distance = float(np.linalg.norm(offset))
        if distance < 1e-6:
            return vessel.heading, 0.0
        reference = offset / distance
        graph_vector = reference.copy()
        graph_speed = 1.25
        if threats and "advice_sin" in threats[0]:
            # The graph actor emits (forward, port) in the ego body frame.
            # Convert that local direction to the world frame before blending
            # it with the world-frame route direction.
            local_forward = float(threats[0]["advice_sin"])
            local_port = float(threats[0]["advice_cos"])
            graph_heading = wrap_angle(
                vessel.heading + math.atan2(local_port, local_forward))
            graph_vector = np.array([
                math.cos(graph_heading), math.sin(graph_heading)])
            graph_speed = 2.0 * float(threats[0]["advice_speed"])
        desired = 0.72 * reference + 0.28 * graph_vector
        repulsion = np.zeros(2, dtype=float)
        nearest = math.inf
        for neighbor_index, neighbor in enumerate(self.vessels):
            if neighbor_index == index:
                continue
            delta = vessel.position - neighbor.position
            center_distance = max(float(np.linalg.norm(delta)), 1e-5)
            clearance = self.oriented_rectangle_clearance(
                vessel.position, vessel.heading, np.array([2.7, 1.35]),
                neighbor.position, neighbor.heading, np.array([2.7, 1.35]))
            nearest = min(nearest, clearance)
            if clearance < 14.0:
                edge = next((item for item in threats
                             if item["neighbor"] == neighbor_index), None)
                graph_weight = 0.0 if edge is None else max(
                    float(edge.get("critical_probability", 0.0)),
                    float(edge.get("risk", 0.0)))
                weight = max(graph_weight, (14.0 - clearance) / 14.0)
                repulsion += (
                    weight * delta / center_distance / max(clearance + 1.5, 0.35))
        starboard = np.array([reference[1], -reference[0]])
        safety_margin = 3.0 + 4.0 * {
            "calm": 0.015, "moderate": 0.06, "medium_wind_wave": 0.06,
            "strong": 0.12, "strong_wind_wave": 0.12,
            "extreme": 0.20, "extreme_wind_wave": 0.20,
        }.get(self.sea_state, 0.015)
        urgency = float(np.clip(
            (safety_margin + 5.0 - nearest) / 5.0, 0.0, 1.0))
        vector = desired + 8.0 * repulsion + 0.9 * urgency * starboard
        desired_heading = math.atan2(vector[1], vector[0])
        desired_speed = min(1.6, max(0.0, graph_speed + 0.35))
        desired_speed *= float(np.clip(
            (nearest - safety_margin + 2.0) / 7.0, 0.08, 1.0))
        if vessel.reached:
            desired_speed = 0.0
        return desired_heading, desired_speed

    def central_residual_observation(self, plans):
        """Build the 108-D state shared by the six-vessel SAC policy."""
        edge_probability = self.graph_probabilities.copy()
        sea_noise = {
            "calm": 0.015, "moderate": 0.06, "medium_wind_wave": 0.06,
            "strong": 0.12, "strong_wind_wave": 0.12,
            "extreme": 0.20, "extreme_wind_wave": 0.20,
        }.get(self.sea_state, 0.015)
        rows = []
        for index, vessel in enumerate(self.vessels):
            base_heading, base_speed = plans[index]
            c, s = math.cos(vessel.heading), math.sin(vessel.heading)
            rotation = np.array([[c, s], [-s, c]])
            goal = rotation @ (vessel.goal - vessel.position)
            nearest = min(
                float(np.linalg.norm(other.position - vessel.position)) - 5.4
                for j, other in enumerate(self.vessels) if j != index)
            rows.extend([
                goal[0] / 100.0, goal[1] / 100.0,
                float(np.linalg.norm(goal)) / 120.0,
                float(np.linalg.norm(vessel.velocity)) / 2.4,
                math.sin(wrap_angle(base_heading - vessel.heading)),
                math.cos(wrap_angle(base_heading - vessel.heading)),
                base_speed / 2.4, float(np.clip(nearest, -10.0, 50.0)) / 50.0,
                float(edge_probability[index].max()),
                float(edge_probability[index].sum() / 5.0),
                float(self.arrival_qualified[index]), sea_noise / 0.20,
            ])
        rows.extend(edge_probability.reshape(-1).tolist())
        return np.asarray(rows, dtype=float)

    def central_safety_threshold(self):
        noise = {"calm": 0.015, "medium_wind_wave": 0.06,
                 "strong_wind_wave": 0.12}.get(self.sea_state, 0.015)
        return 0.8 + 4.0 * noise

    def central_joint_prediction(self, commands, horizon=8.0, dt=0.5):
        """Advance all six commands together using the calibrated V6 predictor."""
        commands = np.asarray(commands, dtype=float).reshape(6, 2)
        position = np.stack([v.position for v in self.vessels]).astype(float)
        heading = np.asarray([v.heading for v in self.vessels], dtype=float)
        speed = np.asarray([np.linalg.norm(v.velocity) for v in self.vessels], dtype=float)
        yaw_rate = np.asarray([v.yaw_rate for v in self.vessels], dtype=float)
        drift = {"calm": (0.00068, -0.00041),
                 "medium_wind_wave": (-0.07546, -0.21089),
                 "strong_wind_wave": (-0.09430, -0.27213)}.get(
                     self.sea_state, (0.0, 0.0))
        pair_i, pair_j = np.triu_indices(6, 1)
        minimum = math.inf
        worst = (-1, -1)
        for _ in range(int(round(horizon / dt))):
            error = (commands[:, 0] - heading + np.pi) % (2 * np.pi) - np.pi
            speed = np.clip(speed + dt * (0.08 * (commands[:, 1] - speed)
                                          + 0.021207032359110145), 0.0, 2.6)
            yaw_rate = np.clip(yaw_rate + dt * (
                0.221694996618843 * error - 0.4823384366818415 * yaw_rate
                - 0.04262763808013115), -math.radians(35), math.radians(35))
            heading = (heading + dt * yaw_rate + np.pi) % (2 * np.pi) - np.pi
            position += dt * (speed[:, None] * np.stack(
                (np.cos(heading), np.sin(heading)), axis=1) + drift)
            gaps = np.linalg.norm(position[pair_i] - position[pair_j], axis=1) - 6.10
            local = int(np.argmin(gaps))
            if gaps[local] < minimum:
                minimum = float(gaps[local])
                worst = int(pair_i[local]), int(pair_j[local])
        return minimum, worst

    def central_base_shield(self, plans):
        """Apply the same deterministic right-of-way rule to both GNN arms."""
        safe = np.asarray([plans[i] for i in range(6)], dtype=float).copy()
        threshold = self.central_safety_threshold()
        visits = {}
        for _ in range(18):
            minimum, pair = self.central_joint_prediction(safe)
            if minimum >= threshold or pair[0] < 0:
                break
            i, j = pair
            visits[pair] = visits.get(pair, 0) + 1
            if visits[pair] >= 2:
                yielding = i if safe[i, 1] >= safe[j, 1] else j
            elif self.arrival_qualified[i] and not self.arrival_qualified[j]:
                yielding = j
            elif self.arrival_qualified[j] and not self.arrival_qualified[i]:
                yielding = i
            else:
                yielding = max(i, j)
            old_speed = safe[yielding, 1]
            safe[yielding, 1] = 0.0 if old_speed < 0.18 else 0.65 * old_speed
            if old_speed - safe[yielding, 1] < 1e-4:
                break
        return {i: tuple(safe[i]) for i in range(6)}

    def central_command_score(self, commands, predicted_clearance=None):
        commands = np.asarray(commands, dtype=float).reshape(6, 2)
        positions = np.stack([v.position for v in self.vessels])
        goals = np.stack([v.goal for v in self.vessels])
        headings = np.asarray([v.heading for v in self.vessels])
        speeds = np.asarray([np.linalg.norm(v.velocity) for v in self.vessels])
        goal_offset = goals - positions
        goal_headings = np.arctan2(goal_offset[:, 1], goal_offset[:, 0])
        forward = commands[:, 1] * np.cos(
            (commands[:, 0] - goal_headings + np.pi) % (2 * np.pi) - np.pi)
        active = ~self.arrival_qualified[:6]
        progress = forward[active]
        if not progress.size:
            return 0.0
        if predicted_clearance is None:
            predicted_clearance, _ = self.central_joint_prediction(commands)
        clearance_credit = min(predicted_clearance, self.central_safety_threshold() + 4.0)
        turn = np.mean(np.abs((commands[:, 0] - headings + np.pi) % (2 * np.pi) - np.pi))
        acceleration = np.mean(np.abs(commands[:, 1] - speeds))
        return float(np.mean(progress) + 0.50 * np.min(progress)
                     + 0.12 * clearance_credit - 0.08 * turn
                     - 0.04 * acceleration)

    def project_central_residual(self, plans, actions, force_reject=False):
        """Use a common residual scale, joint safety, and a V6 advantage gate."""
        base = np.asarray([plans[i] for i in range(6)], dtype=float)
        actions = np.asarray(actions, dtype=float).reshape(6, 2)
        residual = np.column_stack((
            self.residual_sac_actor.heading_scale * actions[:, 0],
            self.residual_sac_actor.speed_scale * actions[:, 1]))
        residual *= self.residual_sac_authority
        self.central_joint_scale = 0.0
        self.central_predicted_advantage = 0.0
        self.central_rejection_reason = "zero_or_forced"
        if force_reject or np.linalg.norm(residual) <= 1e-7:
            return dict(plans), np.zeros((6, 2), dtype=float)
        threshold = self.central_safety_threshold()
        base_score = self.central_command_score(base)
        for scale in (1.0, 0.75, 0.5, 0.25):
            candidate = np.column_stack((
                (base[:, 0] + scale * residual[:, 0] + np.pi) % (2 * np.pi) - np.pi,
                np.clip(base[:, 1] + scale * residual[:, 1], 0.0, 2.0)))
            candidate[self.arrival_qualified[:6], 1] = 0.0
            minimum, _ = self.central_joint_prediction(candidate)
            if minimum < threshold:
                self.central_rejection_reason = "joint_clearance"
                continue
            advantage = self.central_command_score(candidate, minimum) - base_score
            if advantage < 0.01:
                self.central_rejection_reason = "advantage"
                continue
            self.central_joint_scale = scale
            self.central_predicted_advantage = advantage
            self.central_rejection_reason = "accepted"
            return {i: tuple(candidate[i]) for i in range(6)}, scale * residual
        return dict(plans), np.zeros((6, 2), dtype=float)

    def residual_safety_scale(self, index, predicted_clearance, threats):
        """Fade learned residuals near hull, wall, or graph safety limits."""
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
            clearance = min(clearance, self.oriented_rectangle_clearance(
                vessel.position, vessel.heading, hull_half_size,
                np.asarray(center, dtype=float), 0.0,
                np.asarray(half_size, dtype=float)))
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
        for threat in threats:
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

        selected_neighbors = {int(threat["neighbor"]) for threat in threats}
        selected_risk = {
            int(threat["neighbor"]): max(
                float(threat.get("risk", 0.0)),
                float(threat.get("critical_probability", 0.0)))
            for threat in threats
        }
        speed_cap = min(
            2.0 * graph_speed,
            graph_speed_limit(threats, self.environment_uncertainty),
        )

        # Static walls act as virtual constraint nodes for the controller. This
        # keeps geometry protection independent of graph-message freshness.
        hull_half_size = np.array([2.7, 1.35])
        current_wall_clearance = math.inf
        for center, half_size in self.static_rectangles:
            wall_clearance = self.oriented_rectangle_clearance(
                vessel.position, vessel.heading, hull_half_size,
                np.asarray(center, dtype=float), 0.0,
                np.asarray(half_size, dtype=float))
            current_wall_clearance = min(current_wall_clearance, wall_clearance)
        if current_wall_clearance <= 3.0:
            speed_cap = min(speed_cap, 0.35)
        elif current_wall_clearance <= 6.0:
            speed_cap = min(speed_cap, 0.75)
        elif current_wall_clearance <= 9.0:
            speed_cap = min(speed_cap, 1.20)
        self.last_graph_speed_cap[index] = speed_cap

        # Match the MATLAB safety actor: 21 heading choices, six speed
        # choices, finite turn rate / acceleration and a 14 s horizon.
        candidate_headings = [
            wrap_angle(reference_heading + math.radians(value))
            for value in range(-100, 101, 10)
        ]
        nominal_speeds = (0.0, 0.45, 0.85, 1.25, 1.65, 2.0)
        candidate_speeds = tuple(sorted(set(
            [speed for speed in nominal_speeds if speed <= speed_cap + 1e-9]
            + [float(speed_cap)])))
        times = np.arange(0.0, 14.0 + 1e-9, 0.75)
        current_speed = min(2.4, float(np.linalg.norm(vessel.velocity)))
        max_turn_rate = math.radians(22.0)
        max_accel = 0.65
        candidates = []
        drift_gain = 0.15 + 0.85 * float(np.clip(
            self.environment_uncertainty, 0.0, 1.0))

        for heading in candidate_headings:
            offset = wrap_angle(heading - reference_heading)
            for desired_speed in candidate_speeds:
                minimum_clearance = math.inf
                integrated_violation = 0.0
                collision_penalty = 0.0
                final_position = vessel.position.copy()

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
                    predicted_position += (
                        prediction_time * drift_gain * self.filtered_drift[index])
                    final_position = predicted_position

                    clearance_margins = []
                    uncertainty_tube = disturbance_tube_radius(
                        prediction_time, self.environment_uncertainty)
                    for other_index, other in enumerate(self.vessels):
                        if other_index == index:
                            continue
                        if other_index >= self.cooperative_count:
                            other_position = self.predicted_patrol_position(
                                other_index, prediction_time)
                            safety_margin = 4.5
                        else:
                            other_position = (
                                other.position + prediction_time * other.velocity)
                            safety_margin = 3.0
                        edge_weight = 1.0
                        if other_index in selected_neighbors:
                            edge_risk = selected_risk.get(other_index, 0.0)
                            safety_margin += self.selected_edge_margin_gain * (
                                0.55 + 0.45 * edge_risk)
                            edge_weight += 2.0 * edge_risk
                        safety_margin += uncertainty_tube
                        clearance = self.oriented_rectangle_clearance(
                            predicted_position, predicted_heading, hull_half_size,
                            other_position, other.heading, hull_half_size)
                        clearance_margins.append(
                            (clearance, safety_margin, edge_weight))
                    for center, half_size in self.static_rectangles:
                        clearance = self.oriented_rectangle_clearance(
                            predicted_position, predicted_heading, hull_half_size,
                            np.asarray(center, dtype=float), 0.0,
                            np.asarray(half_size, dtype=float))
                        clearance_margins.append((
                            clearance, 3.0 + uncertainty_tube,
                            1.0 + self.environment_uncertainty))

                    step_clearance = min(
                        clearance for clearance, _, _ in clearance_margins)
                    minimum_clearance = min(minimum_clearance, step_clearance)
                    for clearance, safety_margin, edge_weight in clearance_margins:
                        violation = max(0.0, safety_margin - clearance)
                        integrated_violation += (
                            edge_weight * math.exp(-0.16 * prediction_time)
                            * violation ** 2)
                        if clearance < 0.0:
                            collision_penalty += (
                                edge_weight * 2500.0
                                * math.exp(-0.20 * prediction_time))

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
                score = (
                    collision_penalty + 22.0 * integrated_violation
                    + 0.20 * final_goal_distance + 0.85 * target_distance
                    + reverse_progress_penalty
                    + 1.2 * turn ** 2 + 0.35 * port_turn ** 2
                    + low_speed_penalty + 2.0 * graph_disagreement ** 2
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
        now = self.now_seconds()
        if now < self.commitment_until[index]:
            committed = min(candidates, key=lambda candidate: (
                abs(wrap_angle(
                    candidate["heading"] - self.committed_heading[index])),
                abs(candidate["desired_speed"] - self.committed_speed[index]),
            ))
            # Hold a recent maneuver while it remains safely feasible. This
            # avoids 10 Hz left/right and go/stop decision chatter.
            if committed["minimum_clearance"] >= 2.0:
                choice = committed
        emergency = choice["minimum_clearance"] < 1.5
        if emergency:
            # A soft cost alone may still choose a least-bad collision course.
            # Once every nominal candidate violates the safety envelope, switch
            # to a lexicographic fallback: maximize clearance first, then cost.
            best_clearance = max(
                candidate["minimum_clearance"] for candidate in candidates)
            clearance_candidates = [
                candidate for candidate in candidates
                if candidate["minimum_clearance"] >= best_clearance - 0.10
            ]
            choice = min(
                clearance_candidates, key=lambda candidate: candidate["score"])
            choice = dict(choice)
            self.commitment_until[index] = -math.inf
        elif now >= self.commitment_until[index]:
            self.committed_heading[index] = choice["heading"]
            self.committed_speed[index] = choice["desired_speed"]
            self.commitment_until[index] = (
                now + self.maneuver_commitment_duration)
        self.last_predicted_clearance[index] = choice["minimum_clearance"]
        self.emergency_braking[index] = emergency

        self.previous_avoid_offset[index] = choice["offset"]
        left, right, distance = self.command_heading_speed(
            vessel, choice["heading"], choice["desired_speed"])
        if emergency and choice["desired_speed"] <= 0.05:
            vessel_forward = np.array([
                math.cos(vessel.heading), math.sin(vessel.heading)])
            forward_speed = max(
                0.0, float(np.dot(vessel.velocity, vessel_forward)))
            brake = float(np.clip(
                125.0 * forward_speed, 0.0, 0.55 * self.cruise))
            left -= brake
            right -= brake
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
        if not all(v.initialized and now - v.stamp < 1.0 for v in self.vessels):
            for vessel in self.vessels:
                self.publish(vessel, 0.0, 0.0)
            if (not self.startup_failed
                    and time.monotonic() - self.startup_wall_time >= self.startup_timeout):
                self.startup_failed = True
                details = ", ".join(
                    f"{v.name}:{v.valid_start_samples}/{self.required_start_samples}"
                    for v in self.vessels)
                self.get_logger().error(
                    "STARTUP_VALIDATION_FAILED: did not receive clean poses at "
                    f"the configured starts within {self.startup_timeout:.1f} s; "
                    f"samples={details}")
                rclpy.shutdown()
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
        for index in range(len(self.vessels)):
            self.residual_eligible[index] = False
            self.residual_accepted[index] = False
            self.residual_heading_delta[index] = 0.0
            self.residual_speed_delta[index] = 0.0
            self.last_graph_speed_cap[index] = 2.0
            self.last_predicted_clearance[index] = math.nan
            self.emergency_braking[index] = False
        self.central_joint_scale = 0.0
        self.central_predicted_advantage = 0.0
        self.central_rejection_reason = "not_applicable"

        harbor_plans = {}
        central_plans = {}
        central_residual_plans = {}
        central_accepted = None
        residual_actions = None
        if (self.encounter_mode == "harbor"
                and self.control_mode == "csd_safe"
                and self.graph_mode == "gnn_gru_residual_sac"):
            for index in range(self.cooperative_count):
                threats = self.graph_edges[index] if graph_fresh else []
                harbor_plans[index] = self.harbor_graph_shield_command(
                    index, threats)
            observation = self.harbor_residual_observation(harbor_plans)
            residual_actions = self.residual_sac_actor.predict(observation)
        elif (self.encounter_mode == "central"
                and self.control_mode == "csd_safe"
                and self.graph_mode in ("distance_star", "risk_star",
                                        "conditional_csd",
                                        "learned_gnn_gru_graph_shield",
                                        "gnn_gru_residual_sac")
                and graph_fresh):
            for index in range(6):
                central_plans[index] = self.central_gnn_gru_base_plan(
                    index, self.graph_edges[index])
            central_plans = self.central_base_shield(central_plans)
            central_residual_plans = dict(central_plans)
            central_accepted = np.zeros((6, 2), dtype=float)
            if self.graph_mode == "gnn_gru_residual_sac":
                observation = self.central_residual_observation(central_plans)
                residual_actions = self.residual_sac_actor.predict(observation)
                central_residual_plans, central_accepted = (
                    self.project_central_residual(central_plans,
                                                  residual_actions))

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
            elif self.control_mode == "csd_safe" and (
                    graph_fresh or self.encounter_mode == "harbor"):
                # In the harbor benchmark every graph construction method uses
                # the same route tracker and finite-horizon hull / wall safety
                # actor. Only the selected graph edges differ between methods.
                if (self.encounter_mode == "central"
                        and central_residual_plans):
                    desired_heading, desired_speed = central_residual_plans[index]
                    left, right, goal_distance = self.command_heading_speed(
                        vessel, desired_heading, desired_speed)
                    goal_heading = math.atan2(
                        vessel.goal[1] - vessel.position[1],
                        vessel.goal[0] - vessel.position[0])
                    avoid_offset = wrap_angle(desired_heading - goal_heading)
                    command_scale = desired_speed / 2.0
                    if self.graph_mode == "gnn_gru_residual_sac":
                        heading_delta, speed_delta = central_accepted[index]
                        self.residual_eligible[index] = True
                        self.residual_accepted[index] = bool(
                            abs(heading_delta) > 1e-5 or abs(speed_delta) > 1e-5)
                        self.residual_heading_delta[index] = heading_delta
                        self.residual_speed_delta[index] = speed_delta
                elif self.encounter_mode == "harbor":
                    plan = harbor_plans.get(index)
                    if plan is None:
                        plan = self.harbor_graph_shield_command(index, threats)
                    (left, right, goal_distance, avoid_offset, command_scale,
                     base_heading, base_speed) = plan
                    if (residual_actions is not None and not vessel.reached
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
                        desired_heading = wrap_angle(base_heading + heading_delta)
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
                yielding = False
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
                f"{self.filtered_drift[index, 0]:.6f}",
                f"{self.filtered_drift[index, 1]:.6f}",
                f"{self.last_graph_speed_cap[index]:.6f}",
                f"{self.last_predicted_clearance[index]:.6f}",
                int(self.emergency_braking[index]),
                f"{self.central_joint_scale:.3f}",
                f"{self.central_predicted_advantage:.6f}",
                self.central_rejection_reason,
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
