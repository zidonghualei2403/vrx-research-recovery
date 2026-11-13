#!/usr/bin/python3
import csv
import json
import math
import os
from dataclasses import dataclass, field

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64MultiArray


def sigmoid(value):
    return 1.0 / (1.0 + np.exp(-np.clip(value, -60.0, 60.0)))


def linear(value, weight, bias):
    return value @ weight.T + bias.reshape(-1)


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


@dataclass
class VesselState:
    name: str
    position: np.ndarray = field(default_factory=lambda: np.zeros(2))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(2))
    heading: float = 0.0
    stamp: float = -math.inf
    initialized: bool = False
    gps_ready: bool = False
    imu_ready: bool = False


class StrictConditionalModel:
    def __init__(self, path):
        raw = json.loads(open(path, encoding="utf-8").read())
        self.method = raw["method"]
        for key, value in raw.items():
            if key != "method":
                setattr(self, key, np.asarray(value, dtype=float))
        self.hidden_dim = int(np.asarray(self.hidden_dim).reshape(-1)[0])
        self.pair_dim = int(np.asarray(getattr(self, "pair_dim", 9)).reshape(-1)[0])
        self.edge_budget = int(np.asarray(
            getattr(self, "edge_budget", 5)).reshape(-1)[0])

    def pair_encode(self, pair):
        value = np.maximum(0.0, linear(
            pair, self.pair_encoder_0_weight, self.pair_encoder_0_bias))
        return np.maximum(0.0, linear(
            value, self.pair_encoder_2_weight, self.pair_encoder_2_bias))

    def gru(self, encoded, previous):
        input_gates = linear(
            encoded, self.pair_memory_weight_ih_l0, self.pair_memory_bias_ih_l0)
        hidden_gates = linear(
            previous, self.pair_memory_weight_hh_l0, self.pair_memory_bias_hh_l0)
        h = self.hidden_dim
        reset = sigmoid(input_gates[:h] + hidden_gates[:h])
        update = sigmoid(input_gates[h:2*h] + hidden_gates[h:2*h])
        candidate = np.tanh(input_gates[2*h:] + reset * hidden_gates[2*h:])
        return (1.0 - update) * candidate + update * previous

    def predict(self, features):
        value = np.maximum(0.0, linear(
            features, self.conditional_head_net_0_weight,
            self.conditional_head_net_0_bias))
        value = np.maximum(0.0, linear(
            value, self.conditional_head_net_2_weight,
            self.conditional_head_net_2_bias))
        safety_z = float(linear(
            value, self.conditional_head_safety_weight,
            self.conditional_head_safety_bias)[0])
        efficiency_z = float(linear(
            value, self.conditional_head_efficiency_weight,
            self.conditional_head_efficiency_bias)[0])
        critical_logit = float(linear(
            value, self.conditional_head_critical_weight,
            self.conditional_head_critical_bias)[0])
        safety_t = safety_z * float(self.safety_transform_std) + float(self.safety_transform_mean)
        efficiency_t = efficiency_z * float(self.efficiency_transform_std) + float(self.efficiency_transform_mean)
        safety = math.copysign(math.expm1(abs(safety_t)), safety_t)
        efficiency = math.copysign(math.expm1(abs(efficiency_t)), efficiency_t)
        probability = float(sigmoid(critical_logit))
        return safety, efficiency, probability

    def graph_message(self, hidden):
        if len(hidden) == 0:
            return np.zeros(self.hidden_dim)
        messages = np.maximum(0.0, linear(
            hidden, self.message_head_0_weight, self.message_head_0_bias))
        messages = linear(
            messages, self.message_head_2_weight, self.message_head_2_bias)
        logits = linear(
            hidden, self.attention_head_weight, self.attention_head_bias).reshape(-1)
        logits -= np.max(logits)
        weights = np.exp(logits)
        weights /= max(float(np.sum(weights)), 1e-8)
        return np.sum(weights[:, None] * messages, axis=0)

    def action_advice(self, own_state, hidden):
        aggregate = self.graph_message(hidden)
        own = np.maximum(0.0, linear(
            own_state, self.state_encoder_0_weight, self.state_encoder_0_bias))
        own = np.maximum(0.0, linear(
            own, self.state_encoder_2_weight, self.state_encoder_2_bias))
        value = np.concatenate([own, aggregate])
        value = np.maximum(0.0, linear(
            value, self.actor_0_weight, self.actor_0_bias))
        value = np.maximum(0.0, linear(
            value, self.actor_2_weight, self.actor_2_bias))
        raw = linear(value, self.actor_4_weight, self.actor_4_bias)
        norm = max(float(np.linalg.norm(raw[:2])), 1e-6)
        direction = raw[:2] / norm
        speed = float(sigmoid(raw[2]))
        return float(direction[0]), float(direction[1]), speed

    def edge_probabilities(self, hidden):
        value = np.maximum(0.0, linear(
            hidden, self.edge_head_0_weight, self.edge_head_0_bias))
        logits = linear(value, self.edge_head_2_weight, self.edge_head_2_bias)
        logits -= np.max(logits, axis=-1, keepdims=True)
        probabilities = np.exp(logits)
        return probabilities / np.maximum(
            np.sum(probabilities, axis=-1, keepdims=True), 1e-12)


class CSDShadowGraph(Node):
    GRAPH_MODES = (
        "conditional_csd",
        "distance_star",
        "risk_star",
        "full_graph",
        "conditional_no_gru",
        "conditional_no_conformal",
        "gnn_conditional_csd",
        "gnn_coordinated_csd",
        "gnn_residual_csd",
        "gnn_uncertainty_residual_csd",
        "learned_gnn_gru_graph_shield",
        "gnn_gru_residual_sac",
    )

    def __init__(self):
        super().__init__("csd_shadow_graph")
        self.declare_parameter("vessel_names", [
            "wamv", "ci_nav_dynamic_circle_obstacle",
            "ci_nav_dynamic_circle_obstacle_2", "ci_nav_dynamic_circle_obstacle_3",
        ])
        self.declare_parameter("model_path", "")
        self.declare_parameter("gnn_model_path", "")
        self.declare_parameter("learned_gnn_gru_model_path", "")
        self.declare_parameter("log_dir", "/tmp/vrx_csd_shadow")
        self.declare_parameter("update_period", 0.25)
        self.declare_parameter("stale_timeout", 1.0)
        self.declare_parameter("world_origin_x", 0.0)
        self.declare_parameter("world_origin_y", 0.0)
        self.declare_parameter("conformal_quantile", 0.1658)
        self.declare_parameter("graph_mode", "conditional_csd")
        self.declare_parameter("heuristic_edge_budget", 2)
        self.declare_parameter("origin_latitude", -33.724223)
        self.declare_parameter("origin_longitude", 150.679736)
        self.declare_parameter("encounter_mode", "central")
        self.declare_parameter("cooperative_count", 6)
        self.declare_parameter("environment_uncertainty", 0.0)

        self.names = list(self.get_parameter("vessel_names").value)
        model_path = self.get_parameter("model_path").value
        gnn_model_path = self.get_parameter("gnn_model_path").value
        learned_gnn_gru_model_path = self.get_parameter(
            "learned_gnn_gru_model_path").value
        self.log_dir = self.get_parameter("log_dir").value
        self.period = float(self.get_parameter("update_period").value)
        self.stale_timeout = float(self.get_parameter("stale_timeout").value)
        self.origin = np.array([
            float(self.get_parameter("world_origin_x").value),
            float(self.get_parameter("world_origin_y").value),
        ])
        self.conformal_quantile = float(self.get_parameter("conformal_quantile").value)
        self.graph_mode = str(self.get_parameter("graph_mode").value)
        self.environment_uncertainty = float(
            self.get_parameter("environment_uncertainty").value)
        self.residual_authority = float(np.clip(
            (0.90 - self.environment_uncertainty) / 0.15, 0.0, 1.0))
        self.heuristic_edge_budget = int(
            self.get_parameter("heuristic_edge_budget").value)
        if self.graph_mode not in self.GRAPH_MODES:
            raise ValueError(
                f"Unsupported graph_mode={self.graph_mode}; "
                f"expected one of {self.GRAPH_MODES}")
        self.origin_latitude = float(self.get_parameter("origin_latitude").value)
        self.origin_longitude = float(self.get_parameter("origin_longitude").value)
        self.encounter_mode = str(self.get_parameter("encounter_mode").value)
        self.cooperative_count = int(
            self.get_parameter("cooperative_count").value)
        if self.graph_mode in (
                "learned_gnn_gru_graph_shield", "gnn_gru_residual_sac"):
            selected_model_path = learned_gnn_gru_model_path
        else:
            selected_model_path = gnn_model_path if self.uses_gnn else model_path
        if not selected_model_path or not os.path.isfile(selected_model_path):
            raise FileNotFoundError(
                f"Model for graph_mode={self.graph_mode} not found: "
                f"{selected_model_path}")
        self.model = StrictConditionalModel(selected_model_path)
        if self.encounter_mode == "central":
            goal_points = [
                (-45.0, 0.0), (-22.5, -38.97114317),
                (22.5, -38.97114317), (45.0, 0.0),
                (22.5, 38.97114317), (-22.5, 38.97114317),
            ]
        elif self.encounter_mode == "harbor":
            goal_points = [
                (48.0, 22.0), (48.0, -22.0),
                (-48.0, 8.0), (-48.0, -8.0),
                (-25.0, 34.0), (25.0, -34.0),
            ]
        else:
            goal_points = [
                (-38.97114317, 22.5), (-38.97114317, -22.5),
                (0.0, -45.0), (38.97114317, -22.5),
                (38.97114317, 22.5), (0.0, 45.0),
            ]
        self.goals = np.asarray(goal_points, dtype=float)
        self.states = [VesselState(name) for name in self.names]
        n = len(self.names)
        self.hidden = np.zeros((n, n, self.model.hidden_dim))
        self.edge_state = np.zeros((n, n), dtype=int)
        self.dormant_steps = np.zeros((n, n), dtype=int)

        os.makedirs(self.log_dir, exist_ok=True)
        self.edge_file = open(os.path.join(self.log_dir, "shadow_edges.csv"), "w", newline="")
        self.edge_writer = csv.writer(self.edge_file)
        self.edge_writer.writerow([
            "time", "ego", "neighbor", "current_distance", "tcpa",
            "predicted_clearance", "calibrated_risk", "hard_safety",
            "critical_probability", "safety_value", "safety_lower_bound",
            "efficiency_value", "eligible", "selected", "selection_slot",
            "previous_edge_state", "next_edge_state", "graph_mode",
        ])
        self.state_file = open(os.path.join(self.log_dir, "shadow_states.csv"), "w", newline="")
        self.state_writer = csv.writer(self.state_file)
        self.state_writer.writerow(["time", "name", "x", "y", "vx", "vy", "heading", "age"])
        self.active_edge_publisher = self.create_publisher(
            Float64MultiArray, "/csd/active_edges", 10)

        for index, name in enumerate(self.names):
            self.create_subscription(
                NavSatFix, f"/{name}/sensors/gps/gps/fix",
                lambda msg, i=index: self.gps_callback(msg, i), 10)
            self.create_subscription(
                Imu, f"/{name}/sensors/imu/imu/data",
                lambda msg, i=index: self.imu_callback(msg, i), 10)
        self.timer = self.create_timer(self.period, self.step)
        self.get_logger().info(
            f"Read-only graph: mode={self.graph_mode}, vessels={self.names}, "
            f"log_dir={self.log_dir}")
        self.get_logger().info("No propulsion publishers are created by this node.")

    @property
    def uses_gnn(self):
        requested = self.graph_mode in (
            "gnn_conditional_csd", "gnn_coordinated_csd", "gnn_residual_csd",
            "gnn_uncertainty_residual_csd", "learned_gnn_gru_graph_shield",
            "gnn_gru_residual_sac")
        if self.graph_mode == "gnn_uncertainty_residual_csd":
            return requested and self.residual_authority > 1e-6
        return requested

    def own_state_features(self, index):
        vessel = self.states[index]
        c, s = math.cos(vessel.heading), math.sin(vessel.heading)
        world_to_body = np.array([[c, s], [-s, c]])
        goal_body = world_to_body @ (self.goals[index] - vessel.position)
        return np.array([
            goal_body[0] / 50.0,
            goal_body[1] / 50.0,
            np.linalg.norm(self.goals[index] - vessel.position) / 100.0,
            np.linalg.norm(vessel.velocity) / 2.4,
        ])

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def gps_callback(self, msg, index):
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        state = self.states[index]
        now = self.now_seconds()
        earth_radius = 6378137.0
        x = math.radians(msg.longitude - self.origin_longitude) * earth_radius
        x *= math.cos(math.radians(self.origin_latitude))
        y = math.radians(msg.latitude - self.origin_latitude) * earth_radius
        position = np.array([x, y], dtype=float) - self.origin
        if state.initialized:
            dt = max(now - state.stamp, 1e-3)
            measured_velocity = (position - state.position) / dt
            state.velocity = 0.65 * state.velocity + 0.35 * measured_velocity
        state.position = position
        state.stamp = now
        state.gps_ready = True
        state.initialized = state.gps_ready and state.imu_ready

    def imu_callback(self, msg, index):
        state = self.states[index]
        state.heading = yaw_from_quaternion(msg.orientation)
        state.imu_ready = True
        state.initialized = state.gps_ready and state.imu_ready

    @staticmethod
    def domain_support(state, bearing, future, disturbance_growth=0.0):
        relative = math.atan2(math.sin(bearing - state.heading), math.cos(bearing - state.heading))
        forward = max(math.cos(relative), 0.0)
        aft_component = max(-math.cos(relative), 0.0)
        lateral_component = abs(math.sin(relative))
        speed = float(np.linalg.norm(state.velocity))
        longitudinal = 2.7 + 1.2 + 2.4 * speed * (0.25 + 0.75 * forward)
        aft = 2.7 + 0.6 * 1.2
        lateral = 1.35 + 1.0 + 0.25 * speed
        directional = forward * longitudinal + aft_component * aft
        return math.hypot(directional, lateral_component * lateral) + (0.10 + disturbance_growth) * math.sqrt(max(future, 0.0))

    def conflict(self, first, second):
        steps = np.arange(0.0, 16.0 + 1e-9, 0.5)
        p1 = first.position[None, :] + steps[:, None] * first.velocity[None, :]
        p2 = second.position[None, :] + steps[:, None] * second.velocity[None, :]
        relative = p2 - p1
        clearance = []
        for k, future in enumerate(steps):
            bearing = math.atan2(relative[k, 1], relative[k, 0])
            radius = self.domain_support(first, bearing, future)
            radius += self.domain_support(second, bearing + math.pi, future)
            clearance.append(float(np.linalg.norm(relative[k]) - radius))
        best = int(np.argmin(clearance))
        minimum = clearance[best]
        current_clearance = clearance[0]
        tcpa = float(steps[best])
        spatial = 1.0 / (1.0 + math.exp(np.clip(minimum / 1.8, -60.0, 60.0)))
        predicted = spatial * (0.30 + 0.70 * math.exp(-tcpa / 16.0))
        calibration_margin = (
            0.0 if self.graph_mode == "conditional_no_conformal"
            else self.conformal_quantile
        )
        return {
            "distance": float(np.linalg.norm(second.position - first.position)),
            "clearance": minimum,
            "current_clearance": current_clearance,
            "tcpa": tcpa,
            "risk": min(1.0, predicted + calibration_margin),
        }

    def pair_features(self, ego, neighbor, conflict, neighbor_index):
        c, s = math.cos(ego.heading), math.sin(ego.heading)
        world_to_body = np.array([[c, s], [-s, c]])
        relative_position = world_to_body @ (neighbor.position - ego.position)
        relative_velocity = world_to_body @ (neighbor.velocity - ego.velocity)
        basic = [
            relative_position[0] / 80.0, relative_position[1] / 80.0,
            relative_velocity[0] / 5.0, relative_velocity[1] / 5.0,
            conflict["distance"] / 100.0, conflict["risk"],
            np.clip(conflict["clearance"], -20.0, 50.0) / 50.0,
            conflict["tcpa"] / 16.0, np.linalg.norm(neighbor.velocity) / 2.4,
        ]
        if int(getattr(self.model, "pair_dim", 9)) < 15:
            return np.asarray(basic)
        # The harbor-trained model distinguishes cooperative traffic from
        # prescribed noncooperative traffic through a role one-hot vector.
        cooperative = neighbor_index < self.cooperative_count
        return np.asarray([
            relative_position[0] / 100.0, relative_position[1] / 100.0,
            relative_velocity[0] / 5.0, relative_velocity[1] / 5.0,
            conflict["distance"] / 130.0, conflict["risk"],
            np.clip(conflict["clearance"], -20.0, 80.0) / 80.0,
            conflict["tcpa"] / 16.0, np.linalg.norm(neighbor.velocity) / 2.4,
            float(cooperative), 0.0, float(not cooperative),
            np.clip(conflict["current_clearance"], -20.0, 100.0) / 100.0,
            2.7 / 10.0, 1.35 / 5.0,
        ])

    def build_ego_graph(self, ego_index, valid_indices, now):
        n = len(self.states)
        pair = np.zeros((n, self.model.pair_dim))
        conflicts = [None] * n
        hard = np.zeros(n, dtype=bool)
        for neighbor_index in valid_indices:
            if neighbor_index == ego_index:
                continue
            conflict = self.conflict(self.states[ego_index], self.states[neighbor_index])
            conflicts[neighbor_index] = conflict
            pair[neighbor_index] = self.pair_features(
                self.states[ego_index], self.states[neighbor_index], conflict,
                neighbor_index)
            if not self.uses_gnn:
                encoded = self.model.pair_encode(pair[neighbor_index])
                if self.graph_mode == "conditional_no_gru":
                    self.hidden[ego_index, neighbor_index] = encoded
                else:
                    self.hidden[ego_index, neighbor_index] = self.model.gru(
                        encoded, self.hidden[ego_index, neighbor_index])
            hard[neighbor_index] = conflict["risk"] >= 0.65 or conflict["clearance"] <= 0.5

        selected = []
        predictions = {}
        neighbors = [index for index in valid_indices if index != ego_index]
        if self.graph_mode in (
                "learned_gnn_gru_graph_shield", "gnn_gru_residual_sac"):
            if neighbors:
                probabilities = self.model.edge_probabilities(
                    self.hidden[ego_index, neighbors])
                for row, neighbor_index in enumerate(neighbors):
                    active_probability = float(probabilities[row, 2])
                    predicted_class = int(np.argmax(probabilities[row]))
                    # Match the MATLAB learned edge lifecycle in nominal
                    # states, while preserving an independent geometric
                    # safety shield for imminent ship-domain violations.
                    shield_required = bool(
                        conflicts[neighbor_index]["current_clearance"] <= 8.0
                        or conflicts[neighbor_index]["clearance"] <= 7.0
                        or conflicts[neighbor_index]["risk"] >= 0.65
                    )
                    is_selected = predicted_class == 2 or shield_required
                    if is_selected:
                        selected.append(neighbor_index)
                    predictions[neighbor_index] = (
                        active_probability, math.nan, active_probability,
                        conflicts[neighbor_index]["clearance"], is_selected)
        elif self.graph_mode in ("distance_star", "risk_star", "full_graph"):
            if self.graph_mode == "distance_star":
                ranked = sorted(
                    neighbors, key=lambda index: conflicts[index]["distance"])
                selected = ranked[:self.heuristic_edge_budget]
            elif self.graph_mode == "risk_star":
                ranked = sorted(
                    neighbors,
                    key=lambda index: (
                        conflicts[index]["risk"],
                        -conflicts[index]["clearance"],
                    ),
                    reverse=True,
                )
                selected = ranked[:self.heuristic_edge_budget]
            else:
                selected = neighbors
            for neighbor_index in neighbors:
                conflict = conflicts[neighbor_index]
                is_selected = neighbor_index in selected
                predictions[neighbor_index] = (
                    math.nan,
                    math.nan,
                    conflict["risk"],
                    conflict["clearance"],
                    is_selected,
                )
        else:
            budget = min(self.model.edge_budget, len(neighbors))
            for slot in range(budget):
                if selected:
                    context_hidden = (
                        self.model.graph_message(self.hidden[ego_index, selected])
                        if self.uses_gnn
                        else self.hidden[ego_index, selected].mean(axis=0)
                    )
                    context_pair = pair[selected].mean(axis=0)
                else:
                    context_hidden = np.zeros(self.model.hidden_dim)
                    context_pair = np.zeros(9)
                eligible = []
                for neighbor_index in neighbors:
                    if neighbor_index in selected:
                        continue
                    one_hot = np.zeros(3)
                    one_hot[self.edge_state[ego_index, neighbor_index]] = 1.0
                    feature_parts = [
                        self.hidden[ego_index, neighbor_index], pair[neighbor_index],
                        one_hot, context_hidden, context_pair,
                    ]
                    if self.uses_gnn:
                        neighbor_context = [
                            index for index in valid_indices
                            if index != neighbor_index
                        ]
                        feature_parts.append(self.model.graph_message(
                            self.hidden[neighbor_index, neighbor_context]))
                    feature_parts.append(np.array([
                        len(selected) / max(self.model.edge_budget, 1)
                    ]))
                    features = np.concatenate(feature_parts)
                    safety, efficiency, probability = self.model.predict(features)
                    conformal_q = (
                        0.0 if self.graph_mode == "conditional_no_conformal"
                        else float(self.model.safety_conformal_q)
                    )
                    lower = safety - conformal_q
                    if self.uses_gnn:
                        # The graph classifier is used conservatively: either a
                        # calibrated critical prediction or a positive lower
                        # bound can admit an edge. Hard geometric risk remains
                        # the final independent safety gate.
                        is_eligible = bool(
                            hard[neighbor_index]
                            or probability >= float(
                                self.model.calibrated_critical_threshold)
                            or lower >= 0.0
                        )
                    else:
                        is_eligible = bool(hard[neighbor_index] or (
                            probability >= float(
                                self.model.calibrated_critical_threshold)
                            and lower >= 0.0))
                    predictions[neighbor_index] = (
                        safety, efficiency, probability, lower, is_eligible)
                    if is_eligible:
                        priority = 1000.0 * safety + efficiency
                        if probability >= float(
                                self.model.calibrated_critical_threshold):
                            priority += 1e6
                        if hard[neighbor_index]:
                            priority += 1e9 + 1000.0 * conflicts[neighbor_index]["risk"]
                        eligible.append((priority, neighbor_index))
                if not eligible:
                    break
                selected.append(max(eligible)[1])

        previous = self.edge_state[ego_index].copy()
        next_state = np.zeros(n, dtype=int)
        next_state[selected] = 2
        for neighbor_index in valid_indices:
            if neighbor_index == ego_index or next_state[neighbor_index] == 2:
                self.dormant_steps[ego_index, neighbor_index] = 0
                continue
            if previous[neighbor_index] in (1, 2):
                self.dormant_steps[ego_index, neighbor_index] += 1
                if self.dormant_steps[ego_index, neighbor_index] <= 8:
                    next_state[neighbor_index] = 1
        self.edge_state[ego_index] = next_state

        for neighbor_index in valid_indices:
            if neighbor_index == ego_index:
                continue
            safety, efficiency, probability, lower, is_eligible = predictions.get(
                neighbor_index, (math.nan, math.nan, math.nan, math.nan, False))
            conflict = conflicts[neighbor_index]
            self.edge_writer.writerow([
                f"{now:.3f}", self.names[ego_index], self.names[neighbor_index],
                f"{conflict['distance']:.6f}", f"{conflict['tcpa']:.3f}",
                f"{conflict['clearance']:.6f}", f"{conflict['risk']:.6f}",
                int(hard[neighbor_index]), f"{probability:.6f}", f"{safety:.6f}",
                f"{lower:.6f}", f"{efficiency:.6f}", int(is_eligible),
                int(neighbor_index in selected),
                selected.index(neighbor_index) + 1 if neighbor_index in selected else 0,
                int(previous[neighbor_index]), int(next_state[neighbor_index]),
                self.graph_mode,
            ])

        recommendations = []
        action_advice = None
        if self.graph_mode in (
                "gnn_residual_csd", "gnn_uncertainty_residual_csd") \
                and self.uses_gnn and selected:
            action_advice = self.model.action_advice(
                self.own_state_features(ego_index),
                self.hidden[ego_index, selected],
            )
        for neighbor_index in selected:
            safety, efficiency, probability, lower, _ = predictions[neighbor_index]
            conflict = conflicts[neighbor_index]
            recommendations.extend([
                float(ego_index), float(neighbor_index),
                float(conflict["risk"]), float(conflict["clearance"]),
                float(lower), float(hard[neighbor_index]),
                float(conflict["tcpa"]), float(probability),
            ])
            if action_advice is not None:
                recommendations.extend(action_advice)
            elif self.graph_mode == "gnn_uncertainty_residual_csd":
                # Preserve the 11-value message contract while the adaptive
                # gate has fallen back to the conditional CSD graph.
                recommendations.extend((0.0, 1.0, 0.0))
        return recommendations

    def update_gnn_pair_memory(self, valid_indices):
        for ego_index in valid_indices:
            for neighbor_index in valid_indices:
                if neighbor_index == ego_index:
                    continue
                conflict = self.conflict(
                    self.states[ego_index], self.states[neighbor_index])
                pair = self.pair_features(
                    self.states[ego_index], self.states[neighbor_index], conflict,
                    neighbor_index)
                encoded = self.model.pair_encode(pair)
                self.hidden[ego_index, neighbor_index] = self.model.gru(
                    encoded, self.hidden[ego_index, neighbor_index])

    def step(self):
        now = self.now_seconds()
        valid = [
            i for i, state in enumerate(self.states)
            if state.initialized and now - state.stamp <= self.stale_timeout
        ]
        if len(valid) < 2:
            return
        if self.uses_gnn:
            self.update_gnn_pair_memory(valid)
        active_edges = []
        for index in valid:
            state = self.states[index]
            self.state_writer.writerow([
                f"{now:.3f}", state.name, f"{state.position[0]:.6f}",
                f"{state.position[1]:.6f}", f"{state.velocity[0]:.6f}",
                f"{state.velocity[1]:.6f}", f"{state.heading:.6f}",
                f"{now - state.stamp:.6f}",
            ])
            active_edges.extend(self.build_ego_graph(index, valid, now))
        message = Float64MultiArray()
        message.data = active_edges
        self.active_edge_publisher.publish(message)
        self.edge_file.flush()
        self.state_file.flush()

    def destroy_node(self):
        self.edge_file.close()
        self.state_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = CSDShadowGraph()
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
