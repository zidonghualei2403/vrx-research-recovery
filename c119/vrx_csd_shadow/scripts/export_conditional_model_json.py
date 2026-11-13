#!/usr/bin/env python3
import json
from pathlib import Path

import numpy as np
from scipy.io import loadmat


ROOT = Path(__file__).resolve().parents[4]
source = ROOT / "csd_lfcg_gnn_gru_train_v1" / (
    "outputs_conditional_counterfactual_lexicographic_scale300_v1/"
    "models/conditional_counterfactual_best_matlab.mat"
)
target = Path(__file__).resolve().parents[1] / "models" / "conditional_strict_model.json"

required_prefixes = ("pair_encoder_", "pair_memory_", "conditional_head_")
required_scalars = {
    "hidden_dim", "safety_transform_mean", "safety_transform_std",
    "efficiency_transform_mean", "efficiency_transform_std",
    "safety_conformal_q", "efficiency_conformal_q", "conformal_alpha",
    "calibrated_critical_threshold", "edge_budget",
}

data = loadmat(source, squeeze_me=True)
output = {"method": "conditional_counterfactual_strict_graph"}
for key, value in data.items():
    if key.startswith(required_prefixes) or key in required_scalars:
        array = np.asarray(value)
        output[key] = array.item() if array.ndim == 0 else array.tolist()

target.parent.mkdir(parents=True, exist_ok=True)
target.write_text(json.dumps(output, separators=(",", ":")), encoding="utf-8")
print(target)
