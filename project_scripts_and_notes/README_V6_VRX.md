# Harbor SAC V6 VRX validation

This workspace is isolated from every earlier successful workspace. It loads the selected 250k Harbor SAC V6 actor and reproduces its observation schema, residual scale, joint finite-horizon safety projection, and predicted-advantage gate.

## First visible smoke test

```bash
conda deactivate 2>/dev/null || true
cd /home/hzh/Documents/Codex/2026-08-13/hao/vrx_harbor_sac_v6_aligned_ws_v1
bash run_harbor_gnn_gru_vrx.sh calm 20261014 gnn_gru_sac
```

Expected scene: four cooperative WAM-V vessels, two reciprocating WAM-V vessels, and two rectangular static obstacles.

Promotion criteria:

- all six vessels spawn and obtain poses;
- the two traffic vessels reciprocate instead of stopping or circling;
- no hull-hull or hull-wall collision;
- all four cooperative vessels arrive;
- SAC residual commands are accepted only after the V6 joint safety and advantage gate.

Do not run the multi-seed batch until this visible test passes.

## Offline evidence

The selected model and formal 160-episode evaluation are stored in:

`/home/hzh/Documents/Codex/2026-08-13/hao/harbor_shuttle_gnn_gru_sac_v6_advantage_gate`
