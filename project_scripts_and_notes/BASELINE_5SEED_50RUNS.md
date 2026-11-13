# Calm and medium baseline batch

This batch runs four comparison methods in `calm` and `medium_wind_wave`:

- `distance_star`
- `risk_star`
- `conditional_csd`
- `gnn_gru`

The five seeds are `20261011`, `20261014`, `20261017`, `20261018`, and
`20261027`. Each seed is repeated ten times, giving 50 runs for every
sea-state/method pair and 400 runs in total.

All runs use `install_v6`, stable prescribed shuttle traffic, headless Gazebo,
and the same body-aware post-processing. Completed cells survive interruption.

Start:

```bash
bash start_baseline_5seed_50runs.sh
```

Watch:

```bash
python3 watch_baseline_5seed_50runs.py
```

Stop safely:

```bash
bash stop_baseline_5seed_50runs.sh
```

Restart with the start command to resume. Final run-level and aggregate tables
are written under `results/baseline_5seed_50runs_20261030/`.
