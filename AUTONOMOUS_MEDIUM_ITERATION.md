# Autonomous medium-sea controller iteration

## Objective

Starting only from the checksum-protected locked V6 controller, autonomously
produce a reproducible medium-wind/wave candidate that is both safe and fast in
VRX. This is controller repair and validation, not GNN-GRU or SAC retraining.

## Fixed test

- Sea state: `medium_wind_wave`
- Primary seed: `20261018`
- Full-run horizon: `300 s`
- Cooperative vessels: `wamv_1` through `wamv_4`
- Prescribed shuttle vessels: `wamv_5` and `wamv_6`
- Physical hull audit: oriented rectangle, length `5.4 m`, width `2.7 m`
- Starting point: `install_v6`, whose checksums are listed in
  `V6_LOCKED_BASELINE.md`

## Acceptance gate

A candidate passes the primary gate only if all conditions hold:

1. All six vessels are spawned and produce valid pose samples.
2. All four cooperative vessels arrive within `300 s`.
3. Physical vessel-overlap frames are zero.
4. Physical wall-overlap frames are zero.
5. No cooperative vessel remains nearly stationary away from its goal for
   `20 s`, and no sustained orbit or reciprocal stand-off is detected.
6. Fleet completion time is no more than `115.2 s` (locked V6 medium baseline
   `104.69 s` plus ten percent).

After the primary gate passes, rerun the unchanged candidate with seeds
`20261014` and `20261017`. A release candidate requires all three seeds to pass
conditions 1-5; report completion-time mean and spread instead of hiding a bad
seed.

## Iteration policy

1. Preserve all completed comparison-batch results, then stop that batch. It is
   not the objective of this loop.
2. Reproduce locked V6 first and audit it with
   `audit_existing_vrx_results.py`. Never edit `install_v6` or its model files.
3. Each candidate gets a new immutable directory under
   `autonomous_medium_iterations_20261018/iter_NN/`, including source diff,
   build log, launch log, CSV, audit CSV, metrics JSON, and decision note.
4. Change one mechanism family per iteration. The permitted order is:
   - log-only body-aware diagnostics, with no behavior change;
   - closing-rate/TCPA activation for rear and crossing threats;
   - deterministic unilateral right-of-way;
   - release hysteresis and progress recovery.
5. Do not add symmetric emergency reverse, global stop-on-distance logic, or a
   globally enlarged pair domain. Earlier trials showed these cause stand-off
   and can push vessels back into the shuttle lane.
6. Use a short smoke run only to reject startup, spawning, or immediate-control
   failures. Final acceptance always requires the complete `300 s` run or an
   earlier clean four-vessel completion.
7. Stop after eight failed behavior-changing candidates. Mark the loop blocked
   with a ranked failure analysis instead of silently widening thresholds.

## State and reporting

The machine-readable state is `autonomous_medium_iterations_20261018/state.json`.
Update it atomically after every stage. `best_candidate` may be set only for a
candidate that passes the primary gate. A candidate that is collision-free but
misses arrival or time is recorded as `safe_but_overconservative`, not success.

