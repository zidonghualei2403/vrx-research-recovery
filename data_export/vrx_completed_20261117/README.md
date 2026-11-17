# Completed VRX data snapshot (2026-11-17)

This snapshot adds 339 completed runs to the earlier 750-run snapshot:

| Cohort | Runs | Meaning |
| --- | ---: | --- |
| `six_central_v7_75` | 75 | Five methods × three sea states × five common seeds, one run per key |
| `strong_baselines_extra200` | 200 | Conditional CSD and GNN-GRU strong-sea runs, 100 each |
| `strong_h53_extra50` | 50 | H53 strong-sea repetitions 11–20; repetitions 1–10 are in the earlier snapshot |
| `medium_m54_selected14` | 14 | Separate deep-wall reruns of 14 original C119 non-arrivals |

The formal six-vessel seed list is 20261101, 20261102, 20261110, 20261117, 20261123. The earlier 20261011–20261013 six-vessel pilot runs are excluded from this formal comparison. All five methods use the V7 common nominal controller and actuator path; only graph selection and the SAC residual toggle vary. The world, start and goal positions, 300 s limit, and 6 m first-arrival definition are shared. The SAC actor has 108 observation inputs and 12 action outputs. `code/` holds the tested V7 driver, graph node, weights, runner, and auditing code with SHA-256 checksums.

`manifest.csv` gives each archived trajectory, configuration and world file with a SHA-256 hash of the uncompressed trajectory. Six-vessel runs also include compressed graph edge and state logs and a per-run audit. The table `six_central_v7_75.csv` reports the final fleet arrival time as the last vessel's first entry into its 6 m goal region. `observation_duration_s` includes the post-arrival observation window; these time columns must not be interchanged. `center_overlap_samples_under_5p4m` is a center-distance screen. The separate oriented-hull audit uses 2.7 m × 1.35 m half-extents and four separating axes at each synchronized six-vessel sample; its signed SAT gap is not the exact Euclidean surface distance.

The strong baseline table has 400 rows across four methods. This snapshot stores only the 200 newly completed Conditional CSD and GNN-GRU trajectories because the Distance-Star and Risk-Star trajectories are already in `vrx_completed_20261116`. The H53 table has all 100 rows, while this snapshot stores only repetitions 11–20. The M54 selected-failure reruns are a separate intervention and must not replace observations in the original C119 100-run rate. Original local run paths are reduced to run names in public tables. ROS logs, user credentials, and the mobile SSD contents are not uploaded.
