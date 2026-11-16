# Completed VRX run data snapshot (2026-11-16)

This snapshot contains **750 completed runs** copied from the local research workspace. It does not include the strong-sea `conditional_csd` and `gnn_gru` batches, which were still running when this snapshot was made.

| Cohort | Runs | Methods | Sea state |
| --- | ---: | --- | --- |
| `strong_h53_50` | 50 | H53 GNN-GRU + residual SAC recovery candidate | strong wind and wave |
| `strong_baselines_200` | 200 | Distance-Star 100, Risk-Star 100 | strong wind and wave |
| `medium_c119_100` | 100 | Original C119 GNN-GRU + residual SAC | medium wind and wave |
| `medium_baselines_400` | 400 | Distance-Star, Risk-Star, Conditional CSD, GNN-GRU; 100 each | medium wind and wave |

`manifest.csv` lists every run and maps its original run name to the archived trajectory, configuration, world, and available audit. Trajectories are complete `six_vessel_driver.csv` files compressed with gzip; use `gzip -dc file.csv.gz` to read them. The SHA-256 field hashes the **uncompressed** trajectory. Each run also has its actual `csd_harbor.sdf` and `experiment_config.csv`. `tables/` preserves the source campaign result tables and summaries.

The strong-sea baseline world uses the same static-wall depth as H53: wall boxes are 6 m high centered at z=0, keeping their top at z=3 m and extending the lower edge to z=-3 m. Both strong cohorts used GUI Gazebo runs on the replacement computer. The strong baselines used the local scalar-SAT runtime recovery; H53 is a separate local method candidate. Do not relabel the H53 results as original C119 results. The medium C119 100-run table combines 50 existing confirmatory runs with 50 matched confirmatory runs, as recorded in its `source` column.

The `status` column in the medium C119 table is a navigation/safety classification; `FAIL` there does not mean that the run file is missing. The strong H53 `ship_overlap` and `wall_overlap` columns count sampled body-overlap frames. The baseline summary's `collision_free_rate` includes wall contact, so it is not interchangeable with vessel-vessel collision-free rate. A zero sampled-overlap count is not a proof of continuous-time separation.

The existing repository file `data_export/calm_500_runs.csv` is the separate calm-sea metric table. The running strong-sea `conditional_csd` and `gnn_gru` campaigns should be uploaded in a later snapshot after their full cohorts complete.
