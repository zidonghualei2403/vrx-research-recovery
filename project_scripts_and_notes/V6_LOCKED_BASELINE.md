# Locked Harbor V6 baseline

Use `run_locked_v6.sh` for all visible validation runs. It loads the verified
V6 driver, GNN-GRU model, residual SAC actor, and residual authority of 0.35.

Verified seed: `20261014`.

| Sea state | Cooperative arrivals | Fleet arrival time |
| --- | ---: | ---: |
| Calm | 4/4 | 101.94 s |
| Medium wind/wave | 4/4 | 104.69 s |
| Strong wind/wave | 4/4 | 160.85 s |
| Extreme wind/wave | 4/4 | 245.96 s |

The V7/V8 safety-gate experiments did not improve V6. V8 in particular made
the controller over-conservative, reduced progress, and failed to initialize
the extreme-sea run. They must not be used as paper results or as the base for
new tuning.

Locked artifact checksums:

```text
e4dfd79a85e94d899f35786487888f16732472037a0cb4e604843842555c25c9  six_vessel_crossing_driver.py
82d19bbf6b63dd1695eec438ab3e368c4958faf4ac6fd2cd85bca18263357148  harbor_residual_sac_actor.json
862294f60ad23bb2b4ea76d181e664bc585357b8507c4dcce31d077e4422ec92  harbor_gnn_gru_best.json
```
