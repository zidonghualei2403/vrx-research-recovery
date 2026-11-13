# Latest calm and medium-sea results

## Overall results (5 seeds x 10 repetitions)

| Sea state | Version | Runs | 4/4 arrival | Vessel-safe | Wall-safe | Strict safety | Strict success <=130 s | Fleet time, all runs | Fleet time, strictly safe runs |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Calm | Locked V6 | 50 | 50/50 (100%) | 44/50 (88%) | 46/50 (92%) | 40/50 (80%) | 40/50 (80%) | 97.79 +/- 8.14 s | 96.25 +/- 6.04 s |
| Medium wind/wave | C119 | 50 | 50/50 (100%) | 45/50 (90%) | 44/50 (88%) | 38/50 (76%) | 34/50 (68%) | 127.06 +/- 9.03 s | 124.48 +/- 6.15 s |

Strict safety requires 4/4 arrival, zero vessel overlap, zero wall overlap, and no distant 20-second stall. Strict success additionally requires fleet completion within 130 seconds.

## Per-seed results

| Sea state | Seed | Strictly safe | <=130 s strict success | Mean time of strictly safe runs |
|---|---:|---:|---:|---:|
| Calm | 20261101 | 8/10 | 8/10 | 96.48 s |
| Calm | 20261102 | 10/10 | 10/10 | 95.49 s |
| Calm | 20261110 | 7/10 | 7/10 | 96.03 s |
| Calm | 20261117 | 9/10 | 9/10 | 96.35 s |
| Calm | 20261123 | 6/10 | 6/10 | 97.31 s |
| Medium | 20261101 | 9/10 | 8/10 | 126.77 s |
| Medium | 20261102 | 9/10 | 9/10 | 121.51 s |
| Medium | 20261110 | 8/10 | 6/10 | 125.38 s |
| Medium | 20261121 | 5/10 | 4/10 | 125.77 s |
| Medium | 20261123 | 7/10 | 7/10 | 123.43 s |

## Reporting note

The calm and medium rows use different frozen controller variants: locked V6 for calm water and C119 for medium wind/wave. They should be described as sea-state-specific validated variants unless a single unchanged controller is later tested across both sea states.
