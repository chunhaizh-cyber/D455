# Slow Pan ROI Budget Check 001 Decision

Decision: do not promote hand-occlusion multi-ROI probes to motion overall.

| candidate | pass | score | p95 ms | max ms | near | far | ROI count |
|---|---:|---:|---:|---:|---:|---:|---:|
| candidate_0030 | true | 93.390 | 70.7170 | 85.811 | 20 | 10 | 0 |
| candidate_0041 | true | 81.288 | 98.0776 | 128.595 | 16 | 6 | 2 |
| candidate_0040 | true | 81.108 | 99.2818 | 116.919 | 16 | 6 | 2 |

`candidate_0040` and `candidate_0041` are useful hand-occlusion ROI probes, but the low 3% motion threshold and multi-ROI refresh hurt slow-pan quality and latency. `candidate_0030` remains the motion bucket best.
