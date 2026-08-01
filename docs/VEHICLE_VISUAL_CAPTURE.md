# Vehicle Visual Evidence Capture

## Purpose

This capture profile prepares real D455 evidence for reviewing:

```text
stable observation material
+ camera/vehicle motion
+ relative IMU pose and motion diagnostics
+ occlusion or temporary loss
+ newly occupied image/space regions
```

It supports the feasibility review in:

```text
D:\数字生命思路\方案\方案_视觉系统鱼巢实现方案_待核验可行性_20260625.md
```

The recording is evidence material. It does not write Fish Nest world facts and does not by itself validate `SceneVoxelPrior`.

## Current capture contract

Run from `D:\D455`:

```powershell
.\tools\Record-VehicleMenu.cmd
```

The menu, scenario names, operating guidance and input prompt are displayed in Chinese. Enter `1` through `9`; the selected recording starts after a short countdown and stops at its fixed frame budget. After completion, the menu is shown again. Enter `q` to exit.

For unprocessed sensor evidence, record a native RealSense `.bag` instead of the five-panel dashboard:

```powershell
.\x64\Release\D455.exe --record-raw-bag=recordings\raw_vehicle_motion.bag --record-raw-seconds=60
```

This path records the 640x480@30 color, depth, left/right infrared streams and available accelerometer/gyroscope streams directly through librealsense. It does not run segmentation, contour extraction, depth filtering, dashboard composition or video overlays.

Verify a completed recording without exporting or transforming its frames:

```powershell
.\x64\Release\D455.exe --inspect-raw-bag=recordings\raw_vehicle_motion.bag
```

Inspection scans each playback sensor for the full recording and reports the actual frame count, non-monotonic timestamp count, estimated missing-frame count and maximum gap for every stream. It counts accelerometer and gyroscope packets directly instead of inferring IMU completeness from video framesets.

Create a deterministic directory replay sample for the existing evaluation pipeline while leaving the raw bag unchanged:

```powershell
.\x64\Release\D455.exe `
  --convert-raw-bag=recordings\raw_vehicle_motion.bag `
  --convert-raw-bag-dir=analysis_runs\generated_cases\raw_vehicle_motion `
  --convert-raw-bag-start-frame=0 `
  --convert-raw-bag-every-n=15 `
  --convert-raw-bag-max-frames=120
```

The derived replay aligns raw depth to color coordinates and converts depth units to millimeters because that is the current directory replay contract. It does not replace or rewrite the authoritative `.bag`. `source_timestamps.csv` records the source frameset and stream timestamps for every derived frame. Use `--convert-raw-bag-start-frame=N --convert-raw-bag-every-n=1` to preserve a contiguous 30 fps validation window; sparse sampling is only suitable for spatial timeline review, not temporal scoring.

| Key | Scenario | Default frames | Operating constraint |
| ---: | --- | ---: | --- |
| 1 | Parked static baseline | 300 | Vehicle remains parked |
| 2 | Slow straight motion | 600 | Safe normal driving only |
| 3 | Turn and pose change | 600 | Safe normal turn |
| 4 | Normal road vibration | 600 | Do not seek harsh bumps |
| 5 | Occlusion and reappearance | 450 | Parked; passenger operates target |
| 6 | New occupied region | 450 | Parked; passenger places/removes object |
| 7 | Near/far transition | 600 | Camera remains rigidly mounted |
| 8 | Natural lighting transition | 600 | Do not cover lens while driving |
| 9 | Mixed route sequence | 900 | Parked start and parked ending |

For direct single-session capture without the menu:

```powershell
.\tools\Capture-VehicleVisualEvidence.ps1 -Frames 900
```

Dry-run the command and output paths without querying the camera:

```powershell
.\tools\Capture-VehicleVisualEvidence.ps1 -Frames 900 -PlanOnly
.\tools\Capture-VehicleVideoMenu.ps1 -Choice 1 -PlanOnly
```

The default 900 processed frames are nominally 30 seconds at 30 fps. Actual wall-clock duration may be longer when processing cannot sustain 30 fps.

Each session is written under the ignored directory:

```text
recordings/vehicle_visual_YYYYMMDD_HHMMSS/
  dashboard.avi
  acceptance.csv
  profile.csv
  console.log
  session_manifest.json
```

`acceptance.csv` includes the latest accel/gyro sample timestamps, roll, pitch, relative yaw, acceleration vector, gyro vector and motion diagnostics for each processed RGBD frame. `profile.csv` preserves per-frame processing cost. The manifest records branch, commit, command and output sizes.

## Safe operating sequence

1. Mount the D455 rigidly before the vehicle moves. Avoid glass reflections, dashboard obstruction and a loose USB cable.
2. Connect power and confirm at least 10 GiB free on the output drive.
3. Start the script while parked. The script probes the D455 before creating the session.
4. Keep the vehicle and camera still for the opening segment so the recording contains a motion baseline.
5. Let the vehicle move normally. Useful evidence includes a slow straight segment, a turn, normal vibration and a full stop.
6. Do not operate the computer while the vehicle is moving. A passenger should supervise capture if supervision is needed.
7. Let the fixed frame budget stop the session, or press `q` only after the vehicle is safely stopped.

Recommended material sequence inside one clip:

```text
parked baseline
→ gentle start
→ slow translation
→ turn or viewpoint change
→ temporary foreground occlusion if safely available
→ stop and remain still
```

Do not stage an occlusion or interact with the camera while driving.

## Evidence mapping

| Scheme question | Capture evidence | Boundary |
| --- | --- | --- |
| D455 observation remains stable | dashboard video, acceptance metrics | Describes D455 output; does not confirm a Fish Nest fact |
| Camera pose is available | pose fields and IMU timestamps in `acceptance.csv` | Yaw is relative and may drift |
| Motion can explain changed image state | visual motion plus gyro/accel diagnostics | Vehicle acceleration is not pure gravity |
| Occlusion/reappearance can be separated from disappearance | contour video and per-frame cluster/track metrics | Requires later review or replay scoring |
| New occupied regions can be proposed | cluster-map and segmentation views | A new occupied region remains a candidate |
| Processing is timely enough | `profile.csv` p50/p95/max | Must still pass the repository performance gate |

## Important limitation

This profile records the rendered dashboard and synchronized metrics, but it does not save deterministic raw directory replay frames. The existing `--capture-replay-dir` mode saves color, aligned depth16, left IR and right IR, but it runs as a separate capture path and currently does not save IMU or the rendered dashboard.

Therefore:

```text
vehicle dashboard session
  = best current material for visible behavior + pose/motion timing

directory replay session
  = best current material for deterministic algorithm reruns
```

They must not be described as the same synchronized capture. If the feasibility review requires raw RGBD/IR and IMU from exactly the same frames, the next implementation should add RealSense bag recording or extend directory capture with synchronized IMU records before collecting the formal gate dataset.

## Post-capture checks

The script verifies that video, acceptance CSV and profile CSV are non-empty. Before using a session as evidence, also check:

```text
session_manifest.json status == complete
acceptance.csv contains pose_accel_valid and pose_gyro_valid samples
video includes parked baseline and moving interval
profile.csv contains enough frames for p95
camera did not shift relative to its mount
no private or identifying material should be uploaded without review
```

Recordings remain ignored by Git. Uploading or moving a real vehicle recording requires a separate privacy and size decision.
