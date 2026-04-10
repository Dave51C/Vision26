# VISION_PIPELINE_FIXES.md
**Required Fixes For Vision26.py / PiggyVision26.py**
**Scope: Camera pipeline, pose math, metadata, and NetworkTables contract**

This document lists the changes still needed in the current vision pipeline. It includes both the current final-pose path and the planned per-tag path.

---

## 1. High Priority Fixes

### 1.1 Ambiguity Formula
**Current issue:**
The current ambiguity formula behaves opposite to the usual meaning of ambiguity.

Current code:
```python
ambiguity = abs(err1 - err2) / (err1 + err2 + 1e-6)
```

This gives:
- low values when both solutions are similar
- higher values when one solution is clearly better

That is closer to a confidence gap than an ambiguity score.

**Required change:**
Use an ambiguity metric where higher means worse.

Recommended replacement:
```python
errs = sorted(pnperrs)
err1, err2 = errs[0], errs[1]
ambiguity = err1 / (err2 + 1e-6)
```

Interpretation:
- ambiguity near `1.0` = high ambiguity
- ambiguity near `0.0` = low ambiguity

---

### 1.2 Rejection Gate
**Current issue:**
The rejection gate is commented out, so bad solves can still pass through.

**Required change:**
Restore or replace the rejection gate before publishing pose data.

Recommended starter checks:
- reject if `num_tags == 0`
- reject if `avg_reproj_error` is too high
- reject if ambiguity is too high
- reject if any position or angle is physically unrealistic

Suggested starter logic:
```python
if num_tags > 1:
    amb_thresh = 0.5
else:
    amb_thresh = 0.3

if (
    avg_reproj_error is None
    or avg_reproj_error > 5.0
    or avg_ambiguity > amb_thresh
    or num_tags == 0
):
    return None
```

---

### 1.3 Tag Count Source Of Truth
**Current issue:**
`num_tags` and `tag_count = len(tag_ids)` can disagree.

Current code:
```python
self.num_tags = num_tags
self.tag_ids = tag_ids or []
self.tag_count = len(self.tag_ids)
```

**Required change:**
Use one source of truth.

Recommended fix:
```python
self.num_tags = num_tags
self.tag_ids = tag_ids or []
self.tag_count = num_tags
```

Alternative:
- remove `tag_count` entirely and always use `num_tags`

---

### 1.4 Front Camera Table Mismatch
**Current issue:**
`VisionTable.cameras` only creates:
- `LeftCam`
- `RightCam`
- `BackCam`

But the code still supports `FrontCam` elsewhere.

**Required change:**
Either:
- add `FrontCam` to the `VisionTable.cameras` dictionary
- or remove `FrontCam` usage everywhere if it is not part of the actual setup

Recommended fix:
```python
self.cameras = {
    name: CameraTable(base, name)
    for name in ["FrontCam", "LeftCam", "RightCam", "BackCam"]
}
```

---

### 1.5 Inconsistent Failure Return Shapes
**Current issue:**
Some helper functions now return `None` instead of a tuple like `(None, None)`, but callers still expect tuple unpacking.

Examples:
- `tag_pose_world()`
- `camera_pose_world_from_tag()`
- `camera_to_robot_world()`
- `fuse_camera_pose_multitag()`

**Required change:**
Make return shapes consistent.

Recommended rule:
- if a function normally returns 2 values, return `(None, None)` on failure
- if a function normally returns 4 values, return `(None, None, None, None)` on failure

Do not return plain `None` unless the caller explicitly checks for plain `None`.

---

## 2. Latency Fix

### 2.1 Capture-to-Publish Latency
**Current issue:**
Older code used `time.time() - pe.timestamp`, which can mix timebases.

**Current status:**
This has been improved to:
```python
self.latency.set((_now() / 1_000_000.0) - pe.timestamp)
```

**Required verification:**
Confirm that:
- `frame_time` from `grabFrame()` is in microseconds
- dividing by `1_000_000.0` gives seconds
- that timebase matches `_now() / 1_000_000.0`

If confirmed, this is acceptable.

**Optional improvement:**
Also publish explicit processing latency measured with Python wall-clock:

```python
processing_start = time.time()
...
processing_end = time.time()
processing_latency = processing_end - processing_start
```

Publish separately as:
- `processing_latency_sec`
or
- `processing_latency_ms`

---

## 3. Standard Deviation Fixes

### 3.1 `std_x` and `std_y`
**Current issue:**
These are heuristics and need to match the coordinate frame being published.

Current code assumes:
- camera-frame uncertainty rotated into world-frame X/Y

That is acceptable only if `robot_X` and `robot_Y` are world/field coordinates.

**Required change if staying with field coordinates:**
Keep current rotation-to-world logic, but include `num_tags` in the denominator.

Recommended update:
```python
depth_std = k_depth * avg_distance * (1 + avg_reproj_error) / max(np.sqrt(num_tags), 1)
lateral_std = k_lat * avg_distance * (1 + avg_reproj_error) / max(np.sqrt(num_tags), 1)
```

### 3.2 `std_yaw`
**Current issue:**
`std_dev_yaw` is heuristic and acceptable as a starting point, but should be treated as tunable.

Current code:
```python
std_dev_yaw = (
    k_yaw * avg_distance
    * (1 + avg_reproj_error) / (np.sqrt(num_tags) * angle_factor)
)
std_dev_yaw /= (yaw_confidence + 1e-3)
```

**Recommended action:**
Keep this for now, but tune on-robot.

---

## 4. Robot Pose / Coordinate Meaning

### 4.1 Current Meaning
Right now:
- `robot_X`
- `robot_Y`
- `robot_Z`
- `robot_yaw`

represent the final estimated robot pose.

### 4.2 Planned Change
If these will later stop being field robot coordinates and instead become camera-relative values, the names must change.

Recommended rename options:
- `target_x`, `target_y`, `target_z`
- `camera_x`, `camera_y`, `camera_z`

Do not keep the `robot_*` names if the values are no longer robot field pose.

---

## 5. Euler Rotation Requirement For Per-Tag Path

### 5.1 Current Status
The current code still uses OpenCV `rvec` internally.
A helper exists:
```python
def rvecToEulerAngles(rvec):
```

but it is not yet part of the published per-tag contract.

### 5.2 Required change for per-tag mode
If the per-tag NT contract is used, the published rotation must be explicit.

Recommended per-tag rotation contract:
- publish Euler angles in radians
- define them as:
  - `rx = roll`
  - `ry = pitch`
  - `rz = yaw`

Do not publish raw Rodrigues `rvec` unless Java is explicitly written to decode Rodrigues vectors.

---

## 6. Per-Tag Contract Additions

If the pipeline moves back to the per-tag path, the following must be added or restored.

### Required per-camera keys
- `connected`
- `heartbeat`
- `valid`
- `timestamp`
- `tx`
- `ty`
- `tag_ids`
- `tag_count`
- raw `tags` array

### Recommended `tags` array layout
```text
[id, tx, ty, tz, rx, ry, rz, error, ambiguity]
```

Where:
- rotation is Euler radians
- transform direction is `cameraToTag`

### Important
The transform direction must be explicitly defined and must remain consistent:
- `cameraToTag`
not
- `tagToCamera`

---

## 7. Current NT Layout Cleanup

### 7.1 Naming Consistency
Current names mix styles:
- `robot_X`
- `robot_yaw`
- `thetax`
- `avg_distance`

**Recommended cleanup:**
Use one consistent naming style, preferably snake_case:
- `robot_x`
- `robot_y`
- `robot_z`
- `robot_yaw`
- `tx`
- `ty`
- `avg_reproj_error`
- `latency_sec` or `latency_ms`

### 7.2 Keep These Fields
Useful current fields:
- `connected`
- `heartbeat`
- `valid`
- `timestamp`
- `latency`
- `tag_ids`
- `tag_count`
- `avg_distance`
- `ambiguity`
- `std_x`
- `std_y`
- `std_yaw`

---

## 8. Behavior When No Pose Exists

### Current issue
If `pe is None`, `publish()` only sets:
```python
self.valid.set(False)
```

### Required decisions
Define what should happen to:
- `connected`
- stale previous pose values
- heartbeat

Recommended behavior:
- `connected` = whether the camera thread/pipeline is alive
- `valid` = whether this frame produced a usable estimate
- heartbeat may continue if the process is alive
- old pose values may remain, but robot-side code must ignore them when `valid == False`

---

## 9. Recommended Priority Order

1. Fix ambiguity formula
2. Restore rejection gate
3. Make tag-count source of truth consistent
4. Fix failure return shapes
5. Confirm `_now()` latency timebase is correct
6. Add `FrontCam` to the table map if used
7. Improve `std_x/std_y/std_yaw` formulas
8. Rename fields if robot pose fields become camera-relative later
9. Reintroduce per-tag `tags` output if returning to the per-tag architecture
