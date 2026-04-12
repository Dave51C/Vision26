# CUSTOM_VISION_PER_TAG_FIXES.md
**Required Fixes For Current CustomVision Per-Tag Design**
**Scope: `Vision26.py`, `PiggyVision26.py`, and the `CustomVision` NT contract**

This document covers the remaining fixes for the current per-tag `CustomVision` path. It assumes the system will publish per-tag observations rather than only final robot pose.

---

## 1. Highest Priority Fixes

### 1.1 Stop Storing Tags Globally By ID
**Current issue:**
The current table structure stores tags as:
- `CustomVision/Tag_1`
- `CustomVision/Tag_2`
- etc.

If two cameras see the same tag in the same cycle, one camera can overwrite the other camera's data.

**Required change:**
Store tags per camera, then per tag.

Recommended structure:
- `CustomVision/LeftCam/Tag_1/...`
- `CustomVision/LeftCam/Tag_7/...`
- `CustomVision/RightCam/Tag_1/...`
- `CustomVision/BackCam/Tag_7/...`

This prevents camera collisions and keeps the observation source explicit.

---

### 1.2 Fix Ambiguity Formula
**Current issue:**
The ambiguity metric still behaves opposite to the normal "higher = worse" convention.

Current code:
```python
ambiguity = abs(err1 - err2) / (err1 + err2 + 1e-6)
```

**Required change:**
Use a best-vs-second-best ratio:
```python
errs = sorted(pnperrs)
err1, err2 = errs[0], errs[1]
ambiguity = err1 / (err2 + 1e-6)
```

Interpretation:
- closer to `1.0` = high ambiguity
- closer to `0.0` = low ambiguity

---

### 1.3 Make Failure Return Shapes Consistent
**Current issue:**
Several helper functions return plain `None` even though callers unpack tuples.

Examples:
- `tag_pose_world()`
- `camera_pose_world_from_tag()`
- `camera_to_robot_world()`
- `fuse_camera_pose_multitag()`

**Required change:**
Use consistent failure returns:
- return `(None, None)` for 2-value functions
- return `(None, None, None, None)` for 4-value functions

Do not return plain `None` unless the caller explicitly expects plain `None`.

---

### 1.4 Publish CustomVision Every Camera Cycle
**Current issue:**
`customTable.publish(targets, frame_time, proc_ms)` only runs when `len(results) > 0`.

That means:
- no heartbeat update when no tags are seen
- stale validity state
- `hasTag` can stay stale

**Required change:**
Call the `CustomVision` publish path every camera cycle, even if no tags were detected.

Recommended behavior:
- empty target list -> `hasTag = False`
- heartbeat still increments
- stale tag entries are invalidated correctly

---

### 1.5 Rename `publishTimestamp`
**Current issue:**
The field is called `publishTimestamp`, but the value being written is actually frame capture time.

**Required change:**
Rename it to something accurate:
- `captureTimestamp`
or
- `frameTimestamp`

Do not call it publish time if it is capture time.

---

## 2. Required Layout Changes

### 2.1 Recommended Table Structure

Use:
- `CustomVision/heartbeat`
- `CustomVision/hasTag`
- `CustomVision/<CameraName>/heartbeat`
- `CustomVision/<CameraName>/hasTag`
- `CustomVision/<CameraName>/Tag_<id>/...`

For each tag entry:
- `tagPose`
- `targetId`
- `poseAmbiguity`
- `tagArea`
- `captureTimestamp`
- `processingLatency`
- `networkLatency`
- `totalLatency`
- `lastSeen`
- `valid`
- `camera`

This keeps:
- multiple cameras separate
- per-tag data separate
- root-level health optional

---

### 2.2 Define `tagPose` Clearly
**Required change:**
Document exactly what `tagPose` means.

Recommended definition:
- `tagPose = [x, y, z, roll, pitch, yaw]`
- transform direction = `cameraToTag`
- translation units = whatever the contract currently uses
- angles = radians

Do not leave the transform direction implicit.

---

## 3. Metadata Fixes

### 3.1 Clarify `hasTag`
**Current issue:**
`hasTag` is currently global and can be misleading when multiple cameras are publishing independently.

**Required change:**
Decide whether:
- `CustomVision/hasTag` means "any camera currently sees at least one valid tag"
or
- each camera should have its own `hasTag`

Recommended:
- keep per-camera `hasTag`
- optional root-level `hasTag` = OR of all cameras

---

### 3.2 Clarify `connected`
**Current issue:**
Camera connection and pose validity are not the same thing.

**Required change:**
Define:
- `connected` = camera pipeline alive
- `valid` = this particular tag entry is current and usable

Do not use them interchangeably.

---

### 3.3 Heartbeat Scope
**Current issue:**
Right now heartbeat is only global.

**Required change:**
Consider both:
- root heartbeat for whole custom vision process
- per-camera heartbeat for each camera pipeline

This makes failures easier to diagnose.

---

## 4. Math / Quality Fixes

### 4.1 Re-enable Rejection Logic
**Current issue:**
Bad solves can still flow through if they are only down-weighted, not rejected.

**Required change:**
Add a rejection gate before publishing or trusting tag observations.

Suggested starter checks:
- reject if `avg_reproj_error` too high
- reject if ambiguity too high
- reject if no valid tags remain
- reject impossible positions/orientations if applicable

---

### 4.2 Improve `std_x`, `std_y`, `std_yaw`
**Current issue:**
These are heuristic and should be treated as tunable.

**Required change:**
At minimum:
- include `num_tags` in the denominator
- keep uncertainty larger for farther targets
- make yaw uncertainty worse when angle quality is poor

Recommended direction:
```python
depth_std = k_depth * avg_distance * (1 + avg_reproj_error) / max(np.sqrt(num_tags), 1)
lateral_std = k_lat * avg_distance * (1 + avg_reproj_error) / max(np.sqrt(num_tags), 1)
```

Keep `std_dev_yaw` as a tunable heuristic.

---

### 4.3 Keep Tag Count Consistent
**Current issue:**
`num_tags` and `tag_count` can drift if they are derived differently.

**Required change:**
Use one source of truth.

Recommended:
- `tag_count = num_tags`
or
- remove one of them

---

## 5. Per-Tag Rotation Requirement

### 5.1 Euler Rotation For Java
If Java is expected to read the per-tag pose directly, the rotation should be published in a Java-friendly format.

Recommended:
- publish Euler angles
- radians
- explicitly define:
  - `roll`
  - `pitch`
  - `yaw`

Do not assume Java will decode Rodrigues `rvec` unless that is intentionally implemented.

---

## 6. Recommended Order Of Work

1. Change `CustomVision` to per-camera then per-tag
2. Fix ambiguity formula
3. Fix helper return shapes
4. Publish `CustomVision` every camera cycle
5. Rename `publishTimestamp` to `captureTimestamp`
6. Re-enable rejection logic
7. Clean up `connected` / `valid` / heartbeat semantics
8. Tune standard deviations
9. Finalize Java-side subscribers once the table structure is stable

---

## 7. Practical Goal

After these fixes, the custom per-tag path should provide:
- collision-free per-camera tag observations
- stable health/validity metadata
- correctly interpreted ambiguity
- a clear transform contract for Java
- a usable NT schema for multi-camera robot-side fusion
