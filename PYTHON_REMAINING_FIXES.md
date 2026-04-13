# PYTHON_REMAINING_FIXES.md
**Remaining Python Fixes**
**Scope: `Vision26.py` and `PiggyVision26.py`**

---

## 1. Fix Ambiguity Rejection Direction

The ambiguity formula was changed so that:
- higher ambiguity = worse
- lower ambiguity = better

That means the rejection gate must reject when ambiguity is **greater than** the threshold, not less than it.

Required fix:
```python
if (
    avg_reproj_error is None or
    avg_reproj_error > 5.0 or
    avg_ambiguity > amb_thresh or
    num_tags == 0
):
    return None
```

---

## 2. Use One Source Of Truth For Tag Count

Current issue:
`tag_count` is derived from `len(tag_ids)` even though `num_tags` is already passed in.

Required fix:
```python
self.tag_count = num_tags
```

Do not maintain two competing tag-count definitions.

---

## 3. Decide `connected` Meaning

Current issue:
`connected = True` currently means a valid pose was published.

Decide whether `connected` should mean:
- camera/pipeline thread is alive
or
- a valid estimate exists this cycle

Document and keep it consistent.

---

## 4. Verify Latency Timebase

Current code:
```python
self.latency.set((_now() / 1_000_000.0) - pe.timestamp)
```

This is only correct if:
- `pe.timestamp` and `_now()` use the same timebase

Verify that assumption before trusting the latency value.
