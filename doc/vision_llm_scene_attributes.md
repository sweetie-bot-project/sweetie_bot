# Vision → LLM: exposing per-entity attributes to the SceneProvider

**Audience:** the vision agent (perfusion / vision_proxy).
**Why:** the LLM agent now has a `SceneProvider` that gives Sweetie ambient awareness of who/what is
around her (people, objects, who's looking/holding/emoting) by reading the vision output. A few of
the newly-added semantic outputs don't reach it yet. This is a small, well-scoped vision-side change.

## How the LLM consumes vision (the boundary)

The LLM side (`sweetie_bot_llm` → `scene_collector.py` → `sweetie_bot_ai_core`) subscribes to the
**`sweetie_bot_text_msgs/DetectionArray`** topic and, per `Detection`, uses:

- `id` (persistent track id — people are described **by id only**, no names/gallery),
- `type` (`person`, `pony_face`, …),
- `pose.position` (→ bearing/zone via TF),
- **`attribute[]` / `value[]`** — the semantic key/value pairs it renders into her prompt
  (e.g. *"a person on your left (id 3) — looking at you, holding a pony"*).

The **ROS `Detection` message is the contract.** The SceneProvider deliberately does **not** import
perfusion's internal schema — it only reads `attribute[]/value[]`. Whatever keys appear there are
rendered automatically (forward-compatible), so **no LLM-side change is needed** once the keys land.

## The gap

`perfusion/ros/mapping.py` → `detection_to_msg_dict()` fills `attribute[]/value[]` **only from the
free-form `det.attributes` dict**:

```python
keys = list(det.attributes.keys())
... "attribute": keys, "value": [str(det.attributes[k]) for k in keys]
```

But the new semantic outputs are **typed schema fields, not entries in `attributes`**:

- `providers/emotion_openvino.py` → fills typed `emotion`, `emotion_scores`
- `providers/gaze_openvino.py` → fills typed `gaze_vector`, `gaze_angles`, `head_pose`
- `runtime/fuse.py` → attributes person↔object **holding** natively (typed / relational)

Nothing copies these into `det.attributes`, so **they never reach `Detection.attribute[]/value[]`,
and the LLM can't see emotion / gaze / holding.**

## What to do (vision side)

Mirror the **distilled** semantic signals into `det.attributes` (short string values) — either in each
provider, in `fuse.py`, or as a small projection step in the gasket before `detection_to_msg_dict`.
Keep it to compact, human-meaningful values; **do not** dump raw vectors/scores (`emotion_scores`,
`gaze_vector`) into attributes — they waste the LLM's token budget and aren't useful to it.

Recommended keys + value conventions (these match the SceneProvider's pretty-renderer; other keys
still render generically as `key: value`):

| attribute key    | value example        | renders as            | source |
|------------------|----------------------|-----------------------|--------|
| `emotion`        | `happy` / `sad` …    | "looks happy"         | emotion_openvino label (argmax of `emotion`) |
| `gaze_at_robot`  | `yes` / `no`         | "looking at you"      | derive from `gaze_angles`/`head_pose` (facing camera ⇒ small yaw/pitch) |
| `holding`        | `pony` / `phone` …   | "holding a pony"      | fuse.py person↔object holding (object `type`) |
| `smiling`        | `yes` / `no`         | "smiling" / "not smiling" | optional, if available |

Value rules the renderer understands:
- boolean-ish values (`yes`/`no`/`true`/`false`/`1`/`0`) — negatives render as *"not …"* or are dropped;
- for `emotion`/`holding` the value string is inserted (`looks {value}` / `holding {value}`);
- unknown keys render plainly as `key: value` — so you can add more later without telling us.

Only attach an attribute when it's **confident and salient** (e.g. omit `gaze_at_robot` when the head
isn't clearly facing her). A wrong attribute is worse than a missing one — she'll narrate it.

## Files to touch (vision)
- `src/perfusion/ros/mapping.py` — `detection_to_msg_dict()` (single choke point if you project
  typed→attributes here), or
- `src/perfusion/providers/{emotion_openvino,gaze_openvino}.py` + `src/perfusion/runtime/fuse.py`
  (set `det.attributes[...]` at the source).

## What the LLM side does NOT need
- **No names / identity** — people are referred to by `id` only; leave `label` empty.
- **No geometry beyond `pose.position`** — the LLM coarsens it to left/right/near/far itself.
- **No 360° concern** — the LLM already drops anything behind her (operators behind the booth);
  just publish detections as usual.

## Verifying the bridge
Once attributes are populated, on proto3 (LLM branch `llm-agent-rework`):
```bash
rostopic echo -n1 <detections_topic>          # confirm attribute[]/value[] carry emotion/holding/gaze
# then run the LLM agent (roslaunch sweetie_bot_llm agent.launch) and call get_scene, or read the
# node's logged raw prompt — the "Around you right now:" block should show the new attributes.
```
The SceneProvider renders them with no further changes on the LLM side.

## Implementation status (vision side) — DONE 2026-07-01

Implemented in perfusion `core/binding.py` (swai `vision-federation-rework` `f83b7e5`), at the source
so `det.attributes` is clean for every consumer (no gasket change; `ros/mapping.py` forwards verbatim):

- `gaze_at_robot`=`yes` when confidently facing (attention_state DRAWN), omitted otherwise.
- `emotion`=label only when salient+confident (skip `neutral` / max-softmax < `SBVISION_EMOTION_MIN_CONF`=0.4);
  `happy` also sets `smiling`=`yes`. Raw scores kept OFF attributes.
- `holding`=object noun with a salience floor (`SBVISION_HOLD_MIN_CONF`=0.25) so weak crowd near-holds are
  dropped; `held_by`=holder id on the object. `holding_id`/`holding_conf` removed from attributes.

Verified e2e on proto3 over req_frames (deployed fuser + gasket): `attribute[]/value[]` carried
emotion/gaze_at_robot/smiling/holding/held_by; the noise keys are gone. Remaining older keys still present
(attention_state, gaze_pitch/yaw, gesture*) render generically and are harmless. Tune emotion/holding
strictness via the SBVISION_* env vars above.
