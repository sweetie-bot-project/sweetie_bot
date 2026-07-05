"""Camera occlusion — full consumer chain, driven by the REAL image trigger.

Earlier this scenario injected a fake near-depth map (depth_stub:value=0.08) and sat in the
UNKNOWN group: the depth trigger is physically unreachable (monocular depth predicts FAR on a
covered lens), so that test proved nothing about the live path. It now drives a covered camera
(flat gray frame via VisionCluster(covered=True)) so the fuser's image-based OcclusionMonitor
fires exactly as it does on a real covered lens.

Chain under test:
  covered frame -> fuser OcclusionMonitor (image trigger) -> camera_occluded on /detections
  -> scene WARNING banner in the LLM prompt (agent scene_block log)
  -> she complains about the covered camera AND the reply emotion is forced to anger
     (SOAR maps emotion=anger -> evil_look eyes).

Still @unknown until run green on the full sim: the fuser/agent halves are unit-tested (vision
tests/test_occlusion.py; ai_core test_scene.py), but the LLM complaint wording + SOAR emotion
mapping need the live model + SOAR stack.
"""
import pytest

from sweetie_bot_behavior_synth import behavior_test, check, person, scene


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
@pytest.mark.unknown
def test_occlusion_chain_from_covered_camera(world):
    from sweetie_bot_behavior_synth.vision_mode import VisionCluster

    with VisionCluster(covered=True):
        # 1) the fuser must emit camera_occluded on /detections from the covered frame
        got = world.col["detections"].wait_for(
            lambda m: any(d.type == "camera_occluded" for d in m.detections), timeout=25.0)
        assert got is not None, "fuser never emitted camera_occluded from the covered frame"

        # 2) the scene block must carry the WARNING banner (agent observability line)
        # 3) she must complain about the covered camera when spoken to
        # 4) and the reply emotion must be anger (drives the evil_look eyes via SOAR)
        t = world.say_and_wait("Hey Sweetie, what do you see?")
        assert world.col["agent_log"].wait_grep("scene_block: WARNING", timeout=10.0), \
            "camera_occluded never rendered as the scene WARNING banner"
        check.mentions(t.text, ["camera", "blocked", "cover", "lens", "see", "face", "hand"])
        assert t.emotion == "anger", f"camera occlusion must trigger anger, got {t.emotion!r}"
