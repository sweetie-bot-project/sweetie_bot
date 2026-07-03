"""Camera occlusion — UNKNOWN group (implemented 2026-07-02, never verified live: the depth
chain was broken by a zombie fuser + missing remote token; both irrelevant here — the depth map
is injected at the provider's own seam via depth_stub).

Chain under test: stub near-depth map -> fuser OcclusionMonitor -> camera_occluded on
/detections -> scene WARNING banner (agent scene_block log) -> complaint in the reply.
"""
import pytest
import rospy

from sweetie_bot_behavior_synth import behavior_test, check, person, scene
from sweetie_bot_behavior_synth.dsl import ScenarioSpec


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
@pytest.mark.unknown
def test_occlusion_chain_from_depth_seam(world):
    from sweetie_bot_behavior_synth.vision_mode import VisionCluster
    from sweetie_bot_text_msgs.msg import DetectionArray

    with VisionCluster(providers="depth_stub",
                       provider_params="depth_stub:value=0.08"):
        # 1) the fuser must emit camera_occluded on /detections
        got = world.col["detections"].wait_for(
            lambda m: any(d.type == "camera_occluded" for d in m.detections), timeout=25.0)
        assert got is not None, "fuser never emitted camera_occluded from the near depth map"

        # 2) the scene block must carry the WARNING banner (agent observability line)
        # 3) and she must complain about the covered camera when spoken to
        t = world.say_and_wait("Hey Sweetie, what do you see?")
        assert world.col["agent_log"].wait_grep("scene_block: WARNING", timeout=10.0), \
            "camera_occluded never rendered as the scene WARNING banner"
        check.mentions(t.text, ["camera", "blocked", "cover", "lens", "see"])
