"""Person leave/re-enter robustness.

Split (faithfulness ledger):
  * churn survival + re-acquisition: must-PASS (devel TF-guarded attention + fixes);
  * conversation AFTER churn: reproduces the LIVE P19 wedge (talk machinery holding a stale
    interlocutor after the track id changes) - documented must-FAIL until the SOAR-side
    stale-interlocutor retirement is implemented (consolidated doc P19 proper fix).
"""
import pytest
import rospy
import rosnode

from sweetie_bot_behavior_synth import behavior_test, person, scene


def _churn(world, ids):
    world.wait_seen("human", timeout=10)
    for new_id in ids:
        current = list(world.detections.present())
        world.vanish(*current)
        rospy.sleep(4.0)                          # SWM visibility timeout
        world.spawn(person(id=new_id, bearing=(-1) ** new_id * 10.0, dist=1.4))
        rospy.sleep(6.0)


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_leave_reenter_id_churn_survival(world):
    """SOAR survives repeated id churn and re-acquires the newest person."""
    ids = [111, 122, 133]
    _churn(world, ids)
    assert "/soar" in rosnode.get_node_names(), "SOAR died during id churn"
    hits = world.col["soar_log"].wait_grep(rf"human_{ids[-1]}", timeout=12.0)
    assert hits, f"attention never registered the re-entered person human_{ids[-1]}"


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
@pytest.mark.xfail(reason="P19 (live-reproduced): the talk machinery can hold a stale "
                          "interlocutor after id churn - conversation resumes only when the "
                          "rebind race is won; SOAR-side stale-object retirement is the fix",
                   strict=False)
def test_conversation_resumes_after_id_churn(world):
    ids = [211, 222]
    _churn(world, ids)
    t = world.say_and_wait("Hello again! Can you hear me?")
    assert t.said is not None, "no voiced reply after churn (P19 wedge)"
