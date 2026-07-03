"""Scene awareness + short-term retention (must-PASS — verified live 2026-07-03)."""
import rospy

from sweetie_bot_behavior_synth import behavior_test, check, person, pony, scene


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5), pony(id=201, bearing=+35.0, dist=1.2))
def test_sees_pony_when_visible(world):
    world.wait_seen("pony")
    t = world.say_and_wait("Look around - what do you see near you right now?")
    check.mentions(t.text, ["pony", "she"], forbid_negated="see")


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5), pony(id=201, bearing=+35.0, dist=1.2))
def test_hidden_pony_recalled_with_direction(world):
    """Vanished object recalled with its remembered direction (retention + select_salient fix)."""
    world.wait_seen("pony")
    rospy.sleep(2.0)                    # let the retention buffer capture a stable position
    world.vanish(201)
    rospy.sleep(1.5)                    # out of frame, inside the 20 s TTL
    t = world.say_and_wait("Where did the pony go?")
    check.mentions(t.text, ["pony", "she", "her"])
    check.direction(t.text, bearing=world.truth(201).bearing)


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5), pony(id=201, bearing=-40.0, dist=1.2))
def test_hidden_behind_inference(world):
    """Pony vanishing at a bearing aligned with a closer visible person -> 'hidden behind'."""
    world.wait_seen("pony")
    rospy.sleep(2.0)
    # move the person onto the pony's bearing, closer, then hide the pony behind them
    world.move(101, bearing=-38.0, dist=0.9)
    rospy.sleep(1.0)
    world.vanish(201)
    rospy.sleep(1.5)
    t = world.say_and_wait("Where is the pony now?")
    check.mentions(t.text, ["behind", "hiding", "hidden"])


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5), pony(id=201, bearing=+30.0, dist=1.2))
def test_pony_reappears_at_detections_seam(world):
    """Scene-level re-acquisition: vanish, then reappear with a NEW id -> she sees her again.
    NOTE: the live 'lost once = lost forever' bug sits BELOW this seam (vision pipeline);
    its faithful must-fail reproduction lives in the vision-chain mode (test_occlusion/tracking
    fuser tests). This test pins the SOAR+scene half."""
    world.wait_seen("pony")
    world.vanish(201)
    rospy.sleep(3.0)
    world.spawn(pony(id=305, bearing=-20.0, dist=1.0))
    rospy.sleep(2.0)
    t = world.say_and_wait("Can you see the pony?")
    # mechanism-first: the NEW pony must be in the rendered scene she answered from
    assert world.col["agent_log"].wait_grep(r"scene_block: .*pony \(id 305\)", timeout=5.0), \
        "re-appeared pony (id 305) never rendered into her scene"
    check.mentions(t.text, ["pony", "she", "her"])
