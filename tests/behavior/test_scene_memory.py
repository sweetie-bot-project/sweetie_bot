"""Scene awareness + short-term retention (must-PASS — verified live 2026-07-03)."""
import rospy

from sweetie_bot_behavior_synth import behavior_test, check, entity, person, pony, scene


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
@scene(person(id=3, bearing=0.0, dist=1.0, elevation=+30.0,
              attention_state="DRAWN", gaze_pitch="6.9", gaze_yaw="3.7"))
def test_hidden_pony_recalled_under_live_noise(world):
    """LIVE-faithful recall: the clean sibling test passes while the robot fails ("answers
    something not related" — user, 2026-07-06). Reproduces the real perception signature from
    the 2026-07-04 22:59 llm_agent log: the plushie is TWO entities ('pony' body + 'pony_face')
    with junk attrs (held_by, depth_source), flapping appeared/left as the detector flickers,
    everything held "well above" the small robot; the question arrives STT-mangled (verbatim
    live phrasing, 2026-07-03 00:25). Expected behavior (user spec): she states WHERE the pony
    went relative to her own ego — on-topic, correct side, no raw telemetry parroted."""
    world.wait_seen("human")
    world.spawn(entity("pony", id=200004, bearing=-30.0, dist=1.0, elevation=+25.0,
                       held_by="3", depth_source="measured"),
                entity("pony_face", id=200001, bearing=-28.0, dist=1.0, elevation=+25.0,
                       depth_source="measured"))
    rospy.sleep(2.0)                    # retention captures a stable position
    # detector flicker: the face drops out and re-enters with a fresh tracker id (live ids
    # marched 200001 -> 200004 -> ...), churning arrived/left events in the scene block
    world.vanish(200001)
    rospy.sleep(0.8)
    world.spawn(entity("pony_face", id=200007, bearing=-27.0, dist=1.0, elevation=+25.0))
    rospy.sleep(0.8)
    world.vanish(200007)
    rospy.sleep(0.8)
    world.spawn(entity("pony_face", id=200012, bearing=-29.0, dist=1.0, elevation=+25.0))
    rospy.sleep(1.0)
    # now the plushie is hidden for real - every pony entity goes out of frame
    world.vanish(200004, 200012)
    rospy.sleep(1.5)
    world.col["agent_log"].anchor()     # mechanism assert must see the ASK-time block only
    t = world.say_and_wait("Okay, where did the pony went?")
    # mechanism-first: a remembered pony must be in the scene she answered from
    assert world.col["agent_log"].wait_grep(r"scene_block: .*Recently seen.*pony", timeout=5.0), \
        "no remembered pony rendered into the scene she answered from"
    # on-topic + the remembered side, stated from her ego
    check.mentions(t.text, ["pony", "she", "her"])
    check.direction(t.text, bearing=-30.0)
    # no raw perception telemetry parroted back
    low = t.text.lower()
    for banned in ("held_by", "depth_source", "id 200"):
        assert banned not in low, f"raw telemetry leaked into speech: {banned!r} in {t.text!r}"


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5), pony(id=201, bearing=+35.0, dist=1.2))
def test_hidden_pony_recalled_after_conversation_pause(world):
    """Recall must survive a REAL hide-and-ask pace. Live (2026-07-02/03) the asks trailed the
    hide by well over 20 s ("Where did the pony go?" ... "I want to tell me, where did she
    went?" a minute apart) and she had already forgotten — the old retention TTL was 20 s,
    vision-scale, not conversation-scale. Pins the 90 s TTL."""
    world.wait_seen("pony")
    rospy.sleep(2.0)
    world.vanish(201)
    rospy.sleep(35.0)                   # dead air: past the old 20 s TTL, well inside 90 s
    # re-anchor so the mechanism assert can only match the ASK-time scene_block — earlier
    # dead-air blocks still contained the memory and let a 50/50 direction guess false-pass
    # (exactly how the 20 s TTL regression slipped past this test's first version)
    world.col["agent_log"].anchor()
    t = world.say_and_wait("Where did the pony go?")
    assert world.col["agent_log"].wait_grep(r"scene_block: .*Recently seen.*pony", timeout=5.0), \
        "pony memory expired during a conversational pause - retention TTL too short"
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
