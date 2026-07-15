"""Regression: two co-active sibling subprocesses must NOT self-halt the SOAR kernel.

The live wedge "Goal stack depth exceeded 23 on a no-change impasse" (15+ run dirs since
2026-05; fired live 2026-07-14 for ~2.5 min of silent brain death) had a single mechanism:
scheduling*propose*direct-subprocess / *subprocess proposed their per-subprocess
continue-subprocess operators as bare acceptables. When a parent momentarily holds TWO active
subprocesses -- the real race being the missing-speaker flexbe search still active while
looking-at-human initiates on a visible flicker -- the two continue operators TIE with no
resolution, the tie impasse subgoals with no progress, and the goal stack blows past the
limit in a single decision, halting the kernel. The fix adds the indifferent preference (=)
to both proposers (scheduling.soar).

This test drives the real process/scheduling/resource machinery headlessly: it builds a
parent process and, via one automatic ^add-process operator, TWO active sibling subprocesses
under it -- the minimal faithful shape of the co-activation -- then asserts the kernel does
not halt. It sources the LIVE repo scheduling.soar, so reverting the = fix re-reddens it
(verified: stock halts to run-state INTERRUPTED; fixed runs to STOPPED).

Pure SML, no rospy/ROS master -- classical placement next to the package (not tests/behavior,
which is reserved for sim-backed robot behaviours). Skips cleanly where the SML bindings are
absent.
"""
import os
import sys

import pytest

# SML bindings live outside the default path on the deploy hosts (installed at /opt/soar).
try:
    import Python_sml_ClientInterface as sml
except ImportError:
    if os.path.isdir("/opt/soar"):
        sys.path.insert(0, "/opt/soar")
    try:
        import Python_sml_ClientInterface as sml
    except ImportError:
        sml = None

pytestmark = pytest.mark.skipif(sml is None, reason="Python_sml_ClientInterface (SOAR bindings) not available")

# repo soar tree: this file is behavior/sweetie_bot_soar/tests/, rules live in ../soar/unified/
_UNIFIED = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "soar", "unified"))

# Minimal driver: source the real machinery (self-bootstraps time with no io clock via
# initialization-proto3), then create a parent process 'talkish' and two active sibling
# subprocesses under it through the automatic ^add-process mechanism the real behaviours use.
_DRIVER = """
source {unified}/system/system_source.soar
source {unified}/initialization-proto3.soar
source {unified}/wait.soar

sp {{repro*propose*mk-talkish
    (state <s> ^top-state 1 ^name root ^process <rp> ^beliefs <b>)
    (<rp> ^name root)
   -{{ (<b> ^process <t>) (<t> ^name talkish) }}
-->
    (<s> ^operator <o> +, =)
    (<o> ^name mk-talkish ^type automatic ^add-process <t>)
    (<t> ^name talkish ^substate holder ^subprocess-of <rp>)
}}

sp {{repro*propose*spawn-two
    (state <s> ^substate holder ^process <tp> ^beliefs <b>)
    (<tp> ^name talkish)
   -{{ (<b> ^process <p>) (<p> ^name proc-p1) }}
-->
    (<s> ^operator <o> +, =)
    (<o> ^name spawn-two ^type automatic ^add-process <p1> ^add-process <p2>)
    (<p1> ^name proc-p1 ^substate wait ^subprocess-of <tp>)
    (<p2> ^name proc-p2 ^substate wait ^subprocess-of <tp>)
}}
"""


def _run(driver_text, max_decisions=600):
    prints = []

    def on_print(event_id, user, agent, message):
        prints.append(message)

    kernel = sml.Kernel.CreateKernelInNewThread()
    try:
        agent = kernel.CreateAgent("halt_regression")
        assert not agent.HadError(), agent.GetLastErrorDescription()
        agent.RegisterForPrintEvent(sml.smlEVENT_PRINT, on_print, None)

        import tempfile
        with tempfile.NamedTemporaryFile("w", suffix=".soar", delete=False) as f:
            f.write(driver_text)
            path = f.name
        try:
            agent.LoadProductions(path)
            assert not agent.HadError(), "production load failed: " + agent.GetLastErrorDescription()
            agent.RunSelf(max_decisions)
            return agent.GetRunState(), "".join(prints)
        finally:
            os.unlink(path)
    finally:
        kernel.Shutdown()


def test_coactive_sibling_subprocesses_do_not_halt_the_kernel():
    runstate, trace = _run(_DRIVER.format(unified=_UNIFIED))
    # both siblings must actually have been created (else the scenario proved nothing)
    assert trace.count("INITIATE PROCESS") >= 3, \
        "scenario did not create the parent + two sibling processes:\n" + trace[-2000:]
    assert "System halted" not in trace and "Goal stack depth exceeded" not in trace, \
        "kernel self-halted on two co-active sibling subprocesses (scheduling = fix regressed):\n" \
        + trace[-2000:]
    # HALTED/INTERRUPTED run-state == 3/1; a clean run bottoms out at STOPPED == 0
    assert runstate == sml.sml_RUNSTATE_STOPPED, \
        "agent did not run to a clean stop (run-state %d); likely a halt/impasse" % runstate
