// Host self-test for stream/StreamWatchState (stream-recovery FSM).
//
//   g++ -std=c++20 -Isrc tests/stream_watch_selftest.cpp src/stream/StreamWatchState.cpp -o /tmp/sw && /tmp/sw
//
// Drives the lifecycle a frozen live-plot exercises: subscribe -> stale ->
// resync (status-then-add) -> escalating backoff -> failed_no_motor_response ->
// recovery, plus the motor-rejected-ADD branch.
#include "stream/StreamWatchState.h"

#include <cstdio>

using namespace drivescope;
using S = StreamWatchState;
using A = StreamWatchAction;

static int g_fail = 0;
static void check(const char* name, bool ok)
{
    std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_fail;
}

int main()
{
    StreamWatchConfig cfg;  // defaults: stale 1.0s, addTimeout 0.5s, backoff 2..8s, max 4
    StreamWatch w(cfg);

    // 1) Idle -> first tick asks for a subscription.
    check("initial tick = SendAdd", w.tick(0.0) == A::SendAdd && w.state() == S::AddPending);
    w.onAddSent(0.0);

    // 2) ADD_RSP accepted -> Subscribed.
    w.onAddRsp(0.01, true);
    check("add_rsp(ok) -> subscribed", w.state() == S::Subscribed);

    // 3) Values keep it subscribed; tick is a no-op.
    w.onValue(0.1);
    check("value keeps subscribed", w.tick(0.5) == A::None && w.state() == S::Subscribed);

    // 4) No value for > staleSec -> Stale, then tick = SendStatusThenAdd.
    check("goes stale", w.tick(1.2) == A::None && w.state() == S::Stale);
    check("stale recovery = SendStatusThenAdd",
          w.tick(1.2) == A::SendStatusThenAdd && w.state() == S::AddPending);
    w.onAddSent(1.2);

    // 5) Escalating backoff: each unanswered ADD increments the counter and the
    //    next attempt is spaced further out; eventually -> failed.
    //    add@1.2 -> timeout@1.7 (unanswered=1, resyncing)
    check("add#1 timeout -> resyncing", w.tick(1.75) == A::None && w.state() == S::Resyncing &&
                                         w.unansweredAdds() == 1);
    check("backoff: no resend before 2s", w.tick(2.5) == A::None);   // 1.75+2.0=3.75
    check("resend after backoff", w.tick(3.8) == A::SendAdd && w.state() == S::AddPending);
    w.onAddSent(3.8);
    check("add#2 timeout", w.tick(4.4) == A::None && w.unansweredAdds() == 2);   // resyncing
    // backoff now 4.0s (2.0*2): resend ~8.4
    check("resend#3 after 4s backoff", w.tick(8.5) == A::SendAdd);
    w.onAddSent(8.5);
    check("add#3 timeout", w.tick(9.1) == A::None && w.unansweredAdds() == 3);
    // backoff 6.0s: resend ~15.1
    check("resend#4 after 6s backoff", w.tick(15.2) == A::SendAdd);
    w.onAddSent(15.2);
    check("add#4 timeout -> FAILED",
          w.tick(15.8) == A::None && w.state() == S::FailedNoMotorResponse &&
          w.unansweredAdds() == 4);

    // 6) Failed: no spam — silent until the backoff-cap probe.
    check("failed: silent before probe", w.tick(20.0) == A::None &&
                                         w.state() == S::FailedNoMotorResponse);
    check("failed: slow probe at cap", w.tick(24.0) == A::SendAdd &&
                                       w.state() == S::FailedNoMotorResponse);

    // 7) Recovery: a real VALUE arrives -> back to subscribed, counter reset.
    w.onValue(24.5);
    check("value recovers from failed", w.state() == S::Subscribed && w.unansweredAdds() == 0);

    // 8) Motor-rejected ADD (TABLE_FULL/BAD_*): motor is alive -> resyncing, not failed.
    StreamWatch w2(cfg);
    w2.tick(0.0); w2.onAddSent(0.0);
    w2.onAddRsp(0.02, false);
    check("add_rsp(reject) -> resyncing, not failed",
          w2.state() == S::Resyncing && w2.unansweredAdds() == 0);

    std::printf("%s\n", g_fail ? "RESULT: FAIL" : "RESULT: PASS");
    return g_fail ? 1 : 0;
}
