// CalibrationRunGuard.h -- the single-active-run rule for the Calibration tab
// (docs/calibration-autotests-tz.md: "Only one calibration run may be active
// at a time").
//
// Pure domain object: no UI, no transport, no threads. The DriveScope GUI runs
// the calibration state machines on the single ImGui thread, so this guard is a
// plain non-atomic ownership token rather than a lock. Its job is to make the
// "one run at a time" invariant explicit and centralised instead of implied by
// a scatter of *StartPending_ booleans.
//
// Usage (later slices):
//   if (guard.tryAcquire(calib::Test::Theta)) { ... start the run ... }
//   ... while running, guard.owner() == Test::Theta ...
//   guard.release();   // on Done / Failed / Aborted

#pragma once

#include "calibration/CalibrationModel.h"

namespace drivescope {
namespace calib {

class RunGuard {
public:
    // Acquire the single run slot for `test`. Returns false if a (possibly
    // different) run already owns it -- the caller must not start.
    bool tryAcquire(Test test)
    {
        if (active_) return false;
        active_ = true;
        owner_  = test;
        return true;
    }

    // Release the slot. Idempotent; safe to call on every terminal path.
    void release() { active_ = false; }

    bool active() const { return active_; }

    // Owner test id; meaningful only while active() is true.
    Test owner() const { return owner_; }

    // True if `test` currently holds the slot. Lets a panel cheaply ask
    // "am I the active run?" without caring whether some other test is.
    bool ownedBy(Test test) const { return active_ && owner_ == test; }

    // True if a DIFFERENT test holds the slot -- the reason a Start button
    // must be disabled in the panel for `test`.
    bool blockedFor(Test test) const { return active_ && owner_ != test; }

private:
    bool active_ = false;
    Test owner_  = Test::Theta;
};

} // namespace calib
} // namespace drivescope
