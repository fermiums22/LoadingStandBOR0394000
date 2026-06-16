// CalibrationModel.cpp -- compile-time contract checks for the calibration
// domain models. There is no runtime code here yet (the four tests' execution
// logic lands in later slices); this TU exists so the header types are
// compiled by the build and so the firmware-mirror enum values are pinned by
// static_assert. If motor FW renumbers a status byte, change BOTH sides -- a
// mismatch is a wire-protocol break, not a cosmetic edit.

#include "calibration/CalibrationModel.h"
#include "calibration/CalibrationRunGuard.h"

namespace drivescope {
namespace calib {

// MTR_ANS_THETA_CAL phase byte (CalibrationTab.cpp phName[] order).
static_assert(static_cast<uint8_t>(ThetaCalPhase::Idle)     == 0);
static_assert(static_cast<uint8_t>(ThetaCalPhase::Running)  == 1);
static_assert(static_cast<uint8_t>(ThetaCalPhase::DoneOk)   == 2);
static_assert(static_cast<uint8_t>(ThetaCalPhase::DoneFail) == 3);

// Theta CFG status byte (old calibThetaStatusName order).
static_assert(static_cast<uint8_t>(ThetaStatus::Ok)           == 0);
static_assert(static_cast<uint8_t>(ThetaStatus::BadArg)       == 1);
static_assert(static_cast<uint8_t>(ThetaStatus::Busy)         == 2);
static_assert(static_cast<uint8_t>(ThetaStatus::NotReady)     == 3);
static_assert(static_cast<uint8_t>(ThetaStatus::StorageEmpty) == 4);
static_assert(static_cast<uint8_t>(ThetaStatus::StorageFail)  == 5);
static_assert(static_cast<uint8_t>(ThetaStatus::Fault)        == 6);
static_assert(static_cast<uint8_t>(ThetaStatus::Running)      == 7);

// Profiler state / test / error bytes (old calibProfiler*Name order).
static_assert(static_cast<uint8_t>(ProfilerState::Aborted) == 4);
static_assert(static_cast<uint8_t>(ProfilerTest::Kz)       == 2);
static_assert(static_cast<uint8_t>(ProfilerError::Range)   == 7);

// Test ids must match the Calibration tab left-rail order.
static_assert(static_cast<uint8_t>(Test::Theta)       == 0);
static_assert(static_cast<uint8_t>(Test::Machine)     == 1);
static_assert(static_cast<uint8_t>(Test::CurrentStep) == 2);
static_assert(static_cast<uint8_t>(Test::SpeedLoop)   == 3);

// The guard must start unowned.
static_assert(sizeof(RunGuard) >= 1);

} // namespace calib
} // namespace drivescope
