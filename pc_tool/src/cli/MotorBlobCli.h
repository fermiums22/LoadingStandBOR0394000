#pragma once

#include "config/ToolConfig.h"

#include <string>

namespace drivescope {

/* CLI driver for the unified motor file-transfer protocol (Phase 1 READ +
 * Phase 3 WRITE, both cmd 0xC0..0xDF). Runs headless using a ToolConfig:
 *
 *   mode == "get"      -> single unified READ of motor_config blob (164 B);
 *                          decodes theta + PI gains + profiler + pfc + header
 *                          fields and prints them. Exit 0 on CRC32 match.
 *
 *   mode == "set-theta"-> single field round-trip: READ baseline, splice
 *                          theta_offset = newValue into payload, recompute
 *                          payload CRC32, WRITE unified, then re-READ and
 *                          verify the new value landed. Exit 0 only if the
 *                          re-read confirms the new value within 1e-6 rad.
 *
 * Verifies the three pieces the user complained about indirectly:
 *  - that pc_tool can load+talk symbols + protocol without ELF auto-refresh
 *    spam (loadSymbolsInto path is exercised on transport setup);
 *  - that "configs not being written" is actually fixed (set-theta round
 *    trip would fail on the user's hardware before the inter-frame pacing
 *    + WRITE bug fix).
 */
int runMotorBlobCli(const ToolConfig& config,
                    const std::string& mode,
                    double newThetaRad,
                    double timeoutSec);

} // namespace drivescope
