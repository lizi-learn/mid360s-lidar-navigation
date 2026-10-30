# Runtime evidence and long-lived shared-memory readers

Linux-only, vendor-independent checks. No ROS, SDK, bus, actuator, robot identity or office data is included.

RuntimeReadiness replaces permanent placeholder flags with separately verified installation facts. A root-owned record is bound to the machine ID, driver PID/start time and executable device/inode. A calibration acceptance bit additionally requires the same protected calibration file identity, modification time and length. Missing, replaced, writable or mismatched evidence is rejected. Reading happens in an ordinary worker; control-loop consumers use atomics. If that worker stops evaluating, its result expires after 200 ms.

The record's issue time identifies a static acceptance statement. It is **not a sensor acquisition timestamp** and does not refresh hardware, operator, localization or ground health. Existing live gates remain required. The binary record is a local Linux implementation detail, not a network protocol or independent proof of calibration. Production must seal records only from actual reviewed evidence. This repository does not supply a switch that marks unverified hardware accepted.

ImuSourceIdentity is captured **before** constructing a shared-memory reader. It detects deletion/replacement of the source object even though the old mapping remains readable. A reader should stop and reconnect through its normal supervisor; it must preserve original sample timestamps and make the new reader epoch explicit. The test creates only disposable regular files, including a replacement and symlink case.

Build each test with C++17, -O2 -Wall -Wextra -Werror; add -pthread for test_runtime_readiness.cpp. Run the resulting executables. No robot is contacted.

Tests use the current test process and a temporary acceptance file owned by the test UID; production defaults to root-owned evidence. ARM native compile/check-only and the generic tests passed. A separate read-only live probe distinguished verified driver protection from unverified commissioning; it sent no actuator commands. The updated private gait candidate has not replaced the active gait process.

The actual Nav2/AMCL planar application test also completed obstacle detour and return (7.081 m), speed adjustment, sudden-obstacle STOP/blocked notice, APP cancellation, E-right and stick takeover, with zero envelope collision attempts. These are synthetic planar results, not humanoid dynamics or real autonomous-walking acceptance. Real commissioning and earlier narrow-door/turn-tracking failures remain open.
