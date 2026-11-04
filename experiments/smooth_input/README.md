# Native input curve check

Build `check.cpp` with C++17 and the `navigation` include directory. It exercises startup, explicitly normal mathematical braking, changing targets and200000 fixed-seed updates over varied tick durations. It checks magnitude, bounded input derivatives and bounded changes of the derivative. The production `SpeedResponse` wrapper additionally keeps unclassified zero, loss of ownership and original source expiry as immediate resets. See `docs/SMOOTH_INPUTS.md` for the remaining normal-stop-source integration gap.

These tests contain no device, motor, robot socket, ROS or external service operations.
