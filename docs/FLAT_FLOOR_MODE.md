# Explicit flat-floor operation

`P2_NAV_GROUND_POLICY=flat_floor` selects the user-requested flat-floor test assumption. It removes incomplete six-patch near-ground sampling as a motion prerequisite. It does not fill unknown returns or label missing support as observed. The default remains `observed_support`.

The live adapter must still fit the current real cloud successfully. A failed plane fit, stale LiDAR/odometry or current below-plane return in the forward corridor does not permit motion. The same collision monitor, map, footprint, E-switch control, joystick takeover, cancel and native driver protection remain active. A blind region cannot establish that there is no drop; flat-floor mode is not validation for stairs, holes or uneven terrain.

The UI reports the selected assumption and near-field missing samples separately. No additional human confirmation is introduced: one Start click plans and starts the selected goal.

A passive same-stream experiment found that increasing source-timed support history from 0.25 to 0.4 seconds did not recover all six patches (0/220 complete at both windows); this ineffective candidate was not deployed. The flat-floor change is an explicit behavior change, not a claim that the sensor blind area was repaired or physical navigation passed.
