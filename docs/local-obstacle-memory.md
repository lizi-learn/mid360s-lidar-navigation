# Planner and controller obstacle memory

The P2 profile uses the same source-timed dynamic obstacle memory in its fixed
global costmap and rolling local costmap. An infinite local history previously
retained isolated hits in angular bins absent from later height-slice scans.
The global planner could then produce a clear path while the local controller
rejected every forward command.

Dynamic hits retain their original ROS source stamp for three seconds. A fresh
source is required before expiration can remove an old dynamic mark. Expiration
restores the underlying static map; it does not invent free floor, clear static
walls, change authored restrictions, or shrink the footprint. Current obstacle
hits refresh their own lifetime. An offline or delayed source cannot authorize
expiration. On unsynchronized MID-360 data, the ROS source stamp remains the
driver's host receive time; this is not a device sampling-time guarantee.

Rolling origins shift observation times by exactly the same realized integer
cell offset as their obstacle array. Newly exposed cells have no remembered hit.
Cell indices alone must never be used as persistent world locations.

`experiments/arrival/rolling_memory.cpp` compares the stock local layer with the
extension using actual Nav2 layers and the P2 controller. It reproduces zero
forward input before the fix, checks forward output after dynamic expiration,
and retains rejection for current obstacles, static walls and unknown space.
It also checks source loss/delay, refreshed hits and rolling origin shifts.
These synthetic checks do not establish physical walking or distance accuracy.

The Agent chat bubble is reconciled with the later SDK task result. Acceptance
does not establish movement or arrival, and a later failure replaces the old
acceptance message. Failed or uncertain tasks are never automatically replayed.
