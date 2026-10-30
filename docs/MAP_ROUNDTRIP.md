# Preserve unknown map cells

Nav2 Humble's trinary saver writes unknown cells as PGM gray 205, free as 254,
and occupied as 0. Its loader thresholds occupancy `1 - gray / 255`.
The former workflow requested `free_thresh: 0.25`: the unknown pixel's occupancy
is approximately 0.196078, so it became **free** on reload. This was reproduced
on ARM with the real map server. A successful save service or goal planner did
not establish that unknown cells survived a round trip.

The workbench, benchmark and standalone launch now use `free_thresh: 0.196`,
retain `occupied_thresh: 0.65`, and reject unsafe legacy metadata before stopping
the current workflow or launching navigation. The guard accepts this workflow's
standard non-negated trinary convention; other encodings require an explicit
import/validation step. It does not edit maps, clear obstacles or shrink the
robot envelope.

For a legacy map, retain the original directory and inspect its image encoding.
If it is the standard 0/205/254 palette, copy it to a new directory, change only
the copy's free threshold to 0.196, and reload. Compare all reloaded grid cells
against the vertically flipped PGM palette, not just the total map dimensions.
Keep the original image/pose-graph hashes and the old failure record. Do not
apply this procedure blindly to a different image encoding.

The unit regression covers the actual Humble palette and inequalities,
safe/unsafe metadata and rejection without rewriting the source file.
Real ARM saved-map reload validation is recorded separately; this repair does
not establish real gait execution, sensor calibration or obstacle-stop acceptance.

Primary implementation: [Nav2 Humble map I/O](https://github.com/ros-navigation/navigation2/blob/humble/nav2_map_server/src/map_io.cpp).
