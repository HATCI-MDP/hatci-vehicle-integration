# What 5_planner_node produces and 7_controller_node consumes.
#
# This is deliberately NOT nav_msgs/Path. The /path topic carries the centreline
# already projected to ground metres with no range limit, which is the right
# shape for a consumer that wants a path. The controller is not that consumer:
# it runs its own image_to_ground with its own range limit, and then keeps using
# the pixel values afterwards, so it needs the centreline in IMAGE PIXELS.
# Projecting and unprojecting across the process boundary would not round trip.
#
# Everything the controller needs rides in ONE message rather than several
# topics joined by a synchroniser. The centreline and the mask must come from
# the same frame, and message_filters would add a second queue per topic on top
# of the DDS one, which is latency the control loop pays every frame.

std_msgs/Header header

# Centreline in image pixels: u is the column, v the row. Same length, and
# empty means the planner produced no path, which the controller reads as stop.
float32[] centerline_u
float32[] centerline_v

# The connected component the planner selected as the road, row major, one byte
# per pixel, 0 or 255. The controller measures the trail edges along the arc the
# vehicle is actually driving, which needs the mask and not just the centreline.
# This is NOT the same as the /stabilized_mask topic: that is every road pixel,
# this is only the component the planner committed to.
uint32 mask_width
uint32 mask_height
uint8[] planner_mask

# Speed limit scale from the perception gate, 1.0 for full confidence and 0.0
# for stop.
float32 speed_scale

# Narrowest point ahead, in metres. Infinity when the planner found no
# constriction, which serialises as inf and is handled by the controller.
float32 min_clearance_m

# True when the planner could not fit a centreline and fell back. The reason is
# carried through to the command so an operator sees why the vehicle slowed.
bool fallback_active
string fallback_reason

# Top row of the region of interest, needed to map mask rows back to the full
# frame.
int32 roi_top
