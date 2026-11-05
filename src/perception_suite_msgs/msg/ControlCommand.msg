# One controller command, as 7_controller_node publishes it on /command.
#
# The ranges match offroad_autonomy.types.ControlCommand exactly, because the
# node is a thin wrapper around the same controller the simulator drives. A
# consumer that applies these to a vehicle is responsible for mapping them onto
# its own actuator units; nothing here is in newtons, degrees or per cent.

std_msgs/Header header

# -1.0 is full left, 1.0 is full right. Already clipped by the controller.
float32 steering

# 0.0 to 1.0.
float32 throttle

# 0.0 to 1.0. Service brake, not the parking brake.
float32 brake

# 0.0 to 1.0. Held on at a standstill so the vehicle does not creep.
float32 parkingbrake

# Why this command looks the way it does, empty for an ordinary steering
# solution. Carries the planner's fallback reason through so an operator can
# tell a deliberate stop from a dead pipeline. An empty /path topic and a
# stopping command are indistinguishable without it.
string reason
