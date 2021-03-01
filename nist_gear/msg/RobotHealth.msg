# RobotStatus message

# Message informing about the status of both robots.
# This is used to perform task changeover in ariac 2021.
# true: the robot is fully operational and can be used.
# false: the robot should not be used.

# Status of the kitting robot (true=enabled, false=disabled)
bool kitting_robot_enabled
# Status of the assembly robot (true=enabled, false=disabled)
bool assembly_robot_enabled