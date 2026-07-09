# A custom robot status message demonstrating rcldart generation:
# nested Header, string, primitives, a bool, and a float64[] sequence.
std_msgs/Header header
string robot_name
uint8 battery_percentage
bool is_charging
float64[] joint_positions
