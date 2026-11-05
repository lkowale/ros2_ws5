# Toolbar (tool slider) state, published by toolbar_service at 10 Hz.
# Positions in metres from centre, +right (same convention as /tool_bar_position).

builtin_interfaces/Time stamp

# "disabled"  monitor only, commands are not forwarded to the actuator
# "no_sensor" no TBS reading from ESP32 (never received or timed out)
# "estop"     e-stop pressed on the ESP32
# "fault"     latched fault, needs reset_fault (see fault_reason)
# "holding"   auto mode, command stream timed out, holding position
# "moving"    tracking toward target
# "at_target" within tolerance of target
string state
string mode                   # "auto" (follow /tool_bar_position_cmd) or "manual"
string fault_reason

float32 current_position      # measured
float32 raw_position          # measured, firmware units (metres or ADC counts)
float32 target_position       # last target sent to the actuator
float32 firmware_goal         # goal the ESP32 reports (TBS_goal)

float32 min_position          # soft limits enforced by this node
float32 max_position
float32 center_position

# solbot_main: from <dev>/act/state. main_esp32_pio: duty = TPS_pwm/255, rest NaN/"".
float32 duty                  # -1..1
float32 current_a
string firmware_fault         # none | sensor | overcurrent | stall

bool enabled
bool sensor_ok
bool mqtt_connected
bool estop
bool in_motion
bool fault_latched
