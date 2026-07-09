# Pause/unpause command for behavior tree execution
# When paused=true, execution should pause
# When paused=false, execution should resume

bool paused
string source      # Source of the pause command (e.g., "operator", "safety_system", "low_battery")
string reason      # Human-readable reason for the pause

# Severity level (0 = not paused, 1-4 = severity)
uint8 severity
uint8 SEVERITY_NONE = 0      # Not paused / all clear
uint8 SEVERITY_CRITICAL = 1  # Stop all motors immediately
uint8 SEVERITY_HIGH = 2      # Pause, wait for operator decision
uint8 SEVERITY_MEDIUM = 3    # Pause, auto-resume when cleared
uint8 SEVERITY_LOW = 4       # Report only, no pause
