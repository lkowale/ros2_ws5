# Diagnostic report from component to PauseManager
# Components publish this when they detect issues or when issues clear

builtin_interfaces/Time stamp

# Component identification
string component_name      # e.g., "drive", "planter_service", "lift_service"
string component_id        # Instance ID if multiple (e.g., "planter_1")

# Status indicators
uint8 status               # Current component status
uint8 STATUS_OK = 0
uint8 STATUS_WARN = 1
uint8 STATUS_ERROR = 2
uint8 STATUS_STALE = 3

# Severity level for pause decision
uint8 severity             # Recommended severity if error (1-4)
uint8 SEVERITY_CRITICAL = 1  # Stop all motors immediately
uint8 SEVERITY_HIGH = 2      # Pause, wait for operator
uint8 SEVERITY_MEDIUM = 3    # Pause, auto-resume when cleared
uint8 SEVERITY_LOW = 4       # Report only, no pause

# Condition details
string condition_id        # Unique ID for this condition type (e.g., "mqtt_disconnected")
string message             # Human-readable description
string[] details           # Additional key=value details

# State flags
bool condition_active      # True if condition is currently present
bool auto_clearable        # True if condition can auto-clear (for MEDIUM severity)
