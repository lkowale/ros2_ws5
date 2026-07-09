# Single entry in pause history for audit trail

builtin_interfaces/Time start_time    # When pause condition started
builtin_interfaces/Time end_time      # When resolved (zero if still active)

string source                         # Component that reported the condition
string condition_id                   # Condition identifier
string reason                         # Human-readable reason

uint8 severity                        # Severity level (1-4)
float64 duration_seconds              # Duration in seconds (-1 if still active)

string resolution                     # How it was resolved
# Possible values: "active", "auto_cleared", "operator_resumed", "emergency_stop"
