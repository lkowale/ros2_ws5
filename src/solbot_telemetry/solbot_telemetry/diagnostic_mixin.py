#!/usr/bin/env python3
"""
DiagnosticMixin - Reusable diagnostic reporting for components

Provides a mixin class that adds diagnostic reporting capabilities to any ROS2 node.
Components use this to report conditions to the PauseManager.

Usage:
    class MyNode(Node, DiagnosticMixin):
        def __init__(self):
            super().__init__('my_node')
            self.setup_diagnostics('my_component')

        def some_error_handler(self):
            self.report_condition(
                'error_id',
                self.SEVERITY_MEDIUM,
                'Something went wrong',
                active=True,
                auto_clearable=True
            )

        def error_cleared(self):
            self.report_ok('error_id')
"""

from solbot5_msgs.msg import DiagnosticReport


class DiagnosticMixin:
    """Mixin providing diagnostic reporting capabilities to ROS2 nodes."""

    # Severity constants matching DiagnosticReport.msg
    SEVERITY_CRITICAL = 1  # Stop all motors immediately
    SEVERITY_HIGH = 2      # Pause, wait for operator decision
    SEVERITY_MEDIUM = 3    # Pause, auto-resume when cleared
    SEVERITY_LOW = 4       # Report only, no pause

    # Status constants
    STATUS_OK = 0
    STATUS_WARN = 1
    STATUS_ERROR = 2
    STATUS_STALE = 3

    def setup_diagnostics(self, component_name: str, component_id: str = ""):
        """
        Initialize diagnostic publisher.

        Args:
            component_name: Name of the component (e.g., "drive", "planter_service")
            component_id: Optional instance ID for multiple instances
        """
        self._diag_component_name = component_name
        self._diag_component_id = component_id or component_name
        self._diag_pub = self.create_publisher(
            DiagnosticReport, '/diagnostic_report', 10)
        self._active_conditions = set()

    def report_condition(self, condition_id: str, severity: int,
                         message: str, active: bool = True,
                         auto_clearable: bool = True,
                         details: list = None):
        """
        Report a diagnostic condition to PauseManager.

        Args:
            condition_id: Unique ID for this condition type (e.g., "mqtt_disconnected")
            severity: Severity level (use SEVERITY_* constants)
            message: Human-readable description
            active: True if condition is currently present, False if cleared
            auto_clearable: True if condition can auto-clear (for MEDIUM severity)
            details: Optional list of key=value detail strings
        """
        msg = DiagnosticReport()
        msg.stamp = self.get_clock().now().to_msg()
        msg.component_name = self._diag_component_name
        msg.component_id = self._diag_component_id
        msg.severity = severity
        msg.condition_id = condition_id
        msg.message = message
        msg.condition_active = active
        msg.auto_clearable = auto_clearable
        msg.details = details or []

        if active:
            msg.status = self.STATUS_ERROR if severity <= 3 else self.STATUS_WARN
            self._active_conditions.add(condition_id)
        else:
            msg.status = self.STATUS_OK
            self._active_conditions.discard(condition_id)

        self._diag_pub.publish(msg)

    def report_ok(self, condition_id: str, message: str = "Condition cleared"):
        """
        Report that a previously active condition is now OK.

        Args:
            condition_id: The condition ID that was previously reported
            message: Optional message describing the resolution
        """
        if condition_id in self._active_conditions:
            self.report_condition(
                condition_id,
                self.SEVERITY_LOW,  # Severity doesn't matter for cleared conditions
                message,
                active=False
            )

    def has_active_conditions(self) -> bool:
        """Check if there are any active conditions."""
        return len(self._active_conditions) > 0

    def get_active_conditions(self) -> set:
        """Get the set of active condition IDs."""
        return self._active_conditions.copy()
