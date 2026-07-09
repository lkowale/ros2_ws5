#!/usr/bin/env python3
"""
PauseManager Node - Central Component State Supervision

Manages all pause requests from components with severity-based behavior:
- Level 4 (LOW): Log and notify only, no pause
- Level 3 (MEDIUM): Pause + auto-resume when condition clears
- Level 2 (HIGH): Pause + wait for operator /resume_pause service call
- Level 1 (CRITICAL): Pause + stop all motors immediately

Topics:
  Subscribed:
    /diagnostic_report (DiagnosticReport) - Component diagnostics

  Published:
    /pause (Pause) - Aggregated pause state for BT nodes
    /cmd_vel (Twist) - Zero velocity for emergency stop

Services:
  /get_pause_history (GetPauseHistory) - Query pause history
  /resume_pause (ResumePause) - Operator resume command

Parameters:
  - stale_condition_timeout: Seconds before considering condition stale (default: 5.0)
  - history_max_entries: Maximum history entries to keep (default: 1000)
  - publish_rate: Rate to publish pause state in Hz (default: 10.0)
"""

import json
import os

import rclpy
from rclpy.node import Node
import rclpy.executors
import time
from typing import Dict, List, Set
from dataclasses import dataclass, field

from geometry_msgs.msg import Twist
from solbot5_msgs.msg import DiagnosticReport, Pause, PauseHistoryEntry
from solbot5_msgs.srv import GetPauseHistory, ManageSuppressedConditions, ResumePause


@dataclass
class ActiveCondition:
    """Represents an active pause condition."""
    condition_id: str
    source: str
    reason: str
    severity: int
    auto_clearable: bool
    start_time: float
    last_report_time: float = field(default_factory=time.time)


class PauseManager(Node):
    def __init__(self):
        super().__init__('pause_manager')

        # Parameters
        self.stale_timeout = self.declare_parameter(
            'stale_condition_timeout', 5.0).value
        self.history_max = self.declare_parameter(
            'history_max_entries', 1000).value
        publish_rate = self.declare_parameter(
            'publish_rate', 10.0).value
        self.suppress_file = os.path.expanduser(
            self.declare_parameter('suppress_file',
                '~/.ros/pause_suppressed.json').value)
        self.suppress_config = os.path.expanduser(
            self.declare_parameter('suppress_config',
                '~/ros2_ws4/config/pause_suppress.conf').value)

        # State
        self.active_conditions: Dict[str, ActiveCondition] = {}
        self.history: List[PauseHistoryEntry] = []
        self.emergency_stop_active = False
        self.suppressed_conditions: Set[str] = self._load_suppressed()
        self._load_suppress_config()

        # Subscribers
        self.diagnostic_sub = self.create_subscription(
            DiagnosticReport,
            '/diagnostic_report',
            self.diagnostic_callback,
            10
        )

        # Subscribe to manual pause requests (from mqtt_op or other sources)
        self.manual_pause_sub = self.create_subscription(
            Pause,
            '/pause_request',
            self.manual_pause_callback,
            10
        )

        # Publishers
        self.pause_pub = self.create_publisher(Pause, '/pause', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Services
        self.history_srv = self.create_service(
            GetPauseHistory,
            'get_pause_history',
            self.handle_get_history
        )
        self.resume_srv = self.create_service(
            ResumePause,
            'resume_pause',
            self.handle_resume
        )
        self.suppress_srv = self.create_service(
            ManageSuppressedConditions,
            'manage_suppressed_conditions',
            self.handle_manage_suppressed
        )

        # Timers
        self.publish_timer = self.create_timer(
            1.0 / publish_rate, self.publish_pause_state)
        self.stale_check_timer = self.create_timer(
            1.0, self.check_stale_conditions)

        if self.suppressed_conditions:
            self.get_logger().info(
                f'  suppressed at startup ({len(self.suppressed_conditions)}): '
                + ', '.join(sorted(self.suppressed_conditions)))
        self.get_logger().info('PauseManager started')
        self.get_logger().info(f'  stale_condition_timeout: {self.stale_timeout}s')
        self.get_logger().info(f'  history_max_entries: {self.history_max}')

    def manual_pause_callback(self, msg: Pause):
        """Handle manual pause requests from operators (via mqtt_op or other sources)."""
        condition_id = f"operator/{msg.source}"

        if msg.paused:
            # Create manual pause as HIGH severity (requires operator resume)
            if condition_id not in self.active_conditions:
                condition = ActiveCondition(
                    condition_id=condition_id,
                    source=msg.source,
                    reason=msg.reason or "Manual pause requested",
                    severity=Pause.SEVERITY_HIGH,  # Operator must resume
                    auto_clearable=False,
                    start_time=time.time(),
                    last_report_time=time.time()
                )
                self.active_conditions[condition_id] = condition
                self.get_logger().warn(
                    f"[PAUSE HIGH] Manual pause from {msg.source}: {msg.reason}")
                self.add_history_entry(condition, "active")
        else:
            # Manual unpause - clear the condition
            if condition_id in self.active_conditions:
                self.clear_condition(condition_id, "operator_resumed")
                self.get_logger().info(
                    f"[RESUMED] Manual unpause from {msg.source}")

    def diagnostic_callback(self, msg: DiagnosticReport):
        """Process incoming diagnostic report from a component."""
        # Create unique condition ID from component + condition
        full_condition_id = f"{msg.component_name}/{msg.condition_id}"

        if msg.condition_active:
            self.handle_condition_active(full_condition_id, msg)
        else:
            self.handle_condition_cleared(full_condition_id, msg)

    def _is_suppressed(self, condition_id: str) -> bool:
        """Check if a condition_id matches any suppressed pattern (exact or wildcard *)."""
        for pattern in self.suppressed_conditions:
            if pattern.endswith('*'):
                if condition_id.startswith(pattern[:-1]):
                    return True
            elif pattern == condition_id:
                return True
        return False

    def _load_suppressed(self) -> Set[str]:
        try:
            with open(self.suppress_file, 'r') as f:
                return set(json.load(f))
        except Exception:
            return set()

    def _load_suppress_config(self):
        try:
            with open(self.suppress_config, 'r') as f:
                for raw in f:
                    line = raw.strip()
                    if not line or line.startswith('#'):
                        continue
                    self.suppressed_conditions.add(line)
            self.get_logger().info(f'Loaded suppress config: {self.suppress_config}')
        except FileNotFoundError:
            pass
        except Exception as e:
            self.get_logger().warn(f'Failed to read suppress config: {e}')

    def _save_suppressed(self):
        try:
            os.makedirs(os.path.dirname(self.suppress_file), exist_ok=True)
            with open(self.suppress_file, 'w') as f:
                json.dump(sorted(self.suppressed_conditions), f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Failed to save suppress list: {e}')

    def handle_condition_active(self, condition_id: str, msg: DiagnosticReport):
        """Handle a new or ongoing condition being reported."""

        # Check if condition already exists
        if condition_id in self.active_conditions:
            # Update last report time (keeps condition alive)
            self.active_conditions[condition_id].last_report_time = time.time()
            return

        # Check suppress list — silently drop matching conditions
        if self._is_suppressed(condition_id):
            self.get_logger().debug(f"[SUPPRESSED] {condition_id}")
            return

        # Level 4 (LOW) - log only, don't create condition
        if msg.severity == DiagnosticReport.SEVERITY_LOW:
            self.get_logger().info(
                f"[DIAGNOSTIC] {msg.component_name}: {msg.message}")
            return

        # Create new active condition for severity 1-3
        condition = ActiveCondition(
            condition_id=condition_id,
            source=msg.component_name,
            reason=msg.message,
            severity=msg.severity,
            auto_clearable=msg.auto_clearable,
            start_time=time.time(),
            last_report_time=time.time()
        )
        self.active_conditions[condition_id] = condition

        severity_name = self._severity_name(msg.severity)
        self.get_logger().warn(
            f"[PAUSE {severity_name}] {msg.component_name}: {msg.message}")

        # Level 1 (CRITICAL) - emergency stop
        if msg.severity == DiagnosticReport.SEVERITY_CRITICAL:
            self.emergency_stop(msg.message)

        # Add to history as active
        self.add_history_entry(condition, "active")

    def handle_condition_cleared(self, condition_id: str, msg: DiagnosticReport):
        """Handle condition being cleared by component."""
        if condition_id not in self.active_conditions:
            return

        condition = self.active_conditions[condition_id]

        # Only auto-clear MEDIUM severity conditions that are auto_clearable
        if condition.severity == DiagnosticReport.SEVERITY_MEDIUM and condition.auto_clearable:
            self.clear_condition(condition_id, "auto_cleared")
            self.get_logger().info(
                f"[AUTO-CLEARED] {condition_id}: {msg.message}")

    def check_stale_conditions(self):
        """Check for MEDIUM severity conditions that have gone stale (no reports)."""
        now = time.time()

        for cid, condition in list(self.active_conditions.items()):
            # Only check MEDIUM severity auto-clearable conditions
            if condition.severity != DiagnosticReport.SEVERITY_MEDIUM:
                continue
            if not condition.auto_clearable:
                continue

            elapsed = now - condition.last_report_time
            if elapsed > self.stale_timeout:
                self.get_logger().info(
                    f"[STALE-CLEARED] {cid} - no reports for {elapsed:.1f}s")
                self.clear_condition(cid, "auto_cleared")

    def clear_condition(self, condition_id: str, resolution: str):
        """Clear an active condition and update history."""
        if condition_id not in self.active_conditions:
            return

        condition = self.active_conditions.pop(condition_id)
        now = time.time()

        # Close out any orphaned "active" entries for this condition_id
        for entry in self.history:
            if entry.condition_id == condition_id and entry.resolution == "active":
                entry.resolution = "superseded"
                entry.duration_seconds = now - condition.start_time
                entry.end_time = self._float_to_time_msg(now)

        self.add_history_entry(condition, resolution)

        # If no more critical conditions, clear emergency stop
        if self.emergency_stop_active:
            has_critical = any(
                c.severity == DiagnosticReport.SEVERITY_CRITICAL
                for c in self.active_conditions.values()
            )
            if not has_critical:
                self.emergency_stop_active = False
                self.get_logger().info("[EMERGENCY STOP] Cleared - no critical conditions")

    def add_history_entry(self, condition: ActiveCondition, resolution: str):
        """Add or update a history entry."""
        now = time.time()

        # Create history entry
        entry = PauseHistoryEntry()
        entry.start_time = self._float_to_time_msg(condition.start_time)
        entry.source = condition.source
        entry.condition_id = condition.condition_id
        entry.reason = condition.reason
        entry.severity = condition.severity
        entry.resolution = resolution

        if resolution == "active":
            entry.duration_seconds = -1.0
            entry.end_time = self._float_to_time_msg(0)
        else:
            entry.duration_seconds = now - condition.start_time
            entry.end_time = self._float_to_time_msg(now)

        self.history.append(entry)

        # Trim history if needed
        if len(self.history) > self.history_max:
            self.history = self.history[-self.history_max:]

    def emergency_stop(self, reason: str):
        """Execute emergency stop - send zero velocity."""
        self.emergency_stop_active = True
        self.get_logger().error(f"[EMERGENCY STOP] {reason}")

        # Send zero velocity
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)

    def get_highest_severity(self) -> int:
        """Get highest (lowest number) severity among active conditions."""
        if not self.active_conditions:
            return 0
        return min(c.severity for c in self.active_conditions.values())

    def publish_pause_state(self):
        """Publish aggregated pause state to /pause topic."""
        msg = Pause()

        if self.active_conditions:
            highest = self.get_highest_severity()

            # Find first condition at highest severity for source/reason
            for condition in self.active_conditions.values():
                if condition.severity == highest:
                    msg.paused = True
                    msg.source = condition.source
                    msg.reason = condition.reason
                    msg.severity = highest
                    break

            # If emergency stop, keep sending zero velocity
            if self.emergency_stop_active:
                self.emergency_stop("Maintaining emergency stop")
        else:
            msg.paused = False
            msg.source = "pause_manager"
            msg.reason = "All conditions cleared"
            msg.severity = Pause.SEVERITY_NONE

        self.pause_pub.publish(msg)

    def handle_get_history(self, request, response):
        """Handle get_pause_history service request."""
        # Filter entries
        filtered = []
        for entry in self.history:
            # Apply filters
            if request.active_only and entry.resolution != "active":
                continue
            if request.filter_source and entry.source != request.filter_source:
                continue
            if request.min_severity > 0 and entry.severity > request.min_severity:
                continue
            filtered.append(entry)

        response.total_entries = len(filtered)

        # Apply max_entries limit
        if request.max_entries > 0:
            filtered = filtered[-request.max_entries:]

        response.entries = filtered
        response.success = True
        response.message = f"Retrieved {len(filtered)} entries"
        return response

    def handle_resume(self, request, response):
        """Handle resume_pause service request (for HIGH severity conditions)."""
        resumed = []

        for cid, condition in list(self.active_conditions.items()):
            # Only resume HIGH severity conditions via operator
            if condition.severity != DiagnosticReport.SEVERITY_HIGH:
                continue

            # If specific condition requested, check match
            if request.condition_id and request.condition_id not in cid:
                continue

            self.clear_condition(cid, "operator_resumed")
            resumed.append(cid)
            self.get_logger().info(
                f"[OPERATOR RESUMED] {cid} by {request.operator_id}")

        if resumed:
            response.success = True
            response.message = f"Resumed {len(resumed)} condition(s)"
        else:
            response.success = False
            response.message = "No HIGH severity conditions to resume"

        response.resumed_conditions = resumed
        return response

    def handle_manage_suppressed(self, request, response):
        """Add, remove, or list suppressed condition patterns."""
        action = request.action.strip().lower()
        cid = request.condition_id.strip()

        if action == 'list':
            response.success = True
            response.message = f"{len(self.suppressed_conditions)} suppressed pattern(s)"
        elif action == 'add':
            if not cid:
                response.success = False
                response.message = "condition_id required for add"
                response.suppressed_conditions = sorted(self.suppressed_conditions)
                return response
            self.suppressed_conditions.add(cid)
            self._save_suppressed()
            # Also clear it if currently active
            if cid in self.active_conditions:
                self.clear_condition(cid, "suppressed")
            response.success = True
            response.message = f"Added suppress pattern: {cid}"
            self.get_logger().info(f"[SUPPRESS] Added pattern: {cid}")
        elif action == 'remove':
            if cid in self.suppressed_conditions:
                self.suppressed_conditions.discard(cid)
                self._save_suppressed()
                response.success = True
                response.message = f"Removed suppress pattern: {cid}"
                self.get_logger().info(f"[SUPPRESS] Removed pattern: {cid}")
            else:
                response.success = False
                response.message = f"Pattern not found: {cid}"
        else:
            response.success = False
            response.message = f"Unknown action: {action} (use add/remove/list)"

        response.suppressed_conditions = sorted(self.suppressed_conditions)
        return response

    def _severity_name(self, severity: int) -> str:
        """Get human-readable severity name."""
        names = {
            1: "CRITICAL",
            2: "HIGH",
            3: "MEDIUM",
            4: "LOW"
        }
        return names.get(severity, "UNKNOWN")

    def _float_to_time_msg(self, timestamp: float):
        """Convert float timestamp to builtin_interfaces/Time message."""
        from builtin_interfaces.msg import Time
        msg = Time()
        msg.sec = int(timestamp)
        msg.nanosec = int((timestamp - int(timestamp)) * 1e9)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PauseManager()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
