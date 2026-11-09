#include "actuator.h"

#include <Arduino.h>
#include <math.h>

const char *faultName(ActFault f) {
  switch (f) {
    case ActFault::SENSOR: return "sensor";
    case ActFault::OVERCURRENT: return "overcurrent";
    case ActFault::STALL: return "stall";
    case ActFault::TIMEOUT: return "timeout";
    default: return "none";
  }
}

void ActConfig::toJson(JsonObject o) const {
  o["name"] = name;
  o["enabled"] = enabled;
  o["invert"] = invert;
  o["r_min"] = r_min;
  o["r_max"] = r_max;
  o["u_min"] = u_min;
  o["u_max"] = u_max;
  o["deadband"] = deadband;
  o["slow_zone"] = slow_zone;
  o["pwm_min"] = pwm_min;
  o["pwm_max"] = pwm_max;
  o["pwm_slew"] = pwm_slew;
  o["i_limit"] = i_limit;
  o["oc_ms"] = oc_ms;
  o["stall_delta"] = stall_delta;
  o["stall_ms"] = stall_ms;
  o["cmd_timeout_ms"] = cmd_timeout_ms;
  o["max_drive_ms"] = max_drive_ms;
  o["idle_at_boot"] = idle_at_boot;
  o["sensor_check"] = sensor_check;
}

void ActConfig::fromJson(JsonObjectConst o) {
  if (o.containsKey("name")) strlcpy(name, o["name"] | "", sizeof(name));
  enabled = o["enabled"] | enabled;
  invert = o["invert"] | invert;
  r_min = o["r_min"] | r_min;
  r_max = o["r_max"] | r_max;
  u_min = o["u_min"] | u_min;
  u_max = o["u_max"] | u_max;
  deadband = o["deadband"] | deadband;
  slow_zone = o["slow_zone"] | slow_zone;
  pwm_min = o["pwm_min"] | pwm_min;
  pwm_max = o["pwm_max"] | pwm_max;
  pwm_slew = o["pwm_slew"] | pwm_slew;
  i_limit = o["i_limit"] | i_limit;
  oc_ms = o["oc_ms"] | oc_ms;
  stall_delta = o["stall_delta"] | stall_delta;
  stall_ms = o["stall_ms"] | stall_ms;
  cmd_timeout_ms = o["cmd_timeout_ms"] | cmd_timeout_ms;
  max_drive_ms = o["max_drive_ms"] | max_drive_ms;
  idle_at_boot = o["idle_at_boot"] | idle_at_boot;
  sensor_check = o["sensor_check"] | sensor_check;
  if (r_max < r_min) {
    float t = r_min;
    r_min = r_max;
    r_max = t;
  }
  if (slow_zone <= deadband) slow_zone = deadband * 2 + 0.001f;
}

float Actuator::toUnits(float r) const {
  float span = cfg.r_max - cfg.r_min;
  if (fabsf(span) < 1e-4f) return r;
  return cfg.u_min + (r - cfg.r_min) / span * (cfg.u_max - cfg.u_min);
}

float Actuator::fromUnits(float u) const {
  float span = cfg.u_max - cfg.u_min;
  if (fabsf(span) < 1e-6f) return cfg.r_min;
  return cfg.r_min + (u - cfg.u_min) / span * (cfg.r_max - cfg.r_min);
}

float Actuator::clampGoal(float r) const {
  if (r < cfg.r_min) return cfg.r_min;
  if (r > cfg.r_max) return cfg.r_max;
  return r;
}

void Actuator::setGoalRatio(float r) {
  if (isnan(r)) return;
  float g = clampGoal(r);
  uint32_t now = millis();
  // a new target restarts the max-drive clock; resends of the same goal don't. The stall
  // reference is NOT reset: a jammed actuator still trips however often the goal changes.
  if (driving_ && fabsf(g - st.goal_r) > cfg.stall_delta) driveSinceMs_ = now;
  st.goal_r = g;
  lastCmdMs_ = now;
  goalPending_ = true;
  st.active = true;
  // a fresh command clears latched motion faults (not sensor faults)
  if (st.fault == ActFault::STALL || st.fault == ActFault::OVERCURRENT || st.fault == ActFault::TIMEOUT)
    st.fault = ActFault::NONE;
}

void Actuator::hold() {
  st.goal_r = clampGoal(st.r);
  inDeadband_ = true;
}

void Actuator::resetFault() {
  st.fault = ActFault::NONE;
  hold();
}

void Actuator::updatePosition(float r, uint32_t now_ms) {
  bool first = !st.valid;
  st.r = r;
  st.valid = true;
  if (first) {
    st.goal_r = clampGoal(r);  // boot: hold where we are
    st.active = !cfg.idle_at_boot;
    stallRefR_ = r;
    stallRefMs_ = now_ms;
  }
  if (cfg.sensor_check && (r < 0.01f || r > 0.99f)) st.fault = ActFault::SENSOR;
  else if (st.fault == ActFault::SENSOR) st.fault = ActFault::NONE, hold();
}

void Actuator::updateCurrent(float amps_raw) {
  // IS only reports high-side current during the on-time: compensate for duty
  float d = fabsf(st.duty);
  float est = d > 0.2f ? amps_raw / d : amps_raw / 0.2f;
  if (d < 0.01f) est = amps_raw;
  st.current += 0.3f * (est - st.current);
}

float Actuator::control(float dt, bool allowed, uint32_t now_ms) {
  float target = 0.0f;
  if (!cfg.enabled || !st.valid) {
    st.duty = 0;
    st.moving = false;
    return 0;
  }
  if (!st.active) {  // idle until the first command: no drive, not even holding
    st.goal_r = clampGoal(st.r);
    inDeadband_ = true;
  } else if (!allowed) {
    hold();
  } else if (cfg.cmd_timeout_ms && now_ms - lastCmdMs_ > cfg.cmd_timeout_ms && goalPending_) {
    goalPending_ = false;
    hold();
  }

  if (st.active && allowed && st.fault == ActFault::NONE) {
    float err = st.goal_r - st.r;
    float ae = fabsf(err);
    // deadband with hysteresis: stop inside db, restart only beyond 2*db
    if (inDeadband_) inDeadband_ = ae < 2.0f * cfg.deadband;
    else inDeadband_ = ae < cfg.deadband;
    if (!inDeadband_) {
      float k = (ae - cfg.deadband) / (cfg.slow_zone - cfg.deadband);
      if (k > 1) k = 1;
      float mag = cfg.pwm_min + k * (cfg.pwm_max - cfg.pwm_min);
      target = err > 0 ? mag : -mag;
      if (cfg.invert) target = -target;
      // never push further outside the soft limits
      bool outward = (err > 0 && st.r > cfg.r_max) || (err < 0 && st.r < cfg.r_min);
      if (outward) target = 0;
    }
  }

  // slew limit: stopping/decelerating is immediate, reversing goes through zero,
  // accelerating is limited to pwm_slew per second
  float step = cfg.pwm_slew * dt;
  if ((st.duty > 0.01f && target < 0) || (st.duty < -0.01f && target > 0)) st.duty = 0;
  if (fabsf(target) <= fabsf(st.duty)) st.duty = target;
  else if (target > st.duty) st.duty = fminf(target, st.duty + step);
  else st.duty = fmaxf(target, st.duty - step);
  st.moving = fabsf(st.duty) > 0.01f;

  // over-current
  if (st.moving && st.current > cfg.i_limit) {
    if (!ocSinceMs_) ocSinceMs_ = now_ms;
    if (now_ms - ocSinceMs_ > cfg.oc_ms) st.fault = ActFault::OVERCURRENT;
  } else {
    ocSinceMs_ = 0;
  }
  // stall: driving but not moving
  if (st.moving && fabsf(st.duty) >= cfg.pwm_min * 0.95f) {
    if (fabsf(st.r - stallRefR_) > cfg.stall_delta) {
      stallRefR_ = st.r;
      stallRefMs_ = now_ms;
    } else if (now_ms - stallRefMs_ > cfg.stall_ms) {
      st.fault = ActFault::STALL;
    }
  } else {
    stallRefR_ = st.r;
    stallRefMs_ = now_ms;
  }
  // max drive time towards one target
  if (st.moving) {
    if (!driving_) driving_ = true, driveSinceMs_ = now_ms;
    else if (cfg.max_drive_ms && now_ms - driveSinceMs_ > cfg.max_drive_ms) st.fault = ActFault::TIMEOUT;
  } else {
    driving_ = false;
  }
  if (st.fault != ActFault::NONE) {
    driving_ = false;
    st.duty = 0;
    st.moving = false;
  }
  return st.duty;
}
