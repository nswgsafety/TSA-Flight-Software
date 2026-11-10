--[[
  stick_alt.lua - throttle stick commands ABSOLUTE altitude in Guided mode.

  Install: SD card APM/scripts/stick_alt.lua, SCR_ENABLE=1, reboot.
  Fly:     take off in Loiter, switch to Guided. Leave Guided to release.

  While Guided and armed:
    throttle   -> altitude setpoint, MIN_FT (stick low) .. MAX_FT (stick high),
                  scaled with RCx_MIN/RCx_MAX/RCx_REVERSED, slew limited
    roll/pitch -> move the horizontal hold point (body frame), XY_SPEED at full stick
    yaw        -> yaw rate, YAW_RATE at full stick

  RC loss: the setpoints freeze, but only as a brief bridge until Copter's RC
  failsafe triggers and lands (FS_THR_ENABLE=3). Landing changes the mode, which
  releases this script. FS_OPTIONS must not have bit 2 (value 4, "Continue if in
  Guided on RC failsafe") set, or the vehicle would stay frozen in Guided.

  Altitude is the EKF height above its origin. With EK3_SRC1_POSZ=2 that is
  the rangefinder height, so set RNGFND1_GNDCLR to the measured sensor height.

  Written against ArduCopter 4.7.2 (libraries/AP_Scripting/docs/docs.lua).
--]]

local MIN_FT       = 0.5    -- altitude at throttle low (ft)
local MAX_FT       = 3.0    -- altitude at throttle high (ft)
local ALT_SLEW     = 0.3    -- max rate of change of the altitude setpoint (m/s)
local XY_SPEED     = 0.5    -- hold point speed at full roll/pitch stick (m/s), 0 = locked
local XY_LEAD_MAX  = 0.75   -- hold point may lead the vehicle by at most this (m)
local YAW_RATE     = 60     -- yaw rate at full yaw stick (deg/s), 0 = no yaw control

local FT           = 0.3048
local GUIDED       = 4      -- Copter mode number
local RNG_DOWN     = 25     -- ROTATION_PITCH_270
local PERIOD_MS    = 20
local MAV_WARNING  = 4
local MAV_INFO     = 6

local function get_channel(map_param, default)
  local n = param:get(map_param) or default
  local ch = rc:get_channel(math.floor(n))
  if not ch then
    error(string.format('stick_alt: no RC channel for %s', map_param))
  end
  return ch
end

local ch_roll  = get_channel('RCMAP_ROLL', 1)
local ch_pitch = get_channel('RCMAP_PITCH', 2)
local ch_thr   = get_channel('RCMAP_THROTTLE', 3)
local ch_yaw   = get_channel('RCMAP_YAW', 4)

local active = false
local hold_n, hold_e, target = 0, 0, 0
local last_s = 0
local last_warn_s = -10
local rng_bad_since = nil
local tgt = Vector3f()

local function clamp(v, lo, hi)
  return math.max(lo, math.min(hi, v))
end

local function now_s()
  return millis():tofloat() * 0.001
end

-- rate limited warning so a persistent fault does not flood the GCS
local function warn(now, text)
  if now - last_warn_s >= 5 then
    last_warn_s = now
    gcs:send_text(MAV_WARNING, 'stick_alt: ' .. text)
  end
end

local function update()
  local now = now_s()

  if vehicle:get_mode() ~= GUIDED or not arming:is_armed() then
    if active then
      gcs:send_text(MAV_INFO, 'stick_alt: released')
    end
    active = false
    rng_bad_since = nil
    return
  end

  local pos = ahrs:get_relative_position_NED_origin()
  if not pos then
    warn(now, 'no position estimate')
    return
  end
  local alt = -pos:z()

  if not active then
    -- latch where we are; the setpoint then slews from here to the stick position
    hold_n, hold_e, target = pos:x(), pos:y(), alt
    last_s = now
    active = true
    gcs:send_text(MAV_INFO, string.format('stick_alt: engaged at %.2f m', alt))
  end

  local dt = clamp(now - last_s, 0, 0.1)
  last_s = now

  local yaw_rate = 0
  if rc:has_valid_input() then
    -- throttle -> absolute altitude
    local f = (ch_thr:norm_input_ignore_trim() + 1) * 0.5
    local want = (MIN_FT + f * (MAX_FT - MIN_FT)) * FT
    local step = ALT_SLEW * dt
    target = target + clamp(want - target, -step, step)

    -- roll/pitch -> move the hold point in the body frame
    if XY_SPEED > 0 then
      local fwd = -ch_pitch:norm_input_dz() * XY_SPEED
      local right = ch_roll:norm_input_dz() * XY_SPEED
      local yaw = ahrs:get_yaw_rad()
      local c, s = math.cos(yaw), math.sin(yaw)
      hold_n = hold_n + (fwd * c - right * s) * dt
      hold_e = hold_e + (fwd * s + right * c) * dt

      -- keep the hold point on a leash so it cannot run away from the vehicle
      local dn, de = hold_n - pos:x(), hold_e - pos:y()
      local dist = math.sqrt(dn * dn + de * de)
      if dist > XY_LEAD_MAX then
        local k = XY_LEAD_MAX / dist
        hold_n = pos:x() + dn * k
        hold_e = pos:y() + de * k
      end
    end

    yaw_rate = ch_yaw:norm_input_dz() * YAW_RATE
  end
  -- on RC loss the setpoints freeze until Copter's RC failsafe switches to Land

  tgt:x(hold_n)
  tgt:y(hold_e)
  tgt:z(-target)
  if not vehicle:set_target_pos_NED(tgt, false, 0, YAW_RATE > 0, yaw_rate, false, false) then
    warn(now, 'target rejected')
  end

  -- rangefinder watchdog and tuning log (compare SALT.Tgt with SALT.Alt and SALT.Rng)
  local rng = -1
  if rangefinder:has_data_orient(RNG_DOWN) then
    rng = rangefinder:distance_orient(RNG_DOWN)
    rng_bad_since = nil
  else
    rng_bad_since = rng_bad_since or now
    if now - rng_bad_since > 0.5 then
      warn(now, 'rangefinder out of range, take over')
    end
  end
  logger:write('SALT', 'Tgt,Alt,Rng', 'fff', target, alt, rng)
end

local function loop()
  local ok, err = pcall(update)
  if not ok then
    active = false
    gcs:send_text(MAV_WARNING, 'stick_alt: ' .. tostring(err))
    return loop, 1000
  end
  return loop, PERIOD_MS
end

gcs:send_text(MAV_INFO, string.format('stick_alt: loaded, %.1f-%.1f ft', MIN_FT, MAX_FT))
return loop, 1000
