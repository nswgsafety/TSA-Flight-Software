--[[
  Offline logic test for TSA/scripts/stick_alt.lua.

  Runs the real script against a mocked ArduPilot scripting API and a crude
  vehicle model. It proves the script's arithmetic and state handling only.
  It is NOT a substitute for SITL (TSA/SITL.md) or a flight.

  Run from the repo root with any Lua 5.3:
      lua TSA/tests/test_stick_alt.lua
  or, without a Lua install (Node.js only), see TSA/tests/run.js.
--]]

local SCRIPT = SCRIPT_PATH or 'TSA/scripts/stick_alt.lua'
local FT = 0.3048

---------------------------------------------------------------- mock world
local S = {
  t_ms = 0,
  mode = 4, armed = true, rc_valid = true, pos_ok = true, rng_ok = true,
  pwm = { 1500, 1500, 1500, 1500 },
  rc_min = { 1000, 1000, 1000, 1000 },
  rc_max = { 2000, 2000, 2000, 2000 },
  rc_rev = { false, false, false, false },
  n = 0, e = 0, alt = 0.4, yaw = 0,
  frozen = false,          -- vehicle cannot move (leash test)
  tgt = nil, yaw_rate = nil, use_yaw_rate = nil, calls = 0,
  texts = {},
}

local function new_vec()
  local v = { _x = 0, _y = 0, _z = 0 }
  local function acc(k)
    return function(self, val)
      if val == nil then return self[k] end
      self[k] = val
    end
  end
  v.x, v.y, v.z = acc('_x'), acc('_y'), acc('_z')
  return v
end
function Vector3f() return new_vec() end

function millis()
  local t = S.t_ms
  return { tofloat = function() return t end }
end

param = {}
function param:get(name)
  local map = { RCMAP_ROLL = 1, RCMAP_PITCH = 2, RCMAP_THROTTLE = 3, RCMAP_YAW = 4 }
  return map[name]
end

local function make_channel(i)
  local ch = {}
  -- same arithmetic as RC_Channel::norm_input_ignore_trim()
  function ch:norm_input_ignore_trim()
    local lo, hi = S.rc_min[i], S.rc_max[i]
    if hi <= lo then return 0 end
    local r = (S.rc_rev[i] and -2 or 2) * ((S.pwm[i] - lo) / (hi - lo) - 0.5)
    return math.max(-1, math.min(1, r))
  end
  function ch:norm_input_dz()
    local trim, dz = 1500, 20
    local d = S.pwm[i] - trim
    if math.abs(d) <= dz then return 0 end
    local r = d > 0 and d / (S.rc_max[i] - trim) or d / (trim - S.rc_min[i])
    if S.rc_rev[i] then r = -r end
    return math.max(-1, math.min(1, r))
  end
  return ch
end
local channels = { make_channel(1), make_channel(2), make_channel(3), make_channel(4) }

rc = {}
function rc:get_channel(n) return channels[n] end
function rc:has_valid_input() return S.rc_valid end

vehicle = {}
function vehicle:get_mode() return S.mode end
function vehicle:set_target_pos_NED(t, use_yaw, yaw_deg, use_yaw_rate, yaw_rate, yaw_rel, terrain)
  assert(use_yaw == false and yaw_rel == false and terrain == false, 'unexpected flags')
  S.calls = S.calls + 1
  S.tgt = { n = t:x(), e = t:y(), alt = -t:z() }
  S.use_yaw_rate, S.yaw_rate = use_yaw_rate, yaw_rate
  return true
end

arming = {}
function arming:is_armed() return S.armed end

ahrs = {}
function ahrs:get_relative_position_NED_origin()
  if not S.pos_ok then return nil end
  local v = new_vec()
  v:x(S.n); v:y(S.e); v:z(-S.alt)
  return v
end
function ahrs:get_yaw_rad() return S.yaw end

rangefinder = {}
function rangefinder:has_data_orient(_) return S.rng_ok end
function rangefinder:distance_orient(_) return S.alt end

gcs = {}
function gcs:send_text(_, text) S.texts[#S.texts + 1] = text end

logger = {}
function logger:write(...) end

---------------------------------------------------------------- run script
local loop, delay = assert(loadfile(SCRIPT))()

-- advance simulated time, calling the script at the interval it asks for
local function run(seconds)
  local stop = S.t_ms + seconds * 1000
  while S.t_ms < stop do
    loop, delay = loop()
    local dt = delay / 1000
    S.t_ms = S.t_ms + delay
    -- crude vehicle: first-order lag towards the commanded target
    if S.tgt and S.mode == 4 and S.armed and not S.frozen then
      local k = math.min(1, dt / 0.15)
      S.n = S.n + (S.tgt.n - S.n) * k
      S.e = S.e + (S.tgt.e - S.e) * k
      S.alt = S.alt + (S.tgt.alt - S.alt) * k
    end
  end
end

local pass, fail = 0, 0
local function check(name, cond, detail)
  if cond then
    pass = pass + 1
    print('PASS  ' .. name)
  else
    fail = fail + 1
    print('FAIL  ' .. name .. (detail and ('  [' .. detail .. ']') or ''))
  end
end
local function near(a, b, tol) return math.abs(a - b) <= tol end
local function saw(text)
  for _, t in ipairs(S.texts) do
    if t:find(text, 1, true) then return true end
  end
  return false
end

---------------------------------------------------------------- tests
run(1.5)   -- initial 1 s load delay, then engage
check('loads and announces', saw('stick_alt: loaded'))
check('engages in Guided', saw('stick_alt: engaged'))

-- 1-3: absolute mapping at low / mid / high stick
S.pwm[3] = 1000; run(6)
check('stick low  -> 0.5 ft', near(S.tgt.alt, 0.5 * FT, 1e-6), string.format('%.4f m', S.tgt.alt))
check('vehicle follows to 0.5 ft', near(S.alt, 0.5 * FT, 0.01), string.format('%.4f m', S.alt))
S.pwm[3] = 1500; run(6)
check('stick mid  -> 1.75 ft', near(S.tgt.alt, 1.75 * FT, 1e-6), string.format('%.4f m', S.tgt.alt))
S.pwm[3] = 2000; run(6)
check('stick high -> 3.0 ft', near(S.tgt.alt, 3.0 * FT, 1e-6), string.format('%.4f m', S.tgt.alt))

-- 4: absolute, not climb rate: holding the stick does not keep climbing
S.pwm[3] = 1700; run(6)
local a1 = S.tgt.alt
run(30)
check('held stick holds 2.25 ft for 30 s', near(S.tgt.alt, 2.25 * FT, 1e-6) and near(a1, S.tgt.alt, 1e-9),
      string.format('%.4f m', S.tgt.alt))

-- 5: slew limit 0.3 m/s (0.5 ft -> 3 ft is 0.762 m, so about 2.54 s)
S.pwm[3] = 1000; run(6)
S.pwm[3] = 2000; run(1.0)
check('slew: 1 s after full stick, moved 0.30 m', near(S.tgt.alt - 0.5 * FT, 0.30, 0.01),
      string.format('%.3f m', S.tgt.alt - 0.5 * FT))
run(1.4)
check('slew: not there at 2.4 s', S.tgt.alt < 3.0 * FT - 0.02, string.format('%.3f m', S.tgt.alt))
run(0.4)
check('slew: there by 2.8 s', near(S.tgt.alt, 3.0 * FT, 1e-6), string.format('%.3f m', S.tgt.alt))
check('never commands above 3 ft', S.tgt.alt <= 3.0 * FT + 1e-9)

-- 6: uses RC3_MIN / RC3_MAX, and reversal
S.rc_min[3], S.rc_max[3] = 988, 2012
S.pwm[3] = 988; run(6)
check('RC3_MIN=988 maps to 0.5 ft', near(S.tgt.alt, 0.5 * FT, 1e-6), string.format('%.4f m', S.tgt.alt))
S.pwm[3] = 2012; run(6)
check('RC3_MAX=2012 maps to 3.0 ft', near(S.tgt.alt, 3.0 * FT, 1e-6), string.format('%.4f m', S.tgt.alt))
S.rc_rev[3] = true; run(6)
check('RC3_REVERSED flips the mapping', near(S.tgt.alt, 0.5 * FT, 1e-6), string.format('%.4f m', S.tgt.alt))
S.rc_rev[3] = false; S.rc_min[3], S.rc_max[3] = 1000, 2000
S.pwm[3] = 1500; run(6)

-- 7: pitch forward moves the hold point north at yaw 0, at about 0.5 m/s
local n0, e0, alt0 = S.n, S.e, S.tgt.alt
S.pwm[2] = 1000; run(3); S.pwm[2] = 1500; run(2)
check('pitch forward moves north ~1.5 m', near(S.n - n0, 1.5, 0.1), string.format('%.2f m', S.n - n0))
check('pitch forward does not move east', near(S.e - e0, 0, 0.01), string.format('%.3f m', S.e - e0))
check('height unchanged while translating', near(S.tgt.alt, alt0, 1e-9))
local n1 = S.n
run(3)
check('stops when stick is centred', near(S.n, n1, 0.01), string.format('%.3f m', S.n - n1))

-- 8: roll right moves east at yaw 0
n0, e0 = S.n, S.e
S.pwm[1] = 2000; run(2); S.pwm[1] = 1500; run(2)
check('roll right moves east ~1.0 m', near(S.e - e0, 1.0, 0.1) and near(S.n - n0, 0, 0.01),
      string.format('n %.2f e %.2f', S.n - n0, S.e - e0))

-- 9: body frame: facing east, pitch forward moves east
S.yaw = math.pi / 2
n0, e0 = S.n, S.e
S.pwm[2] = 1000; run(2); S.pwm[2] = 1500; run(2)
check('facing east, forward moves east', near(S.e - e0, 1.0, 0.1) and near(S.n - n0, 0, 0.01),
      string.format('n %.2f e %.2f', S.n - n0, S.e - e0))
S.yaw = 0

-- 10: leash: a blocked vehicle cannot be led by more than 0.75 m
S.frozen = true
S.pwm[2] = 1000; run(10); S.pwm[2] = 1500
local lead = math.sqrt((S.tgt.n - S.n) ^ 2 + (S.tgt.e - S.e) ^ 2)
check('hold point leash is 0.75 m', near(lead, 0.75, 0.02), string.format('%.3f m', lead))
S.frozen = false; run(4)

-- 11: yaw stick gives yaw rate, centre gives zero
S.pwm[4] = 2000; run(0.2)
check('full yaw stick -> 60 deg/s', S.use_yaw_rate == true and near(S.yaw_rate, 60, 1e-6), tostring(S.yaw_rate))
S.pwm[4] = 1500; run(0.2)
check('centred yaw stick -> 0 deg/s', near(S.yaw_rate, 0, 1e-9), tostring(S.yaw_rate))

-- 12: RC loss freezes the setpoints (bridge until the failsafe lands)
local hold = { n = S.tgt.n, e = S.tgt.e, alt = S.tgt.alt }
S.rc_valid = false
S.pwm[3], S.pwm[2], S.pwm[4] = 900, 1000, 2000   -- garbage while the link is down
run(3)
check('RC loss: height setpoint frozen', near(S.tgt.alt, hold.alt, 1e-9), string.format('%.4f m', S.tgt.alt))
check('RC loss: position setpoint frozen', near(S.tgt.n, hold.n, 1e-9) and near(S.tgt.e, hold.e, 1e-9))
check('RC loss: yaw rate zero', near(S.yaw_rate, 0, 1e-9))
S.rc_valid = true; S.pwm = { 1500, 1500, 1500, 1500 }; run(4)

-- 13: leaving Guided releases; nothing is commanded outside Guided
local calls = S.calls
S.mode = 9; run(2)   -- Land, as the RC failsafe would select
check('releases when mode leaves Guided', saw('stick_alt: released'))
check('no targets sent outside Guided', S.calls == calls, string.format('%d extra', S.calls - calls))

-- 14: re-engage latches the current height first, then slews (no jump)
S.alt = 0.80; S.n, S.e = 5, 5
S.mode = 4; run(0.02)
check('re-engage starts from current height', near(S.tgt.alt, 0.80, 0.01), string.format('%.3f m', S.tgt.alt))
check('re-engage holds current position', near(S.tgt.n, 5, 1e-6) and near(S.tgt.e, 5, 1e-6))

-- 15: disarmed -> inactive
calls = S.calls
S.armed = false; run(1)
check('no targets sent when disarmed', S.calls == calls)
S.armed = true; run(1)

-- 16: lost position estimate -> warns, sends nothing
calls = S.calls
S.pos_ok = false; run(1)
check('no position: warns and sends nothing', S.calls == calls and saw('no position estimate'))
S.pos_ok = true; run(1)

-- 17: rangefinder dropout -> warns the pilot
S.t_ms = S.t_ms + 6000   -- clear the 5 s warning rate limit
S.rng_ok = false; run(2)
check('rangefinder dropout warns', saw('rangefinder out of range'))
S.rng_ok = true

print(string.format('\n%d passed, %d failed', pass, fail))
if fail > 0 then error('stick_alt logic test failed') end
