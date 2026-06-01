-- set_home_on_ekf.lua
--
-- Watches EKF health and sets the vehicle home the first time the EKF
-- becomes healthy after boot.  Designed for indoor LH2 VPE setups where
-- there is no GPS to trigger an automatic home set.
--
-- Deploy: copy to APM/scripts/ on the FC SD card.
-- Requires: ArduPilot 4.3+ (Copter/Rover/Plane with Lua scripting enabled).
--
-- NOTE on ODOMETRY vs VPE:
--   The firmware currently sends VISION_POSITION_ESTIMATE (msg #102).
--   ArduPilot 4.4+ (and PX4) prefer ODOMETRY (msg #331) because it also
--   carries linear velocity, which the EKF fuses for a much tighter state.
--   At 10 Hz you can finite-difference the centroid for vx/vy/vz and send
--   NaN covariances for unknowns.  Worth implementing in mavlink.c once
--   the VPE pipeline is stable.

local POLL_MS   = 500   -- check interval while waiting
local DONE_MS   = 5000  -- heartbeat interval after home is set

local home_set = false

local function update()
    if home_set then
        return update, DONE_MS
    end

    if not ahrs:initialised() then
        gcs:send_text(6, "LH2: waiting — AHRS not initialised")
        return update, POLL_MS
    end

    if not ahrs:healthy() then
        gcs:send_text(6, "LH2: waiting — EKF not healthy")
        return update, POLL_MS
    end

    local loc = ahrs:get_location()
    if not loc then
        gcs:send_text(6, "LH2: EKF healthy but no location yet")
        return update, POLL_MS
    end

    if ahrs:set_home(loc) then
        home_set = true
        gcs:send_text(6, "LH2: home set from EKF position")
    else
        gcs:send_text(6, "LH2: set_home failed")
    end

    return update, POLL_MS
end

return update()
