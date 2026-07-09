-- lh2_bs_params.lua
--
-- Pushes LH2 base-station geometry to the Pico over MAVLink UART by sending
-- 25 NAMED_VALUE_FLOAT messages every second.  The Pico parses these directly
-- without any parameter-list handshake, so it receives the data within 1–2 s
-- of boot regardless of how many other FC parameters exist.
--
-- Name convention (≤10 chars, no prefix):
--   NUMBS         — number of active base stations
--   BS{i}X/Y/Z   — world-frame origin of BS i [m]
--   BS{i}R{r}{c} — rotation matrix entry (local→world), row r, col c
--
-- Room geometry lives here. Update after any physical re-measurement and
-- redeploy to APM/scripts/ on the FC SD card.
--
-- The param table registration below is kept for Mission Planner visibility.
-- It does NOT affect what the Pico receives.
--
-- Deploy: copy to APM/scripts/ on the FC SD card.

-- ── Param table (Mission Planner visibility only) ───────────────────────────

local PARAM_TABLE_KEY    = 73
local PARAM_TABLE_PREFIX = "LH2_"
local NUM_PARAMS         = 25

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, NUM_PARAMS),
       "LH2_BS: add_table failed")
assert(param:add_param(PARAM_TABLE_KEY,  1, 'NUM_BS',   2),    "LH2_BS: NUM_BS")
assert(param:add_param(PARAM_TABLE_KEY,  2, 'BS0_X',    0.00), "LH2_BS: BS0_X")
assert(param:add_param(PARAM_TABLE_KEY,  3, 'BS0_Y',    0.00), "LH2_BS: BS0_Y")
assert(param:add_param(PARAM_TABLE_KEY,  4, 'BS0_Z',    3.45), "LH2_BS: BS0_Z")
assert(param:add_param(PARAM_TABLE_KEY,  5, 'BS0_R00',  0.00), "LH2_BS: BS0_R00")
assert(param:add_param(PARAM_TABLE_KEY,  6, 'BS0_R01', -1.00), "LH2_BS: BS0_R01")
assert(param:add_param(PARAM_TABLE_KEY,  7, 'BS0_R02',  0.00), "LH2_BS: BS0_R02")
assert(param:add_param(PARAM_TABLE_KEY,  8, 'BS0_R10',  0.00), "LH2_BS: BS0_R10")
assert(param:add_param(PARAM_TABLE_KEY,  9, 'BS0_R11',  0.00), "LH2_BS: BS0_R11")
assert(param:add_param(PARAM_TABLE_KEY, 10, 'BS0_R12',  1.00), "LH2_BS: BS0_R12")
assert(param:add_param(PARAM_TABLE_KEY, 11, 'BS0_R20', -1.00), "LH2_BS: BS0_R20")
assert(param:add_param(PARAM_TABLE_KEY, 12, 'BS0_R21',  0.00), "LH2_BS: BS0_R21")
assert(param:add_param(PARAM_TABLE_KEY, 13, 'BS0_R22',  0.00), "LH2_BS: BS0_R22")
assert(param:add_param(PARAM_TABLE_KEY, 14, 'BS1_X',    2.26), "LH2_BS: BS1_X")
assert(param:add_param(PARAM_TABLE_KEY, 15, 'BS1_Y',    0.00), "LH2_BS: BS1_Y")
assert(param:add_param(PARAM_TABLE_KEY, 16, 'BS1_Z',    3.45), "LH2_BS: BS1_Z")
assert(param:add_param(PARAM_TABLE_KEY, 17, 'BS1_R00',  0.00), "LH2_BS: BS1_R00")
assert(param:add_param(PARAM_TABLE_KEY, 18, 'BS1_R01', -1.00), "LH2_BS: BS1_R01")
assert(param:add_param(PARAM_TABLE_KEY, 19, 'BS1_R02',  0.00), "LH2_BS: BS1_R02")
assert(param:add_param(PARAM_TABLE_KEY, 20, 'BS1_R10',  0.00), "LH2_BS: BS1_R10")
assert(param:add_param(PARAM_TABLE_KEY, 21, 'BS1_R11',  0.00), "LH2_BS: BS1_R11")
assert(param:add_param(PARAM_TABLE_KEY, 22, 'BS1_R12',  1.00), "LH2_BS: BS1_R12")
assert(param:add_param(PARAM_TABLE_KEY, 23, 'BS1_R20', -1.00), "LH2_BS: BS1_R20")
assert(param:add_param(PARAM_TABLE_KEY, 24, 'BS1_R21',  0.00), "LH2_BS: BS1_R21")
assert(param:add_param(PARAM_TABLE_KEY, 25, 'BS1_R22',  0.00), "LH2_BS: BS1_R22")

gcs:send_text(6, "LH2_BS: ready, pushing poses @ 1 Hz")

-- ── NAMED_VALUE_FLOAT push (received by Pico firmware) ──────────────────────

local function update()
    gcs:send_named_float('NUMBS',   param:get('LH2_NUM_BS'))
    gcs:send_named_float('BS0X',    param:get('LH2_BS0_X'))
    gcs:send_named_float('BS0Y',    param:get('LH2_BS0_Y'))
    gcs:send_named_float('BS0Z',    param:get('LH2_BS0_Z'))
    gcs:send_named_float('BS0R00',  param:get('LH2_BS0_R00'))
    gcs:send_named_float('BS0R01',  param:get('LH2_BS0_R01'))
    gcs:send_named_float('BS0R02',  param:get('LH2_BS0_R02'))
    gcs:send_named_float('BS0R10',  param:get('LH2_BS0_R10'))
    gcs:send_named_float('BS0R11',  param:get('LH2_BS0_R11'))
    gcs:send_named_float('BS0R12',  param:get('LH2_BS0_R12'))
    gcs:send_named_float('BS0R20',  param:get('LH2_BS0_R20'))
    gcs:send_named_float('BS0R21',  param:get('LH2_BS0_R21'))
    gcs:send_named_float('BS0R22',  param:get('LH2_BS0_R22'))
    gcs:send_named_float('BS1X',    param:get('LH2_BS1_X'))
    gcs:send_named_float('BS1Y',    param:get('LH2_BS1_Y'))
    gcs:send_named_float('BS1Z',    param:get('LH2_BS1_Z'))
    gcs:send_named_float('BS1R00',  param:get('LH2_BS1_R00'))
    gcs:send_named_float('BS1R01',  param:get('LH2_BS1_R01'))
    gcs:send_named_float('BS1R02',  param:get('LH2_BS1_R02'))
    gcs:send_named_float('BS1R10',  param:get('LH2_BS1_R10'))
    gcs:send_named_float('BS1R11',  param:get('LH2_BS1_R11'))
    gcs:send_named_float('BS1R12',  param:get('LH2_BS1_R12'))
    gcs:send_named_float('BS1R20',  param:get('LH2_BS1_R20'))
    gcs:send_named_float('BS1R21',  param:get('LH2_BS1_R21'))
    gcs:send_named_float('BS1R22',  param:get('LH2_BS1_R22'))
    return update, 1000
end

return update, 1000
