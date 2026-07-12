/**
 * @file  yaw.c
 * @brief Yaw estimator — fuses rigid-body sensor geometry with velocity-derived
 *        heading using inverse-variance circular weighting.
 */

#include <math.h>
#include <stdbool.h>
#include "yaw.h"
#include "angle_decoder/angle_decoder.h"   /* NUM_SENSORS */

#define YAW_PI            3.14159265358979323846f

/* Physical sensor layout in body frame — must match SYN_SENSOR_OFF in main.c:
 *   S0=(0,0)  S1=(L,0)  S2=(L,L)  S3=(0,L)  where L = SENSOR_BASELINE */
#define SENSOR_BASELINE   0.050f   /* 5 cm inter-sensor spacing */
#define YAW_DIST_TOL      0.020f   /* 20 mm pair-distance tolerance */

/* Velocity yaw tuning */
#define YAW_VEL_FLOOR_VAR 0.10f    /* min vel variance even at high speed [rad²] */
#define YAW_VEL_MIN_SPEED 0.10f    /* below this speed, vel yaw decays [m/s] */
#define YAW_VEL_DECAY     0.05f    /* variance inflation per 10 Hz tick when still */

/* 90° consistency gate: skip blending if estimates disagree more than this */
#define YAW_GATE_RAD      (YAW_PI * 0.5f)

/* Yaw variance [rad²] indexed by number of valid axis pairs (0–4). */
static const float YAW_VAR_TABLE[5] = { 9.87f, 0.40f, 0.20f, 0.10f, 0.05f };

/* -------------------------------------------------------------------------- */

/*
 * Estimate yaw from the rigid-body layout of the sensor array.
 *
 * Each pair whose body-frame displacement is along body-X or body-Y gives an
 * independent yaw estimate; they are fused via circular mean.  A pair is
 * rejected if its measured XY separation deviates from SENSOR_BASELINE by
 * more than YAW_DIST_TOL (bad triangulation or stale cross-contamination).
 *
 * Returns true (+ sets *yaw_out [rad] and *var_out [rad²]) when at least one
 * valid pair exists; false when no pair can be formed.
 */
static bool _yaw_from_sensors(const lh2_point3d_t *pts, int n,
                               float *yaw_out, float *var_out)
{
    float pos[NUM_SENSORS][3];
    bool  have[NUM_SENSORS];
    for (int s = 0; s < NUM_SENSORS; s++) have[s] = false;
    for (int i = 0; i < n; i++) {
        uint8_t s = pts[i].sensor_id;
        if (s < NUM_SENSORS) {
            pos[s][0] = pts[i].xyz[0];
            pos[s][1] = pts[i].xyz[1];
            pos[s][2] = pts[i].xyz[2];
            have[s]   = true;
        }
    }

    /*
     * Axis pairs: {from, to, is_body_x}.
     *   body_x=true  → W[to]−W[from] ≈ body-X in world → yaw = atan2(Δy, Δx)
     *   body_x=false → W[to]−W[from] ≈ body-Y in world → yaw = atan2(−Δx, Δy)
     */
    static const struct { uint8_t a, b; bool body_x; } PAIRS[4] = {
        {0, 1, true},    /* S0→S1: body +X */
        {3, 2, true},    /* S3→S2: body +X */
        {0, 3, false},   /* S0→S3: body +Y */
        {1, 2, false},   /* S1→S2: body +Y */
    };

    float sin_sum = 0.0f, cos_sum = 0.0f;
    int   n_pairs = 0;

    for (int p = 0; p < 4; p++) {
        uint8_t a = PAIRS[p].a, b = PAIRS[p].b;
        if (!have[a] || !have[b]) continue;

        float dx   = pos[b][0] - pos[a][0];
        float dy   = pos[b][1] - pos[a][1];
        float dist = sqrtf(dx*dx + dy*dy);

        if (fabsf(dist - SENSOR_BASELINE) > YAW_DIST_TOL) continue;

        float yaw_est = PAIRS[p].body_x ? atan2f(dy, dx)
                                        : atan2f(-dx, dy);
        sin_sum += sinf(yaw_est);
        cos_sum += cosf(yaw_est);
        n_pairs++;
    }

    if (n_pairs == 0) return false;

    *yaw_out = atan2f(sin_sum, cos_sum);
    *var_out = YAW_VAR_TABLE[n_pairs];
    return true;
}

/* -------------------------------------------------------------------------- */

void yaw_fuse(const lh2_point3d_t *pts, int n_pts,
              float cx, float cy, uint64_t now_us,
              float pos_var,
              yaw_vel_state_t *vel,
              float *yaw_out, float *var_out,
              float q_out[4])
{
    /* --- 1. Update velocity-yaw state ------------------------------------- */
    if (vel->prev_us > 0) {
        float dt  = (float)(now_us - vel->prev_us) * 1e-6f;
        float vx  = (cx - vel->prev_cx) / dt;
        float vy  = (cy - vel->prev_cy) / dt;
        float spd = sqrtf(vx*vx + vy*vy);

        if (spd > YAW_VEL_MIN_SPEED) {
            vel->yaw = atan2f(vy, vx);
            /* Angular noise from position differencing: σ_θ ≈ √2·σ_pos/(spd·dt)
             * → vel_var = 2·pos_var / (spd·dt)², floored to keep it honest
             * even when the velocity direction diverges from the body heading. */
            float d  = spd * dt;
            vel->var = 2.0f * pos_var / (d * d);
            if (vel->var < YAW_VEL_FLOOR_VAR) vel->var = YAW_VEL_FLOOR_VAR;
        } else {
            vel->var += YAW_VEL_DECAY;
            if (vel->var > 9.87f) vel->var = 9.87f;
        }
    }
    vel->prev_cx = cx;
    vel->prev_cy = cy;
    vel->prev_us = now_us;

    /* --- 2. Sensor-geometry yaw estimate ---------------------------------- */
    float sens_yaw = 0.0f, sens_var = 9.87f;
    bool  has_sens = _yaw_from_sensors(pts, n_pts, &sens_yaw, &sens_var);
    bool  has_vel  = (vel->var < 9.87f);

    /* --- 3. Inverse-variance circular fusion ------------------------------ */
    float yaw = 0.0f, var = 9.87f;

    if (has_sens && has_vel) {
        float diff = sens_yaw - vel->yaw;
        /* Wrap to [−π, π] */
        while (diff >  YAW_PI) diff -= 2.0f * YAW_PI;
        while (diff < -YAW_PI) diff += 2.0f * YAW_PI;

        if (fabsf(diff) < YAW_GATE_RAD) {
            /* Both estimates agree: blend proportional to confidence */
            float w_s  = 1.0f / sens_var;
            float w_v  = 1.0f / vel->var;
            yaw = atan2f(w_s * sinf(sens_yaw) + w_v * sinf(vel->yaw),
                         w_s * cosf(sens_yaw) + w_v * cosf(vel->yaw));
            var = 1.0f / (w_s + w_v);
        } else {
            /* Estimates disagree by > 90°: use the more confident source */
            if (sens_var <= vel->var) { yaw = sens_yaw; var = sens_var; }
            else                      { yaw = vel->yaw; var = vel->var; }
        }
    } else if (has_sens) {
        yaw = sens_yaw; var = sens_var;
    } else if (has_vel) {
        yaw = vel->yaw; var = vel->var;
    }
    /* else: yaw=0, var=9.87 → identity quaternion, EKF treats yaw as unknown */

    /* --- 4. Output -------------------------------------------------------- */
    *yaw_out = yaw;
    *var_out = var;
    q_out[0] = cosf(yaw * 0.5f);
    q_out[1] = 0.0f;
    q_out[2] = 0.0f;
    q_out[3] = sinf(yaw * 0.5f);
}
