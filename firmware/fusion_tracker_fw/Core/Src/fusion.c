/**
 * @file  fusion.c
 * @brief Hand-written single-target EKF: radar position + IMU yaw -> world track.
 *
 * State x = [px, py, vx, vy]. Constant-velocity prediction; linear position
 * update from the (yaw-rotated) radar measurement. Q/R start from the values
 * tuned in the Python sim (sim/run_fusion.py).
 */
#include "fusion.h"
#include <math.h>

#define Q_DENSITY 0.2f     /* process-noise spectral density (tuned in sim)     */
#define R_POS     0.05f    /* radar position measurement variance, m^2 (~0.22 m) */
#define R_BEARING 0.01f    /* camera bearing variance, rad^2 (~6 deg) — loosened vs jitter */
#define COAST_MAX 8        /* radar misses (~0.8 s @10 Hz) before dropping track */

static float X[4];             /* state                                 */
static float P[4][4];          /* covariance                            */
static bool  s_have = false;   /* holding a track?                      */
static int   s_miss = 0;
static float s_yaw0 = 0.0f;    /* IMU yaw captured at startup (zero reference) */
static bool  s_yaw0_set = false;

static void mat4_mul(const float A[4][4], const float B[4][4], float C[4][4])
{
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      float s = 0.0f;
      for (int k = 0; k < 4; k++) s += A[i][k] * B[k][j];
      C[i][j] = s;
    }
}

void fusion_init(void)
{
  for (int i = 0; i < 4; i++) {
    X[i] = 0.0f;
    for (int j = 0; j < 4; j++) P[i][j] = 0.0f;
  }
  s_have = false;
  s_miss = 0;
}

/* x = F x ;  P = F P F^T + Q   (constant-velocity F for step dt) */
static void predict(float dt)
{
  X[0] += dt * X[2];
  X[1] += dt * X[3];

  const float F[4][4] = {{1, 0, dt, 0}, {0, 1, 0, dt}, {0, 0, 1, 0}, {0, 0, 0, 1}};
  float Ft[4][4], FP[4][4], NP[4][4];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) Ft[i][j] = F[j][i];
  mat4_mul(F, P, FP);
  mat4_mul(FP, Ft, NP);

  const float d3 = dt * dt * dt / 3.0f, d2 = dt * dt / 2.0f;
  const float Q[4][4] = {{d3, 0, d2, 0}, {0, d3, 0, d2}, {d2, 0, dt, 0}, {0, d2, 0, dt}};
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) P[i][j] = NP[i][j] + Q_DENSITY * Q[i][j];
}

/* Linear position update (H selects px,py). Exploits H's structure. */
static void update_pos(float zx, float zy)
{
  const float y0 = zx - X[0], y1 = zy - X[1];               /* innovation */
  const float s00 = P[0][0] + R_POS, s01 = P[0][1];
  const float s10 = P[1][0], s11 = P[1][1] + R_POS;         /* S = HPH^T+R */
  const float det = s00 * s11 - s01 * s10;
  if (fabsf(det) < 1e-9f) return;
  const float i00 = s11 / det, i01 = -s01 / det;
  const float i10 = -s10 / det, i11 = s00 / det;            /* S^-1 */

  float K[4][2];                                            /* K = P[:,0:2] S^-1 */
  for (int i = 0; i < 4; i++) {
    K[i][0] = P[i][0] * i00 + P[i][1] * i10;
    K[i][1] = P[i][0] * i01 + P[i][1] * i11;
  }
  for (int i = 0; i < 4; i++) X[i] += K[i][0] * y0 + K[i][1] * y1;

  float r0[4], r1[4];                                       /* P = (I-KH)P */
  for (int j = 0; j < 4; j++) { r0[j] = P[0][j]; r1[j] = P[1][j]; }
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) P[i][j] -= K[i][0] * r0[j] + K[i][1] * r1[j];
}

void fusion_step(const ld2450_frame_t *radar, float yaw_rad, float dt)
{
  const ld2450_target_t *tg = NULL;
  for (int t = 0; t < LD2450_MAX_TARGETS; t++)
    if (radar->target[t].valid) { tg = &radar->target[t]; break; }

  if (!s_yaw0_set) { s_yaw0 = yaw_rad; s_yaw0_set = true; }
  const float yaw = yaw_rad - s_yaw0;           /* zero yaw to boot orientation */
  const float c = cosf(yaw), s = sinf(yaw);

  if (!s_have) {                       /* acquire: init from first detection */
    if (tg) {
      const float xs = tg->x * 0.001f, ys = tg->y * 0.001f;   /* mm -> m */
      X[0] = c * xs - s * ys;  X[1] = s * xs + c * ys;
      X[2] = 0.0f;             X[3] = 0.0f;
      for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) P[i][j] = (i == j) ? ((i < 2) ? 1.0f : 4.0f) : 0.0f;
      s_have = true; s_miss = 0;
    }
    return;
  }

  predict(dt);
  if (tg) {
    const float xs = tg->x * 0.001f, ys = tg->y * 0.001f;
    update_pos(c * xs - s * ys, s * xs + c * ys);            /* rotate to world */
    s_miss = 0;
  } else if (++s_miss > COAST_MAX) {
    s_have = false;                                          /* coasted too long */
  }
}

/* Nonlinear bearing update from the camera. h(x) = atan2(px, py):
 * 0 = target straight out (+Y), positive = to the right (+X). */
void fusion_update_camera(float world_bearing_rad)
{
  if (!s_have) return;
  const float px = X[0], py = X[1], r2 = px * px + py * py;
  if (r2 < 1e-4f) return;                       /* avoid singularity at origin */
  const float h = atan2f(px, py);
  const float H0 = py / r2, H1 = -px / r2;      /* dh/dpx, dh/dpy */

  float y = (world_bearing_rad - s_yaw0) - h;   /* remove startup yaw offset; wrapped below */
  while (y > 3.14159265f)  y -= 6.28318531f;
  while (y < -3.14159265f) y += 6.28318531f;

  const float HP0 = H0 * P[0][0] + H1 * P[1][0];
  const float HP1 = H0 * P[0][1] + H1 * P[1][1];
  const float S = HP0 * H0 + HP1 * H1 + R_BEARING;   /* scalar */
  if (S < 1e-9f) return;

  float K[4];                                   /* K = P H^T / S */
  for (int i = 0; i < 4; i++) K[i] = (P[i][0] * H0 + P[i][1] * H1) / S;
  for (int i = 0; i < 4; i++) X[i] += K[i] * y;

  float r0[4], r1[4];                           /* P = (I - K H) P */
  for (int j = 0; j < 4; j++) { r0[j] = P[0][j]; r1[j] = P[1][j]; }
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      P[i][j] -= K[i] * (H0 * r0[j] + H1 * r1[j]);
}

fused_track_t fusion_track(void)
{
  fused_track_t t = { X[0], X[1], X[2], X[3], s_have };
  return t;
}
