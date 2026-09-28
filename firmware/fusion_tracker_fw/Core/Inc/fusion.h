/**
 * @file  fusion.h
 * @brief Single-target Extended Kalman Filter fusing radar position with the
 *        IMU heading (yaw) into a world-frame track.
 *
 * State x = [px, py, vx, vy] (world frame, metres / m·s^-1), constant-velocity.
 * The radar gives Cartesian position in the SENSOR frame; the IMU yaw rotates it
 * into a fixed WORLD frame before the (linear) position update. Mirrors sim/ekf.py.
 */
#ifndef FUSION_H
#define FUSION_H

#include <stdbool.h>
#include "ld2450.h"

typedef struct {
  float px, py;   /* m, world frame          */
  float vx, vy;   /* m/s                      */
  bool  valid;    /* a track is being held    */
} fused_track_t;

/** Reset the filter. */
void fusion_init(void);

/** Advance dt seconds and fuse the newest radar frame, rotating the measurement
 *  into the world frame with yaw_rad (from the IMU). */
void fusion_step(const ld2450_frame_t *radar, float yaw_rad, float dt);

/** Nonlinear bearing update from the camera. world_bearing_rad is the target
 *  azimuth in the world frame (0 = straight out / +Y, + = right / +X). */
void fusion_update_camera(float world_bearing_rad);

/** Current fused track estimate. */
fused_track_t fusion_track(void);

#endif /* FUSION_H */
