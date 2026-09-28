"""Project a detection's pixel column to an azimuth bearing (degrees).

The camera's horizontal field of view maps image columns to angles: a person at
the image center is at 0 deg; at the far left/right edge, -/+ HFOV/2. Positive =
to the right of the optical axis. The exact HFOV is calibratable; the Pi Camera
Module 3 (standard lens) is ~66 deg horizontal.
"""

DEFAULT_HFOV_DEG = 66.0     # Pi Camera Module 3 (standard) approx. horizontal FOV


def pixel_to_bearing(cx, image_width, hfov_deg=DEFAULT_HFOV_DEG):
    """cx: bbox center column (px). Returns azimuth in degrees, +right of center."""
    norm = (cx - image_width / 2.0) / (image_width / 2.0)   # -1 (left) .. +1 (right)
    return norm * (hfov_deg / 2.0)
