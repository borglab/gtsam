#!/usr/bin/env python3
"""Extract SamoaWaypoints.h from a real vehicle_pose_est.csv trajectory.

Reads a pose CSV (columns: timestamp,tx,ty,tz,qw,qx,qy,qz), keeps the window
[--start, --start + --duration) seconds from the first row's timestamp,
converts the logged quaternion to aerospace roll/pitch/yaw and unwraps yaw,
smooths tx/ty/tz and the three angle channels with a short moving average to
remove per-sample estimator jitter, resamples to --waypoints points evenly
spaced in TIME (dt = duration / (waypoints - 1)) so the output replays at
the real speed and turn rate, rigidly transforms it so the first waypoint
is at the origin and the first yaw is exactly zero (gravity axis left
alone), and writes the result as a
gtsam_unstable/examples_common/SamoaWaypoints.h header in the same style as
IMUScenarios.h's defaultSplineWaypoints().

Usage:
    extract_survey_waypoints.py INPUT.csv [--start 590] [--duration 300]
        [--waypoints 1500] [--smooth-window 2.0] [--output SamoaWaypoints.h]
"""
import argparse
import csv
import math
import os

import numpy as np


def read_poses(path, start, duration):
    """Return (timestamps, positions, quaternions) for rows in the window
    [start, start + duration) s relative to row 0's timestamp.

    timestamps are re-zeroed to the start of the window. quaternions are
    (qw, qx, qy, qz) tuples.
    """
    timestamps, positions, quats = [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        t0 = None
        for row in reader:
            t = float(row["timestamp"])
            if t0 is None:
                t0 = t
            rel_t = t - t0
            if rel_t < start:
                continue
            if rel_t > start + duration:
                break
            timestamps.append(rel_t - start)
            positions.append(
                (float(row["tx"]), float(row["ty"]), float(row["tz"]))
            )
            quats.append(
                (float(row["qw"]), float(row["qx"]), float(row["qy"]),
                 float(row["qz"]))
            )
    return timestamps, positions, quats


def quats_to_euler(quats):
    """Convert (qw, qx, qy, qz) quaternions to aerospace 3-2-1 Euler angles.

    R = Rz(yaw) * Ry(pitch) * Rx(roll), body-to-nav. Returns (roll, pitch,
    yaw) arrays in radians; yaw is unwrapped, so it may exceed +-pi.
    """
    q = np.asarray(quats)
    qw, qx, qy, qz = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
    pitch = np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1.0, 1.0))
    roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
    return roll, pitch, np.unwrap(yaw)


def smooth(timestamps, values, window_s):
    """Centered moving average over a `window_s`-second window, by sample
    count inferred from the local sample rate (irregular timestamps are
    fine). values is an (N, C) array; returns an array of the same shape."""
    values = np.asarray(values, dtype=float)
    n = len(values)
    out = np.empty_like(values)
    half = window_s / 2.0
    lo = 0
    for i in range(n):
        while timestamps[i] - timestamps[lo] > half:
            lo += 1
        hi = i
        while hi + 1 < n and timestamps[hi + 1] - timestamps[i] <= half:
            hi += 1
        out[i] = values[lo:hi + 1].mean(axis=0)
    return out


def resample_by_time(timestamps, positions, extra, num_waypoints, duration):
    """Uniform-in-time resample to num_waypoints samples spaced
    dt = duration / (num_waypoints - 1) apart across the window, linearly
    interpolating positions and `extra` channels against `timestamps`."""
    timestamps = np.asarray(timestamps, dtype=float)
    positions = np.asarray(positions, dtype=float)
    extra = np.asarray(extra, dtype=float)
    targets = duration * np.arange(num_waypoints) / (num_waypoints - 1)
    out_pos = np.column_stack(
        [np.interp(targets, timestamps, positions[:, c])
         for c in range(positions.shape[1])]
    )
    out_extra = np.column_stack(
        [np.interp(targets, timestamps, extra[:, c])
         for c in range(extra.shape[1])]
    )
    return out_pos, out_extra


def to_inertial_frame(positions, yaw):
    """Recenter at the origin and align the initial heading with nav +x.

    Applies the rigid transform p -> Rz(-yaw0) (p - p0) to the positions and
    subtracts yaw0 from the yaw channel, so the first waypoint lands exactly
    on the origin, the first yaw is exactly zero, and the initial direction
    of travel points along the nav x axis.

    Only yaw is removed, never roll/pitch: +z must stay the gravity (depth)
    axis, and a full 3-D de-rotation would tilt it, breaking both the
    gravity alignment the filters assume and the depth sensor's meaning.
    The vehicle's true (small) initial roll and pitch are kept in the
    output.
    """
    heading0 = yaw[0]
    cos_y, sin_y = math.cos(-heading0), math.sin(-heading0)
    x0, y0, z0 = positions[0]
    out_pos = []
    for x, y, z in positions:
        dx, dy, dz = x - x0, y - y0, z - z0
        out_pos.append((cos_y * dx - sin_y * dy, sin_y * dx + cos_y * dy, dz))
    out_yaw = [yw - heading0 for yw in yaw]
    return out_pos, out_yaw, heading0


def format_vectors(vectors, precision):
    lines = []
    per_line = 2
    for i in range(0, len(vectors), per_line):
        chunk = vectors[i:i + per_line]
        entries = ", ".join(
            "gtsam::Vector3({:.{p}f}, {:.{p}f}, {:.{p}f})".format(
                *(round(v, precision) + 0.0 for v in pt), p=precision
            )
            for pt in chunk
        )
        lines.append(f"      {entries},")
    return lines


def render_header(waypoints, attitudes, source_path, start, duration, dt,
                  heading0):
    lines = []
    lines.append("/**")
    lines.append(" * @file SamoaWaypoints.h")
    lines.append(" * @brief Waypoints and attitudes for the SamoaSurvey scenario, generated")
    lines.append(" *        from a real AUV trajectory. Regenerate with")
    lines.append(" *        scripts/extract_survey_waypoints.py; do not hand-edit.")
    lines.append(" *")
    lines.append(f" * Source: {source_path}")
    lines.append(" * The window matters: the vehicle is on the surface and descending for")
    lines.append(" * the first ~300 s of this log, where the logged heading disagrees with")
    lines.append(" * the course over ground by ~167 deg and the apparent speed is 2-3 m/s.")
    lines.append(" * The on-bottom survey runs from ~300 s to ~1300 s, and there the logged")
    lines.append(" * attitude agrees with the course to a few degrees; R^T v resolves to")
    lines.append(" * 0.42 m/s surge with ~0.01 m/s sway. Pick a window inside that span.")
    lines.append(" *")
    lines.append(f" * Window: [{start:g}, {start + duration:g}) s from the start of the log,")
    lines.append(f" *        {duration:g} s duration. Position and attitude (converted from the")
    lines.append(" *        logged quaternion to aerospace roll/pitch/yaw) are both extracted,")
    lines.append(" *        smoothed, and resampled to {} waypoints uniform in TIME".format(len(waypoints)))
    lines.append(f" *        (dt = {dt:.6f} s), so the sequence replays at the real speed and")
    lines.append(" *        turn rate -- NOT evenly spaced by arc length. Rigidly moved into")
    lines.append(" *        the inertial frame: translated so the first waypoint is exactly")
    lines.append(" *        the origin, and rotated about the gravity axis by")
    lines.append(f" *        {-heading0:+.4f} rad so the first yaw is exactly zero and the initial")
    lines.append(" *        direction of travel points along nav +x. Roll and pitch are kept")
    lines.append(" *        as logged -- only yaw is rotated, so +z stays the gravity (depth)")
    lines.append(" *        axis. Z-down NED, matching this repo's convention -- no sign")
    lines.append(" *        conversion applied. Regenerate with:")
    lines.append(" *   python3 scripts/extract_survey_waypoints.py \\")
    lines.append(f" *       {source_path} --start {start:g} --duration {duration:g} \\")
    lines.append(" *       --output SamoaWaypoints.h")
    lines.append(" */")
    lines.append("#pragma once")
    lines.append("")
    lines.append("#include <gtsam/geometry/Point3.h>")
    lines.append("")
    lines.append("#include <vector>")
    lines.append("")
    lines.append("namespace imu_scenarios {")
    lines.append("")
    lines.append(
        f"/// A {duration:g} s window of a real AUV survey trajectory"
    )
    lines.append(
        f"/// ({os.path.basename(source_path)}), smoothed, resampled to points"
    )
    lines.append(
        f"/// uniform in time (dt = {dt:.6f} s), and moved into the inertial"
    )
    lines.append(
        "/// frame (starts at the origin, initial heading along nav +x,"
    )
    lines.append(
        "/// gravity axis untouched). See file header for provenance."
    )
    lines.append("inline std::vector<gtsam::Vector3> samoaSurveyWaypoints() {")
    lines.append("  return {")
    lines.extend(format_vectors(waypoints, 3))
    lines.append("  };")
    lines.append("}")
    lines.append("")
    lines.append(
        "/// Attitude (roll, pitch, yaw), in radians, matching"
    )
    lines.append(
        "/// samoaSurveyWaypoints() point-for-point. See file header for"
    )
    lines.append(
        "/// provenance."
    )
    lines.append("inline std::vector<gtsam::Vector3> samoaSurveyAttitudes() {")
    lines.append("  return {")
    lines.extend(format_vectors(attitudes, 5))
    lines.append("  };")
    lines.append("}")
    lines.append("")
    lines.append("}  // namespace imu_scenarios")
    lines.append("")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="path to vehicle_pose_est.csv")
    parser.add_argument("--start", type=float, default=590.0,
                        help="seconds from the first row's timestamp to "
                             "start the window at")
    parser.add_argument("--duration", type=float, default=300.0,
                        help="seconds of trajectory to keep from --start")
    parser.add_argument("--waypoints", type=int, default=1500,
                        help="number of output waypoints (default 1500 = 5 Hz "
                             "over the default 300 s)")
    parser.add_argument("--smooth-window", type=float, default=2.0,
                        help="moving-average window in seconds")
    parser.add_argument("--output",
                        default=os.path.join(os.path.dirname(__file__), "..",
                                            "SamoaWaypoints.h"),
                        help="output header path")
    args = parser.parse_args()

    timestamps, positions, quats = read_poses(args.input, args.start,
                                              args.duration)
    if len(positions) < 2:
        raise SystemExit("not enough rows in the requested window")

    roll, pitch, yaw = quats_to_euler(quats)
    values = np.hstack([np.asarray(positions),
                        np.column_stack([roll, pitch, yaw])])
    smoothed = smooth(timestamps, values, args.smooth_window)
    smoothed_positions, smoothed_angles = smoothed[:, :3], smoothed[:, 3:]

    waypoints, angles = resample_by_time(timestamps, smoothed_positions,
                                         smoothed_angles, args.waypoints,
                                         args.duration)
    waypoints, yaw, heading0 = to_inertial_frame(waypoints, angles[:, 2])
    attitudes = list(zip(angles[:, 0], angles[:, 1], yaw))

    dt = args.duration / (args.waypoints - 1)
    header = render_header(waypoints, attitudes, os.path.abspath(args.input),
                           args.start, args.duration, dt, heading0)
    with open(args.output, "w") as f:
        f.write(header)
    print(f"wrote {len(waypoints)} waypoints to {args.output}")


if __name__ == "__main__":
    main()
