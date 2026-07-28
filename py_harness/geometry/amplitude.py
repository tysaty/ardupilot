"""Amplitude-weave geometry (``TASK-004``).

The shipping ``modules/continuous_weave.lua`` law, ported to the harness: a
lateral sinusoid ``y(s) = A * sin(2*pi*s/lambda)`` laid along the normal to the
line from the aircraft to the (moving) target, with the amplitude limited so the
wave's curvature never exceeds ``1/R_min``.

Two things differ from a naive port and are deliberate:

* **Envelope.** ``continuous_weave.lua`` uses a linear distance envelope
  ``A_cap * q`` (its ``smoothstep`` is defined but unused). ``TASK-004`` uses
  **smoothstep** by default; ``envelope="linear"`` reproduces the shipping law
  for comparison (``ADR`` note in the task).
* **Plane anchoring.** The straight reference runs from the **aircraft's current
  position** toward the target (``u = 0`` is the aircraft), reproducing the
  achieved-amplitude behaviour of ``A-DEC-009`` rather than correcting it.

Stateless and Lua-transliterable (``DEC-2026-06-25-04``): plain ``math``, plain
floats and dicts, no ``numpy``, no closures over module state. The phase ``s`` is
passed in (the adapter derives it from the snapshot's ``t_s`` and speed), so the
module holds no accumulator. Frame is ``x = East``, ``y = North``.
"""

import math


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def smoothstep(q):
    """Smooth 0->1 ramp, ``3q^2 - 2q^3``. Defined in the Lua, used here."""
    return 3.0 * q * q - 2.0 * q * q * q


def desired_amplitude(distance_m, a_cap_m, d_start_m, d_full_m,
                      envelope="smoothstep"):
    """Distance-dependent desired weave amplitude.

    ``q`` ramps 0->1 as the distance to the target falls from ``d_start`` to
    ``d_full`` (``d_start > d_full``). ``smoothstep`` is the default (`TASK-004`);
    ``linear`` reproduces ``continuous_weave.lua``'s ``A_cap * q``.
    """
    if d_start_m <= d_full_m:
        raise ValueError("need d_start > d_full")
    q = clamp((d_start_m - distance_m) / (d_start_m - d_full_m), 0.0, 1.0)
    shaped = smoothstep(q) if envelope == "smoothstep" else q
    return a_cap_m * shaped


def curvature_limited_amplitude(lambda_m, r_min_m, eta):
    """Largest amplitude keeping ``curvature <= 1/R_min`` for the wave.

    For ``y = A sin(2*pi*s/lambda)`` the peak curvature is ``A*(2*pi/lambda)^2``,
    so ``A_max = eta * lambda^2 / (4*pi^2*R_min)``. ``eta`` in ``(0, 1]`` is the
    safety factor; ``eta = 1`` sits exactly at the curvature limit.

    Raises:
        ValueError: For a non-positive ``R_min`` or ``lambda`` (division), or an
            ``eta`` outside ``(0, 1]``.
    """
    if r_min_m <= 0.0:
        raise ValueError("r_min_m must be positive, got %r" % r_min_m)
    if lambda_m <= 0.0:
        raise ValueError("lambda_m must be positive, got %r" % lambda_m)
    if not (0.0 < eta <= 1.0):
        raise ValueError("eta must be in (0, 1], got %r" % eta)
    return eta * (lambda_m * lambda_m) / (4.0 * math.pi * math.pi * r_min_m)


def effective_amplitude(distance_m, lambda_m, r_min_m, a_cap_m, d_start_m,
                        d_full_m, eta, envelope="smoothstep"):
    """The commanded amplitude: the desired envelope, capped by curvature."""
    a_desired = desired_amplitude(distance_m, a_cap_m, d_start_m, d_full_m,
                                  envelope)
    return min(a_desired, curvature_limited_amplitude(lambda_m, r_min_m, eta))


def wave_derivatives(s_m, amplitude_m, lambda_m, phase_rad=0.0):
    """``(y, y', y'')`` of ``y = A sin(2*pi*s/lambda + phase)`` at arc length s.

    Raises:
        ValueError: For a non-positive ``lambda`` (the wavenumber divides by it).
    """
    if lambda_m <= 0.0:
        raise ValueError("lambda_m must be positive, got %r" % lambda_m)
    k = 2.0 * math.pi / lambda_m
    arg = k * s_m + phase_rad
    y = amplitude_m * math.sin(arg)
    y_prime = amplitude_m * k * math.cos(arg)
    y_double_prime = -amplitude_m * k * k * math.sin(arg)
    return y, y_prime, y_double_prime


def curvature(y_prime, y_double_prime):
    """Planar curvature ``|y''| / (1 + y'^2)^(3/2)``."""
    return abs(y_double_prime) / (1.0 + y_prime * y_prime) ** 1.5


def guidance(px, py, psi_i, tx, ty, s_m, lambda_m, r_min_m, a_cap_m,
             d_start_m, d_full_m, eta, look_ahead_m, envelope="smoothstep",
             phase_rad=0.0):
    """One guidance point for the amplitude weave, plane-anchored.

    The straight reference runs from the aircraft toward the target; a point one
    look-ahead along it is offset laterally by the curvature-limited wave. Frame
    ``x = East``, ``y = North``.

    Returns a dict ``{gx, gy, amplitude, curvature, curvature_limit}``.

    Raises:
        ValueError: When the aircraft is on the target (no weave direction).
    """
    dx, dy = tx - px, ty - py
    distance_m = math.hypot(dx, dy)
    if distance_m < 1e-6:
        raise ValueError("aircraft is on the target; no weave direction")

    # Unit vector toward the target and the left normal, in (East, North).
    ux, uy = dx / distance_m, dy / distance_m
    nx, ny = -uy, ux

    # Plane-anchored straight reference: u = 0 is the aircraft (A-DEC-009).
    u = min(look_ahead_m, distance_m)
    line_x = px + u * ux
    line_y = py + u * uy

    amp = effective_amplitude(distance_m, lambda_m, r_min_m, a_cap_m,
                              d_start_m, d_full_m, eta, envelope)
    y, y_prime, y_double_prime = wave_derivatives(s_m, amp, lambda_m, phase_rad)
    return {
        "gx": line_x + nx * y,
        "gy": line_y + ny * y,
        "amplitude": amp,
        "curvature": curvature(y_prime, y_double_prime),
        "curvature_limit": curvature_limited_amplitude(lambda_m, r_min_m, eta),
    }
