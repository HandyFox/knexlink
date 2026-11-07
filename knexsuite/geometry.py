"""Small vector helpers. Coordinates are inches.

World axes:  X = left(-) / right(+)   Y = back(-) / front(+)   Z = down(-) / up(+)
"""
import math
import numpy as np

UP = np.array([0.0, 0.0, 1.0])
FWD = np.array([0.0, 1.0, 0.0])


def vec(v):
    return np.asarray(v, dtype=float).reshape(3)


def norm(v):
    return float(np.linalg.norm(v))


def unit(v, fallback=(1.0, 0.0, 0.0)):
    v = vec(v)
    n = np.linalg.norm(v)
    if n < 1e-12:
        return vec(fallback)
    return v / n


def axis_frame(a):
    """Reference directions for angles around axis `a`.

    Returns (u0, v0): u0 is "angle 0" (straight up if the axis is not vertical,
    otherwise straight forward); v0 = a x u0 is "angle +90" by the right-hand rule.
    """
    a = unit(a)
    ref = UP if abs(float(a @ UP)) < 0.95 else FWD
    u0 = ref - (ref @ a) * a
    u0 = unit(u0)
    v0 = np.cross(a, u0)
    return u0, v0


def angle_dir(a, deg):
    u0, v0 = axis_frame(a)
    r = math.radians(deg)
    return math.cos(r) * u0 + math.sin(r) * v0


def angle_of(a, d):
    """Angle (deg) of direction d around axis a, in the axis_frame convention."""
    u0, v0 = axis_frame(a)
    return math.degrees(math.atan2(float(d @ v0), float(d @ u0)))


def rot_matrix(a, theta):
    """3x3 right-hand rotation by theta (rad) about unit axis a."""
    a = unit(a)
    x, y, z = a
    c, s = math.cos(theta), math.sin(theta)
    C = 1.0 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def rotate_about(p, c, a, theta):
    return c + rot_matrix(a, theta) @ (vec(p) - c)


def rigid(Rm, t):
    M = np.eye(4)
    M[:3, :3] = Rm
    M[:3, 3] = t
    return M


def frame_along(p0, p1):
    """4x4 transform whose local +Z runs from p0 toward p1 (no scaling)."""
    p0 = vec(p0)
    z = unit(vec(p1) - p0, (0, 0, 1))
    helper = UP if abs(float(z @ UP)) < 0.9 else FWD
    x = unit(np.cross(helper, z))
    y = np.cross(z, x)
    M = np.eye(4)
    M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = x, y, z, p0
    return M


def seg_seg_dist(p1, q1, p2, q2):
    """Closest distance between segments p1-q1 and p2-q2 (Ericson)."""
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = float(d1 @ d1)
    e = float(d2 @ d2)
    f = float(d2 @ r)
    eps = 1e-12
    if a <= eps and e <= eps:
        return norm(p1 - p2)
    if a <= eps:
        s = 0.0
        t = min(max(f / e, 0.0), 1.0)
    else:
        c = float(d1 @ r)
        if e <= eps:
            t = 0.0
            s = min(max(-c / a, 0.0), 1.0)
        else:
            b = float(d1 @ d2)
            den = a * e - b * b
            s = min(max((b * f - c * e) / den, 0.0), 1.0) if den > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = min(max(-c / a, 0.0), 1.0)
            elif t > 1.0:
                t = 1.0
                s = min(max((b - c) / a, 0.0), 1.0)
    return norm((p1 + d1 * s) - (p2 + d2 * t))


def point_line_dist(p, c, a):
    v = vec(p) - c
    return norm(v - (v @ a) * a)


def skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)


def exp_rot(w):
    """Rotation matrix for rotation vector w (axis * angle)."""
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3) + skew(w)
    return rot_matrix(w / th, th)


def perp(a):
    """Any unit vector perpendicular to a."""
    a = unit(a)
    h = UP if abs(float(a @ UP)) < 0.9 else np.array([1.0, 0.0, 0.0])
    return unit(cross(a, h))


def frame_from(x_dir, z_dir):
    """3x3 with columns x, y, z where x = x_dir, z ~ z_dir (made perpendicular)."""
    x = unit(x_dir)
    z = vec(z_dir) - (vec(z_dir) @ x) * x
    if np.linalg.norm(z) < 1e-9:
        z = perp(x)
    z = unit(z)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def orthonormalize(R):
    u, _, vt = np.linalg.svd(R)
    M = u @ vt
    if np.linalg.det(M) < 0:
        u[:, -1] *= -1
        M = u @ vt
    return M


def mat4(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def cross(a, b):
    """Fast 3-vector cross product (np.cross is slow for single vectors)."""
    return np.array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])
