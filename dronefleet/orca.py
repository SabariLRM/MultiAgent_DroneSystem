"""Optimal Reciprocal Collision Avoidance in 3-D (van den Berg, Guy, Lin, Manocha 2011).

A pure-Python port of the velocity-obstacle half-spaces and the 3-D linear
program of the RVO2-3D library. Every neighbour ``j`` of agent ``i`` yields a
half-space of permitted velocities ``{v : (v - point) . normal >= 0}``; the new
velocity is the one closest to the preferred velocity that satisfies all of
them inside the speed sphere. If they cannot all be met, ``linear_program4``
picks the velocity that violates the worst one least.

Vectors are plain ``(x, y, z)`` tuples. The caller works in a space in which
the separation bubble is a sphere (see :mod:`dronefleet.flight`).
"""

from __future__ import annotations

import math

EPS = 1e-5
Plane = tuple  # (px, py, pz, nx, ny, nz): point on the plane and its unit normal


def _dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b) -> tuple:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(a) -> tuple:
    n = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
    return (a[0] / n, a[1] / n, a[2] / n) if n > 0 else (0.0, 0.0, 0.0)


def orca_plane(rel_pos, rel_vel, v_self, radius: float, tau: float, dt: float, share: float = 0.5) -> Plane:
    """The ORCA half-space for one neighbour.

    ``rel_pos = p_j - p_i``, ``rel_vel = v_i - v_j``; ``share`` is agent i's
    part of the avoidance (0.5 reciprocal, 1.0 if the neighbour will not react).
    """
    px, py, pz = rel_pos
    vx, vy, vz = rel_vel
    dist_sq = px * px + py * py + pz * pz
    r_sq = radius * radius
    if dist_sq > r_sq:
        inv_tau = 1.0 / tau
        wx, wy, wz = vx - inv_tau * px, vy - inv_tau * py, vz - inv_tau * pz
        w_sq = wx * wx + wy * wy + wz * wz
        dot = wx * px + wy * py + wz * pz
        if dot < 0.0 and dot * dot > r_sq * w_sq:
            # project on the cut-off sphere
            wl = math.sqrt(w_sq)
            nx, ny, nz = wx / wl, wy / wl, wz / wl
            k = radius * inv_tau - wl
        else:
            # project on the cone
            a = dist_sq
            b = px * vx + py * vy + pz * vz
            cx, cy, cz = _cross(rel_pos, rel_vel)
            c = (vx * vx + vy * vy + vz * vz) - (cx * cx + cy * cy + cz * cz) / (dist_sq - r_sq)
            disc = max(0.0, b * b - a * c)
            t = (b + math.sqrt(disc)) / a
            wx, wy, wz = vx - t * px, vy - t * py, vz - t * pz
            wl = math.sqrt(wx * wx + wy * wy + wz * wz)
            if wl < EPS:
                wx, wy, wz, wl = -px, -py, -pz, math.sqrt(dist_sq)
            nx, ny, nz = wx / wl, wy / wl, wz / wl
            k = radius * t - wl
    else:
        # already inside the radius: get out within one time step
        inv_dt = 1.0 / dt
        wx, wy, wz = vx - inv_dt * px, vy - inv_dt * py, vz - inv_dt * pz
        wl = math.sqrt(wx * wx + wy * wy + wz * wz)
        if wl < EPS:
            dl = math.sqrt(dist_sq) or 1.0
            wx, wy, wz, wl = -px / dl, -py / dl, -pz / dl, 1.0
        nx, ny, nz = wx / wl, wy / wl, wz / wl
        k = radius * inv_dt - wl
    return (v_self[0] + share * k * nx, v_self[1] + share * k * ny, v_self[2] + share * k * nz, nx, ny, nz)


# ----------------------------------------------------------------------------
# the linear programs of RVO2-3D
# ----------------------------------------------------------------------------

def _lp1(planes, plane_no: int, lp, ld, radius: float, opt, direction_opt: bool):
    """Optimise on the line (point ``lp``, direction ``ld``) against planes[:plane_no]."""
    dot = _dot(lp, ld)
    disc = dot * dot + radius * radius - _dot(lp, lp)
    if disc < 0.0:
        return None
    sd = math.sqrt(disc)
    t_left, t_right = -dot - sd, -dot + sd
    for i in range(plane_no):
        pl = planes[i]
        n = (pl[3], pl[4], pl[5])
        num = (pl[0] - lp[0]) * n[0] + (pl[1] - lp[1]) * n[1] + (pl[2] - lp[2]) * n[2]
        den = _dot(ld, n)
        if den * den <= EPS:
            if num > 0.0:
                return None
            continue
        t = num / den
        if den >= 0.0:
            t_left = max(t_left, t)
        else:
            t_right = min(t_right, t)
        if t_left > t_right:
            return None
    if direction_opt:
        t = t_right if _dot(opt, ld) > 0.0 else t_left
    else:
        t = _dot(ld, (opt[0] - lp[0], opt[1] - lp[1], opt[2] - lp[2]))
        t = t_left if t < t_left else t_right if t > t_right else t
    return (lp[0] + t * ld[0], lp[1] + t * ld[1], lp[2] + t * ld[2])


def _lp2(planes, plane_no: int, radius: float, opt, direction_opt: bool):
    pl = planes[plane_no]
    n = (pl[3], pl[4], pl[5])
    pt = (pl[0], pl[1], pl[2])
    plane_dist = _dot(pt, n)
    pd_sq = plane_dist * plane_dist
    r_sq = radius * radius
    if pd_sq > r_sq + EPS:
        return None
    plane_r_sq = r_sq - pd_sq
    centre = (plane_dist * n[0], plane_dist * n[1], plane_dist * n[2])
    if direction_opt:
        d = _dot(opt, n)
        po = (opt[0] - d * n[0], opt[1] - d * n[1], opt[2] - d * n[2])
        po_sq = _dot(po, po)
        if po_sq <= EPS:
            result = centre
        else:
            s = math.sqrt(plane_r_sq / po_sq)
            result = (centre[0] + s * po[0], centre[1] + s * po[1], centre[2] + s * po[2])
    else:
        d = (pt[0] - opt[0]) * n[0] + (pt[1] - opt[1]) * n[1] + (pt[2] - opt[2]) * n[2]
        result = (opt[0] + d * n[0], opt[1] + d * n[1], opt[2] + d * n[2])
        if _dot(result, result) > r_sq:
            pr = (result[0] - centre[0], result[1] - centre[1], result[2] - centre[2])
            s = math.sqrt(plane_r_sq / max(_dot(pr, pr), 1e-12))
            result = (centre[0] + s * pr[0], centre[1] + s * pr[1], centre[2] + s * pr[2])
    for i in range(plane_no):
        q = planes[i]
        qn = (q[3], q[4], q[5])
        if (q[0] - result[0]) * qn[0] + (q[1] - result[1]) * qn[1] + (q[2] - result[2]) * qn[2] > 0.0:
            cp = _cross(qn, n)
            if _dot(cp, cp) <= EPS:
                return None                     # parallel, and plane i invalidates this one
            ld = _norm(cp)
            ln = _cross(ld, n)
            den = _dot(ln, qn)
            if abs(den) <= 1e-12:
                return None
            s = ((q[0] - pt[0]) * qn[0] + (q[1] - pt[1]) * qn[1] + (q[2] - pt[2]) * qn[2]) / den
            lp = (pt[0] + s * ln[0], pt[1] + s * ln[1], pt[2] + s * ln[2])
            result = _lp1(planes, i, lp, ld, radius, opt, direction_opt)
            if result is None:
                return None
    return result


def _lp3(planes, radius: float, opt, direction_opt: bool):
    """Returns (index of the first plane that could not be met, result)."""
    if direction_opt:
        result = (opt[0] * radius, opt[1] * radius, opt[2] * radius)
    elif _dot(opt, opt) > radius * radius:
        o = _norm(opt)
        result = (o[0] * radius, o[1] * radius, o[2] * radius)
    else:
        result = opt
    for i, pl in enumerate(planes):
        if (pl[0] - result[0]) * pl[3] + (pl[1] - result[1]) * pl[4] + (pl[2] - result[2]) * pl[5] > 0.0:
            r = _lp2(planes, i, radius, opt, direction_opt)
            if r is None:
                return i, result
            result = r
    return len(planes), result


def _lp4(planes, n_hard: int, begin: int, radius: float, result):
    """Minimise the largest violation of planes[n_hard:], keeping planes[:n_hard] satisfied
    (as RVO2's 2-D version does for static obstacles)."""
    distance = 0.0
    for i in range(max(begin, n_hard), len(planes)):
        pi = planes[i]
        ni = (pi[3], pi[4], pi[5])
        if (pi[0] - result[0]) * ni[0] + (pi[1] - result[1]) * ni[1] + (pi[2] - result[2]) * ni[2] > distance:
            proj = list(planes[:n_hard])
            for j in range(n_hard, i):
                pj = planes[j]
                nj = (pj[3], pj[4], pj[5])
                cp = _cross(nj, ni)
                if _dot(cp, cp) <= EPS:
                    if _dot(ni, nj) > 0.0:
                        continue
                    point = (0.5 * (pi[0] + pj[0]), 0.5 * (pi[1] + pj[1]), 0.5 * (pi[2] + pj[2]))
                else:
                    ln = _cross(cp, ni)
                    den = _dot(ln, nj)
                    if abs(den) <= 1e-12:
                        continue
                    s = ((pj[0] - pi[0]) * nj[0] + (pj[1] - pi[1]) * nj[1] + (pj[2] - pi[2]) * nj[2]) / den
                    point = (pi[0] + s * ln[0], pi[1] + s * ln[1], pi[2] + s * ln[2])
                nn = _norm((nj[0] - ni[0], nj[1] - ni[1], nj[2] - ni[2]))
                proj.append((point[0], point[1], point[2], nn[0], nn[1], nn[2]))
            fail, r = _lp3(proj, radius, ni, True)
            if fail >= len(proj):
                result = r
            distance = (pi[0] - result[0]) * ni[0] + (pi[1] - result[1]) * ni[1] + (pi[2] - result[2]) * ni[2]
    return result


def solve(planes: list, pref, max_speed: float, n_hard: int = 0):
    """Velocity closest to ``pref`` inside the speed sphere that satisfies every half-space
    (or, if that is impossible, the one that minimises the largest violation of the others
    while keeping the first ``n_hard`` planes, e.g. static obstacles, satisfied)."""
    fail, result = _lp3(planes, max_speed, pref, False)
    if fail < len(planes):
        result = _lp4(planes, n_hard, fail, max_speed, result)
    return result
