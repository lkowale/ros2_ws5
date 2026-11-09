"""Route ONE net that FreeRouting leaves open, on a 0.1 mm grid around the existing copper.

  env -i PATH=/usr/bin:/bin HOME=$HOME python3 route_net.py <board> <net> <ref> <pad> <out.ses> [x0 y0 x1 y1]

Finds the shortest two-layer path (orthogonal + 45 deg moves, vias cost extra) from the pad to any
copper already on <net>, keeping the Default net class rules (0.25 mm track, 0.2 mm clearance,
0.6/0.3 mm via) to all other copper, pads and the board edge. Zones are ignored (they are refilled
around the new track). Writes the route as a Specctra session that gen_pcb.import_ses() reads, so
`finish` can import it like a FreeRouting pass (SES_INCREMENTAL).
"""
import heapq
import math
import sys

import numpy as np
import pcbnew

TRACK_W, CLEAR, VIA_D = 0.25, 0.2, 0.6
G = 0.1          # grid (mm)
VIA_COST = 30    # grid steps


def main():
    board_path, net, ref, padnum, out = sys.argv[1:6]
    x0, y0, x1, y1 = [float(v) for v in sys.argv[6:10]] if len(sys.argv) > 9 else (0.0, 0.0, 120.0, 85.0)
    b = pcbnew.LoadBoard(board_path)
    mm = pcbnew.ToMM
    nx, ny = int(round((x1 - x0) / G)) + 1, int(round((y1 - y0) / G)) + 1
    xs = x0 + np.arange(nx) * G
    ys = y0 + np.arange(ny) * G
    X, Y = np.meshgrid(xs, ys)  # [iy, ix]
    blocked = np.zeros((2, ny, nx), bool)   # track centre may not go here (layer 0 = F, 1 = B)
    noVia = np.zeros((ny, nx), bool)        # via centre may not go here
    target = np.zeros((2, ny, nx), bool)    # copper of <net>

    def window(cx0, cy0, cx1, cy1, r):
        ix0 = max(0, int((min(cx0, cx1) - r - x0) / G) - 1); ix1 = min(nx, int((max(cx0, cx1) + r - x0) / G) + 2)
        iy0 = max(0, int((min(cy0, cy1) - r - y0) / G) - 1); iy1 = min(ny, int((max(cy0, cy1) + r - y0) / G) + 2)
        return slice(iy0, iy1), slice(ix0, ix1)

    def seg_dist(sl, ax, ay, bx, by):
        px, py = X[sl], Y[sl]
        dx, dy = bx - ax, by - ay
        L = dx * dx + dy * dy
        u = np.zeros_like(px) if L == 0 else np.clip(((px - ax) * dx + (py - ay) * dy) / L, 0, 1)
        return np.hypot(px - (ax + u * dx), py - (ay + u * dy))

    def mark(layers, ax, ay, bx, by, half, own):
        r_tr = half + CLEAR + TRACK_W / 2
        r_via = half + CLEAR + VIA_D / 2
        sl = window(ax, ay, bx, by, max(r_tr, r_via))
        d = seg_dist(sl, ax, ay, bx, by)
        for L in layers:
            if own:
                target[L][sl] |= d <= half
            else:
                blocked[L][sl] |= d < r_tr
        if not own:
            noVia[sl] |= d < r_via

    lay = {pcbnew.F_Cu: [0], pcbnew.B_Cu: [1]}
    for t in b.GetTracks():
        own = t.GetNetname() == net
        if t.Type() == pcbnew.PCB_VIA_T:
            p = t.GetPosition()
            mark([0, 1], mm(p.x), mm(p.y), mm(p.x), mm(p.y), mm(t.GetWidth()) / 2, own)
        else:
            s, e = t.GetStart(), t.GetEnd()
            mark(lay.get(t.GetLayer(), []), mm(s.x), mm(s.y), mm(e.x), mm(e.y), mm(t.GetWidth()) / 2, own)
    goal_xy = None
    for fp in b.GetFootprints():
        for p in fp.Pads():
            own = p.GetNetname() == net
            layers = [L for L, kl in ((0, pcbnew.F_Cu), (1, pcbnew.B_Cu)) if p.IsOnLayer(kl)]
            c = p.GetPosition()
            cx, cy = mm(c.x), mm(c.y)
            if fp.GetReference() == ref and p.GetNumber() == padnum:
                goal_xy = (cx, cy, layers)
                continue
            bb = p.GetBoundingBox()
            w, h = mm(bb.GetWidth()) / 2, mm(bb.GetHeight()) / 2
            if p.GetShape() == pcbnew.PAD_SHAPE_CIRCLE:
                mark(layers, cx, cy, cx, cy, w, own)
            else:  # rectangle-ish pad: approximate as a capsule along its long axis (conservative)
                if w >= h:
                    mark(layers, cx - (w - h), cy, cx + (w - h), cy, h, own)
                else:
                    mark(layers, cx, cy - (h - w), cx, cy + (h - w), w, own)
            if not own and p.GetDrillSize().x > 0:  # drilled hole blocks both layers
                mark([0, 1], cx, cy, cx, cy, mm(p.GetDrillSize().x) / 2, False)
    # board edge: keep the track 0.3 mm inside (EDGE rule) -> centre 0.425 mm
    e = 0.3 + TRACK_W / 2
    edge = (X < e) | (Y < e) | (X > 120 - e) | (Y > 85 - e)
    blocked[0] |= edge; blocked[1] |= edge; noVia |= edge | (X < 0.6) | (Y < 0.6) | (X > 119.4) | (Y > 84.4)
    # keepout rule areas (antenna)
    for z in b.Zones():
        if z.GetIsRuleArea():
            bb = z.GetBoundingBox()
            sl = window(mm(bb.GetLeft()), mm(bb.GetTop()), mm(bb.GetRight()), mm(bb.GetBottom()), 0.4)
            blocked[0][sl] = True; blocked[1][sl] = True; noVia[sl] = True

    gx, gy, glayers = goal_xy
    import os
    if os.environ.get('MAP'):   # ascii map around the start pad: # blocked, o own net, + no via
        for iy in range(max(0, int((gy - y0) / G) - 60), min(ny, int((gy - y0) / G) + 60), 2):
            row = ''
            for L in (0, 1):
                for ix in range(max(0, int((gx - x0) / G) - 80), min(nx, int((gx - x0) / G) + 80), 2):
                    row += 'o' if target[L, iy, ix] else '#' if blocked[L, iy, ix] else '+' if noVia[iy, ix] else '.'
                row += '   '
            print('%5.1f %s' % (y0 + iy * G, row))
    gi = (int(round((gy - y0) / G)), int(round((gx - x0) / G)))
    # copper already connected to the start pad is not a target: route to ANOTHER island of the net
    from scipy import ndimage
    lab = [ndimage.label(target[L], structure=np.ones((3, 3)))[0] for L in (0, 1)]
    off = lab[0].max() + 1
    parent = {}
    def find(a):
        while parent.get(a, a) != a:
            a = parent[a]
        return a
    both = target[0] & target[1]   # F and B copper on the same cell: via / THT pad (approximation)
    for a, c in set(zip(lab[0][both].tolist(), (lab[1][both] + off).tolist())):
        ra, rc = find(a), find(c)
        if ra != rc:
            parent[ra] = rc
    near = np.hypot(X - gx, Y - gy) < 0.6   # copper ending on the pad
    start = {find(v) for L in glayers for v in (lab[L][near] + (off if L else 0)).tolist() if v not in (0, off)}
    if start:
        for L in (0, 1):
            ids = np.unique(lab[L][target[L]])
            mine = [i for i in ids if find(int(i) + (off if L else 0)) in start]
            target[L] &= ~np.isin(lab[L], mine)
        print('start pad already joins %d island(s): routing to the rest of %s' % (len(start), net))
    # start: the goal pad (THT -> both layers); search outwards to any <net> copper
    pq, best, prev = [], {}, {}
    for L in glayers:
        s = (L, gi[0], gi[1])
        best[s] = 0.0
        heapq.heappush(pq, (0.0, 0.0, s))
    tgt_cells = np.argwhere(target)
    def h(iy, ix):  # cheap admissible-ish heuristic: distance to nearest target cell (sampled)
        return 0.0
    moves = [(0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),
             (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
    pad_r = 0.85  # inside the goal pad copper the clearance check is moot
    found = None
    while pq:
        f, g, (L, iy, ix) = heapq.heappop(pq)
        if g > best.get((L, iy, ix), 1e18):
            continue
        if target[L, iy, ix]:
            found = (L, iy, ix)
            break
        for dy, dx, c in moves:
            jy, jx = iy + dy, ix + dx
            if not (0 <= jy < ny and 0 <= jx < nx):
                continue
            in_pad = math.hypot(X[jy, jx] - gx, Y[jy, jx] - gy) < pad_r
            if blocked[L, jy, jx] and not in_pad and not target[L, jy, jx]:
                continue
            if dy and dx:  # no corner cutting
                if (blocked[L, iy, jx] and not target[L, iy, jx]) or (blocked[L, jy, ix] and not target[L, jy, ix]):
                    continue
            ng = g + c
            k = (L, jy, jx)
            if ng < best.get(k, 1e18):
                best[k] = ng; prev[k] = (L, iy, ix)
                heapq.heappush(pq, (ng, ng, k))
        if not noVia[iy, ix] and not blocked[1 - L, iy, ix]:
            ng = g + VIA_COST
            k = (1 - L, iy, ix)
            if ng < best.get(k, 1e18):
                best[k] = ng; prev[k] = (L, iy, ix)
                heapq.heappush(pq, (ng, ng, k))
    if not found:
        print('NO ROUTE'); sys.exit(1)
    path = [found]
    while path[-1] in prev:
        path.append(prev[path[-1]])
    path.reverse()  # goal pad -> net copper
    # split into per-layer polylines + vias, drop collinear points
    wires, vias, cur = [], [], [path[0]]
    for a, c in zip(path, path[1:]):
        if a[0] != c[0]:
            wires.append(cur); vias.append((a[1], a[2])); cur = [c]
        else:
            cur.append(c)
    wires.append(cur)
    def simplify(pts):
        out = [pts[0]]
        for i in range(1, len(pts) - 1):
            a, m, c = out[-1], pts[i], pts[i + 1]
            if (m[1] - a[1]) * (c[2] - m[2]) != (c[1] - m[1]) * (m[2] - a[2]):
                out.append(m)
        out.append(pts[-1])
        return out
    U = 10000  # session units: um * 10
    xy = lambda iy, ix: (int(round((x0 + ix * G) * U)), int(round(-(y0 + iy * G) * U)))
    lines = ['(session route_net', '  (routes', '    (resolution um 10)', '    (network_out', '      (net %s' % net]
    length = 0.0
    for wpts in wires:
        if len(wpts) < 2:
            continue
        sp = simplify(wpts)
        layer = 'F.Cu' if sp[0][0] == 0 else 'B.Cu'
        coords = '  '.join('%d %d' % xy(p[1], p[2]) for p in sp)
        lines.append('        (wire (path %s %d  %s))' % (layer, int(TRACK_W * U), coords))
        length += sum(math.hypot(q[1] - p[1], q[2] - p[2]) * G for p, q in zip(sp, sp[1:]))
    for iy, ix in vias:
        lines.append('        (via "Via[0-1]_600:300_um" %d %d)' % xy(iy, ix))
    lines += ['      )', '    )', '  )', ')']
    open(out, 'w').write('\n'.join(lines) + '\n')
    print('routed %s: %.1f mm, %d vias -> %s' % (net, length, len(vias), out))


if __name__ == '__main__':
    main()
