import cv2, numpy as np, json
res = json.load(open('sweep/sweep.json'))
out = []
for r in res:
    i = r['i']; pts = []
    for k in range(3):
        im = cv2.imread(f'sweep/f{i:02d}_{k}.jpg')
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        m = ((hsv[..., 2] < 95) & (hsv[..., 1] > 50)).astype(np.uint8)  # black steel, not shadow
        m[585:] = 0                                      # above the tape
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        n, lab, st, _ = cv2.connectedComponentsWithStats(m)
        if n < 2: pts.append(None); continue
        j = 1 + np.argmax(st[1:, cv2.CC_STAT_AREA])
        if st[j, cv2.CC_STAT_AREA] < 3000: pts.append(None); continue
        ys, xs = np.nonzero(lab == j)
        s = ys - xs                                       # bottom-left extreme
        c = np.argmax(s); pts.append((int(xs[c]), int(ys[c])))
        if k == 1:
            v = im.copy(); cv2.circle(v, pts[-1], 8, (0, 0, 255), 2)
            cv2.imwrite(f'sweep/c{i:02d}.jpg', v[400:620, max(0, pts[-1][0]-200):pts[-1][0]+200])
    ok = [p for p in pts if p]
    u = np.mean([p[0] for p in ok]) if ok else None
    v = np.mean([p[1] for p in ok]) if ok else None
    out.append(dict(r, u=u, v=v, pts=pts)); print(i, r['target'], round(r['pos'], 4), pts)
json.dump(out, open('sweep/corners.json', 'w'), indent=1)
