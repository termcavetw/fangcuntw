"""骨架變形 v2（給 JS 移植用的參考實作）

輸入：引擎 collect() 的原始骨架 strokes=[{t,p:[[x,y],...]}]
輸出：同樣結構的新控制點

做法
  1. 頂點：不同筆畫距離 < TOL 的端點合併成同一個頂點（接點不散）。
  2. 每一段線段給一個「目標向量」（方向＋長度），由各個變形參數決定。
  3. 限制：T 接點留在對方線段上、原本相交的兩段交點比例不變、白縫等寬。
  4. 最小平方法解出全部頂點。只取「參數造成的差」：最後位置＝原位置＋(解－零參數解)。
  5. 交錯保護：原本不相連、不相交的兩筆，變形後若交錯或間距掉到原本的 60% 以下
     且小於 60 單位，就把這兩筆的變形量減半重解，最多 5 輪。

參數（全部 0 ＝原樣，逐點不變）
  spread  撇捺開合（度，正＝張開）
  ext     主筆伸展（最長的橫放長，其餘橫縮短）
  rise    橫畫斜勢（度）
  conv    豎畫內收（度，正＝下端往中線收）
  octo    斜筆幾何化（0..1，撇捺點提的方向吸到 0/45/90 度）
  vsnap   陡撇拉直的角度範圍（度，配合 octo；捺不拉）
  dotv    點換成直立短豎（0..1）
  evengap 白縫等寬（0..1）
"""
import math
import numpy as np

TOL = 14.0
NEAR = 30.0
EG_OVERLAP = 0.8
EG_W = 2.0
PARAMS = ("spread", "ext", "rise", "conv", "octo", "vsnap", "dotv", "evengap")


def cls(v):
    dx, dy = v
    a = math.degrees(math.atan2(abs(dy), abs(dx)))
    if a < 25: return "H"
    if a > 65: return "V"
    d = v if dy < 0 else (-dx, -dy)
    return "P" if d[0] < 0 else "N"


def rot(v, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def seg_cross(A, B, C, D, lo=0.0, hi=1.0):
    r = B - A; s = D - C; den = r[0] * s[1] - r[1] * s[0]
    if abs(den) < 1e-9: return None
    u = ((C - A)[0] * s[1] - (C - A)[1] * s[0]) / den
    w = ((C - A)[0] * r[1] - (C - A)[1] * r[0]) / den
    if lo < u < hi and lo < w < hi: return u, w
    return None


def seg_dist(A, B, C, D):
    if seg_cross(A, B, C, D) is not None: return 0.0
    def pd(P, A, B):
        ab = B - A; L2 = ab @ ab
        u = 0.0 if L2 < 1e-12 else min(1.0, max(0.0, ((P - A) @ ab) / L2))
        return float(np.linalg.norm(A + u * ab - P))
    return min(pd(A, C, D), pd(B, C, D), pd(C, A, B), pd(D, A, B))


class Skeleton:
    """把筆畫拆成頂點與線段，並找出接點、T 接、交叉這些拓撲關係。只算一次。"""

    def __init__(self, strokes, tol=TOL):
        self.strokes = strokes
        pts = [(si, pi, np.array(p, float)) for si, st in enumerate(strokes) for pi, p in enumerate(st["p"])]
        n = len(pts); par = list(range(n))
        def find(i):
            while par[i] != i:
                par[i] = par[par[i]]; i = par[i]
            return i
        for i in range(n):
            for j in range(i):
                if pts[i][0] != pts[j][0] and np.linalg.norm(pts[i][2] - pts[j][2]) < tol:
                    par[find(i)] = find(j)
        roots = sorted({find(i) for i in range(n)})
        vid = {r: k for k, r in enumerate(roots)}
        self.var = [vid[find(i)] for i in range(n)]
        self.idx = {(si, pi): k for k, (si, pi, _) in enumerate(pts)}
        self.pts = pts
        self.nv = len(roots)
        P0 = np.zeros((self.nv, 2)); cnt = np.zeros(self.nv)
        for k, (_, _, p) in enumerate(pts):
            P0[self.var[k]] += p; cnt[self.var[k]] += 1
        self.P0 = P0 / cnt[:, None]
        allp = np.array([p for _, _, p in pts])
        self.lo, self.hi = allp.min(0), allp.max(0)
        self.c0 = (self.lo + self.hi) / 2
        self.hw = max(1.0, (self.hi[0] - self.lo[0]) / 2)
        # 線段
        self.edges = []
        for si, st in enumerate(strokes):
            for pi in range(len(st["p"]) - 1):
                a, b = self.idx[(si, pi)], self.idx[(si, pi + 1)]
                v = pts[b][2] - pts[a][2]
                if np.linalg.norm(v) > 1e-6:
                    self.edges.append((self.var[a], self.var[b], si, cls(v), v, (pts[a][2] + pts[b][2]) / 2))
        hl = {}
        for va, vb, si, c, v, m in self.edges:
            if c == "H": hl[si] = hl.get(si, 0) + np.linalg.norm(v)
        self.main = max(hl, key=hl.get) if len(hl) >= 2 else None
        # 拓撲關係：哪些筆畫彼此相連（共用頂點、T 接、交叉）
        self.related = set()
        sv = [set() for _ in strokes]
        for (si, pi), k in self.idx.items(): sv[si].add(self.var[k])
        for i in range(len(strokes)):
            for j in range(i):
                if sv[i] & sv[j]: self.related.add((j, i))
        self.tjoints = []
        for k, (si, pi, p) in enumerate(pts):
            for sj in range(len(strokes)):
                if sj == si: continue
                for a, b in self.segs(sj):
                    if self.var[a] == self.var[k] or self.var[b] == self.var[k]: continue
                    A, B = pts[a][2], pts[b][2]; ab = B - A; L2 = ab @ ab
                    if L2 < 1e-6: continue
                    u = ((p - A) @ ab) / L2
                    if .04 < u < .96 and np.linalg.norm(A + u * ab - p) < tol:
                        self.tjoints.append((self.var[k], self.var[a], self.var[b], u))
                        self.related.add((min(si, sj), max(si, sj)))
        self.crosses = []
        for si in range(len(strokes)):
            for sj in range(si):
                for a, b in self.segs(si):
                    for c, d in self.segs(sj):
                        r = seg_cross(pts[a][2], pts[b][2], pts[c][2], pts[d][2], .06, .94)
                        if r:
                            self.crosses.append((self.var[a], self.var[b], self.var[c], self.var[d], r[0], r[1]))
                            self.related.add((sj, si))
        # 相連的兩筆在接點處的夾角（共用頂點、T 接）。變形後被壓扁就是疊在一起。
        self.folds = []
        Q0 = self.orig_positions()
        inc = {}
        for si in range(len(strokes)):
            for a, b in self.segs(si):
                inc.setdefault(self.var[a], []).append((si, a, b))   # 從 a 出發
                inc.setdefault(self.var[b], []).append((si, b, a))   # 從 b 出發
        for v, lst in inc.items():
            for x in range(len(lst)):
                for y in range(x):
                    si, a, b = lst[x]; sj, c, d = lst[y]
                    if si == sj: continue
                    ang = self.angle(Q0[b] - Q0[a], Q0[d] - Q0[c])
                    if ang >= 25: self.folds.append(("v", si, a, b, sj, c, d, ang))
        for vk, va, vb, u in self.tjoints:
            for si, a, b in inc.get(vk, []):
                for sj in range(len(strokes)):
                    for c, d in self.segs(sj):
                        if {self.var[c], self.var[d]} == {va, vb} and sj != si:
                            ang = self.angle(Q0[b] - Q0[a], Q0[d] - Q0[c], line=True)
                            if ang >= 25: self.folds.append(("t", si, a, b, sj, c, d, ang))
        # 筆端靠近別筆的線段時，記下它在哪一側（變形後換邊＝筆端穿出去）
        self.ends = []
        for k, (si, pi, P) in enumerate(self.pts):
            if pi not in (0, len(strokes[si]["p"]) - 1): continue
            for sj in range(len(strokes)):
                if sj == si: continue
                for a, b in self.segs(sj):
                    if self.var[a] == self.var[k] or self.var[b] == self.var[k]: continue
                    A, B = Q0[a], Q0[b]; ab = B - A; L = np.linalg.norm(ab)
                    if L < 1e-6: continue
                    u = ((P - A) @ ab) / (L * L)
                    if not (-0.1 <= u <= 1.1): continue
                    sd = (ab[0] * (P - A)[1] - ab[1] * (P - A)[0]) / L
                    if 3 < abs(sd) < 120: self.ends.append((k, si, sj, a, b, sd))
        # 原本不相連的筆畫對，記下原本的最近距離
        self.d0 = {}
        for i in range(len(strokes)):
            for j in range(i):
                if (j, i) in self.related: continue
                d = self.pair_dist(self.orig_positions(), j, i)
                if d < NEAR: self.related.add((j, i)); continue   # 原本就幾乎相接（轉角沒合併到的接點），當成相連
                self.d0[(j, i)] = d

    @staticmethod
    def angle(u, v, line=False):
        nu, nv = np.linalg.norm(u), np.linalg.norm(v)
        if nu < 1e-6 or nv < 1e-6: return 180.0
        a = math.degrees(math.acos(max(-1.0, min(1.0, (u @ v) / (nu * nv)))))
        return min(a, 180 - a) if line else a

    def segs(self, si):
        return [(self.idx[(si, pi)], self.idx[(si, pi + 1)]) for pi in range(len(self.strokes[si]["p"]) - 1)]

    def orig_positions(self):
        return np.array([p for _, _, p in self.pts])

    def pair_dist(self, Q, i, j):
        best = 1e9
        for a, b in self.segs(i):
            for c, d in self.segs(j):
                best = min(best, seg_dist(Q[a], Q[b], Q[c], Q[d]))
        return best

    def pair_cross(self, Q, i, j):
        for a, b in self.segs(i):
            for c, d in self.segs(j):
                if seg_cross(Q[a], Q[b], Q[c], Q[d], .02, .98): return True
        return False

    # ---------------------------------------------------------------
    def solve(self, prm, alpha):
        """alpha[si]：該筆變形量的比例（交錯保護用）。回傳頂點位置。"""
        spread = prm.get("spread", 0); ext = prm.get("ext", 0); rise = prm.get("rise", 0)
        conv = prm.get("conv", 0); octo = prm.get("octo", 0); vsnap = prm.get("vsnap", 0)
        dotv = prm.get("dotv", 0); evengap = prm.get("evengap", 0)
        rows, rhs = [], []
        def add(coef, b, w):
            r = np.zeros(self.nv)
            for k, cf in coef: r[k] += cf
            rows.append(r * w); rhs.append(np.asarray(b, float) * w)
        for va, vb, si, c, v, m in self.edges:
            t = v.copy(); st_t = self.strokes[si].get("t")
            if dotv and st_t == "dian":
                L = np.linalg.norm(v); tv = np.array([0.0, -L if v[1] <= 0 else L])
                t = v + (tv - v) * dotv
            elif (octo or vsnap) and st_t in ("pie", "na", "dian", "ti"):
                # 斜筆幾何化：撇捺之所以是撇捺，就是因為它們是斜的。只把 25°–65° 的
                # 斜筆吸到 45°；太平、太陡的保留原角度（不吸成橫或豎，否則禾變未、火變兩豎）。
                # 例外：很陡的撇（離垂直 vsnap 度以內）照設定拉成直豎。
                a = math.degrees(math.atan2(v[1], v[0]))
                h = abs(a) % 180; h = min(h, 180 - h)          # 與水平線的夾角 0..90
                ax = a; k = octo
                if 25 <= h <= 65:
                    ax = math.copysign(45, a) if abs(a) <= 90 else math.copysign(135, a)
                if vsnap and st_t == "pie" and abs(abs(a) - 90) < vsnap: ax = 90 if a > 0 else -90; k = 1
                t = rot(v, (ax - a) * k)
            elif c == "H":
                t = rot(t, rise)
                if self.main is not None: t = t * ((1 + ext) if si == self.main else (1 - ext / 3))
            elif c == "V":
                t = rot(t, -conv * (m[0] - self.c0[0]) / self.hw)
            else:
                d = v if v[1] < 0 else -v
                t = rot(t, -spread if d[0] < 0 else spread)
            t = v + (t - v) * alpha[si]
            add([(vb, 1), (va, -1)], t, 1.0)
        for vk, va, vb, u in self.tjoints:
            add([(vk, 1), (va, -(1 - u)), (vb, -u)], [0, 0], 8.0)
        for va, vb, vc, vd, u, w in self.crosses:
            add([(va, 1 - u), (vb, u), (vc, -(1 - w)), (vd, -w)], [0, 0], 8.0)
        if evengap:
            P0 = self.P0
            for cl, axis in (("H", 1), ("V", 0)):
                segs_ = [(va, vb, (P0[va] + P0[vb]) / 2, min(P0[va][1 - axis], P0[vb][1 - axis]),
                          max(P0[va][1 - axis], P0[vb][1 - axis]))
                         for va, vb, si, c, v, m in self.edges if c == cl and np.linalg.norm(v) > 60]
                segs_.sort(key=lambda z: z[2][axis])
                used = [False] * len(segs_)
                for i in range(len(segs_)):
                    if used[i]: continue
                    chain = [i]
                    for j in range(i + 1, len(segs_)):
                        a_, b_ = segs_[chain[-1]], segs_[j]
                        ov = min(a_[4], b_[4]) - max(a_[3], b_[3])
                        if ov > EG_OVERLAP * min(a_[4] - a_[3], b_[4] - b_[3]) and 30 < b_[2][axis] - a_[2][axis] < 260:
                            chain.append(j)
                    if len(chain) < 3: continue
                    for k in chain: used[k] = True
                    pos = [segs_[k][2][axis] for k in chain]; g = (pos[-1] - pos[0]) / (len(chain) - 1)
                    for k0, k1 in zip(chain, chain[1:]):
                        a_, b_ = segs_[k0], segs_[k1]; cur = b_[2][axis] - a_[2][axis]
                        tgt = cur + (g - cur) * evengap
                        vv = np.zeros(2); vv[axis] = tgt; vv[1 - axis] = (b_[2] - a_[2])[1 - axis]
                        add([(b_[0], .5), (b_[1], .5), (a_[0], -.5), (a_[1], -.5)], vv, EG_W)
        for k in range(self.nv): add([(k, 1)], self.P0[k], 0.03)
        A = np.array(rows); B = np.array(rhs)
        X, *_ = np.linalg.lstsq(A, B, rcond=None)
        # 放回字面框（主筆伸展時以主筆以外的範圍為準，主筆可以伸出去）
        keep = np.ones(self.nv, bool); ol_mask = np.ones(len(self.pts), bool)
        if self.main is not None and ext > 0:
            for k, (si, pi, _) in enumerate(self.pts):
                if si == self.main: keep[self.var[k]] = False; ol_mask[k] = False
            if keep.sum() < 2: keep[:] = True; ol_mask[:] = True
        ol = self.orig_positions()[ol_mask]
        olo, ohi = ol.min(0), ol.max(0); body = X[keep]; nl, nh = body.min(0), body.max(0)
        s = min((ohi[0] - olo[0]) / max(1, nh[0] - nl[0]), (ohi[1] - olo[1]) / max(1, nh[1] - nl[1]))
        X = (X - (nl + nh) / 2) * s + (olo + ohi) / 2
        fl, fh = X.min(0), X.max(0); fit = min(1.0, 960 / max(1, fh[0] - fl[0]), 960 / max(1, fh[1] - fl[1]))
        if fit < 1: X = (X - (fl + fh) / 2) * fit + (fl + fh) / 2
        fl, fh = X.min(0), X.max(0); X += np.maximum(0, 20 - fl) - np.maximum(0, fh - 980)
        return X

    def positions(self, prm, alpha, Xb):
        """每個原始控制點的新位置＝原位置＋(解－零參數解)。"""
        X = self.solve(prm, alpha)
        return np.array([p + X[self.var[k]] - Xb[self.var[k]] for k, (_, _, p) in enumerate(self.pts)])


def poke(S, Q, i, j, m):
    """兩筆真的穿過對方：交點離兩條線段的端點都超過 m（墨的一半厚），端點貼著不算。"""
    for a, b in S.segs(i):
        for c, d in S.segs(j):
            A, B, C, D = Q[a], Q[b], Q[c], Q[d]
            r = B - A; s_ = D - C; den = r[0] * s_[1] - r[1] * s_[0]
            if abs(den) < 1e-9: continue
            ca = C - A
            u = (ca[0] * s_[1] - ca[1] * s_[0]) / den; w = (ca[0] * r[1] - ca[1] * r[0]) / den
            if not (0 < u < 1 and 0 < w < 1): continue
            La, Lb = np.linalg.norm(r), np.linalg.norm(s_)
            if min(u, 1 - u) * La > m and min(w, 1 - w) * Lb > m: return True
    return False


def violations(S, Q, clear=60.0):
    """clear＝筆畫寬度（EM）。兩條中心線距離小於它，墨就黏在一起。
       判定為撞：交錯；或原本墨沒碰到、變形後碰到；或原本已碰到、又被擠近三成以上。"""
    bad = []
    for kind, si, a, b, sj, c, d, ang0 in S.folds:
        ang = S.angle(Q[b] - Q[a], Q[d] - Q[c], line=(kind == "t"))
        if ang < 12 and ang < 0.5 * ang0: bad.append((min(si, sj), max(si, sj)))
    for k, si, sj, a, b, s0 in S.ends:
        if abs(s0) >= clear: continue
        A, B, P = Q[a], Q[b], Q[k]; ab = B - A; L = np.linalg.norm(ab)
        if L < 1e-6: continue
        u = ((P - A) @ ab) / (L * L)
        if not (-0.1 <= u <= 1.1): continue
        s1 = (ab[0] * (P - A)[1] - ab[1] * (P - A)[0]) / L
        if s1 * s0 < 0 and abs(s1) > 0.06 * clear: bad.append((min(si, sj), max(si, sj)))
    # 相連／相貼的筆：原本沒互相穿過，變形後穿過去（火、以、夂、辶 的長撇拉直後切過鄰筆）
    key = ("rel0", clear)
    if key not in S.__dict__:
        Q0 = S.orig_positions()
        S.__dict__[key] = [(i, j) for (i, j) in sorted(S.related) if not poke(S, Q0, i, j, 0.5 * clear)]
    for i, j in S.__dict__[key]:
        if poke(S, Q, i, j, 0.5 * clear): bad.append((i, j))
    for (i, j), d0 in S.d0.items():
        if S.pair_cross(Q, i, j):
            bad.append((i, j)); continue
        d1 = S.pair_dist(Q, i, j)
        if (d0 >= clear and d1 < clear) or (d0 < clear and d1 < 0.7 * d0): bad.append((i, j))
    return bad


def deform(strokes, guard=True, report=None, clear=60.0, **prm):
    if not any(prm.get(k, 0) for k in PARAMS):
        return [[list(map(float, q)) for q in st["p"]] for st in strokes]
    S = Skeleton(strokes)
    alpha = np.ones(len(strokes))
    Xb = S.solve({}, alpha)
    Q = S.positions(prm, alpha, Xb)
    lam = 1.0; rounds = 0
    if guard:
        # 第一段：只減弱撞到的那幾筆
        for rounds in range(1, 4):
            bad = violations(S, Q, clear)
            if not bad: break
            for i, j in bad:
                alpha[i] *= 0.5; alpha[j] *= 0.5
            Q = S.positions(prm, alpha, Xb)
        # 第二段：還撞就把整個字的變形量一起縮小，二分法找不撞的最大比例
        if violations(S, Q, clear):
            lo, hi = 0.0, 1.0; Qlo = S.orig_positions()
            for _ in range(6):
                mid = (lo + hi) / 2
                pm = dict(prm, evengap=prm.get("evengap", 0) * mid)   # 白縫等寬不屬於任何一筆，要一起縮
                Qm = S.positions(pm, alpha * mid, Xb)
                if violations(S, Qm, clear): hi = mid
                else: lo, Qlo = mid, Qm
            lam = lo; Q = Qlo
    if report is not None:
        report.update(rounds=rounds, lam=round(lam, 3), softened=int((alpha < 1).sum()),
                      residual=len(violations(S, Q, clear)) if guard else 0)
    out = []
    for si, st in enumerate(strokes):
        out.append([[round(float(Q[S.idx[(si, pi)]][0]), 1), round(float(Q[S.idx[(si, pi)]][1]), 1)]
                    for pi in range(len(st["p"]))])
    return out
