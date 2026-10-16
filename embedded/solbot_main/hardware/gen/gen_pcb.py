"""Generate solbot_main.kicad_pcb from netlist.json (written by gen_sch.py).

Phases:
  place   -> board outline, footprints, locked high-current copper, keepouts,
             exports solbot_main.dsn for FreeRouting
  finish  -> imports solbot_main.ses, adds GND pours + stitching, fills zones,
             writes DRC report

Run with a clean environment (KiCad python), e.g.
  env -i PATH=/usr/bin:/bin HOME=$HOME python3 gen_pcb.py place
"""
import json
import os
import shutil
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.abspath(os.path.join(HERE, '..'))
PCB = os.path.join(OUT, 'solbot_main.kicad_pcb')
DSN = os.path.join(OUT, 'solbot_main.dsn')
SES = os.path.join(OUT, 'solbot_main.ses')
PLACED = os.path.join(HERE, 'placed.kicad_pcb')   # unrouted snapshot
FPDIRS = ['/usr/share/kicad/footprints', os.path.join(OUT, 'lib')]

BW, BH = 190.0, 155.0
COL_X = {'BAT1': 14, 'BAT2': 40, 'BAT3': 66, 'BAT4': 92, 'PV': 118}
BUS_Y = 66.0


def mm(v):
    return pcbnew.FromMM(v)


def V(x, y):
    return pcbnew.VECTOR2I(mm(x), mm(y))


def load_fp(fpid):
    lib, name = fpid.split(':')
    for d in FPDIRS:
        p = os.path.join(d, lib + '.pretty')
        if os.path.isdir(p) and os.path.exists(os.path.join(p, name + '.kicad_mod')):
            fp = pcbnew.FootprintLoad(p, name)
            fp.SetFPID(pcbnew.LIB_ID(lib, name))
            return fp
    raise FileNotFoundError(fpid)


class Gen:
    def __init__(self):
        self.board = pcbnew.BOARD()
        self.nets = {}
        self.fps = {}
        self.parts = json.load(open(os.path.join(HERE, 'netlist.json')))
        self.via_exclude = [(152, BUS_Y), (168, BUS_Y)]

    def net(self, name):
        if name not in self.nets:
            n = pcbnew.NETINFO_ITEM(self.board, name)
            self.board.Add(n)
            self.nets[name] = n
        return self.nets[name]

    # ---------------------------------------------------------------- parts
    def add_parts(self):
        for p in self.parts:
            fp = load_fp(p['footprint'])
            fp.SetReference(p['ref'])
            fp.SetValue(p['value'][:40])
            fp.SetPath(pcbnew.KIID_PATH(p['path']))
            for pad in fp.Pads():
                num = pad.GetNumber()
                if num in p['pads']:
                    pad.SetNet(self.net(p['pads'][num]))
            self.board.Add(fp)
            self.fps[p['ref']] = (fp, p)

    def by(self, pred):
        return [r for r, (fp, p) in self.fps.items() if pred(p)]

    def place(self, ref, x, y, rot=0):
        fp = self.fps[ref][0]
        fp.SetOrientationDegrees(rot)
        fp.SetPosition(V(x, y))
        self.fps[ref][1]['_placed'] = True

    def pad_pos(self, ref, num, largest=True):
        fp = self.fps[ref][0]
        pads = [p for p in fp.Pads() if p.GetNumber() == str(num)]
        if largest:
            pads.sort(key=lambda p: -(p.GetSize().x * p.GetSize().y))
        pp = pads[0].GetPosition()
        return pcbnew.ToMM(pp.x), pcbnew.ToMM(pp.y)

    def bbox(self, ref):
        fp = self.fps[ref][0]
        bb = fp.GetBoundingBox(False, False)
        return (pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()),
                pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom()))

    def pack(self, refs, x0, y0, x1, y1, gap=0.8):
        """Row-pack footprints into a rectangle (mm)."""
        x, y, row_h = x0, y0, 0.0
        for ref in refs:
            fp = self.fps[ref][0]
            fp.SetOrientationDegrees(0)
            fp.SetPosition(V(0, 0))
            l, t, r, b = self.bbox(ref)
            w, h = r - l, b - t
            if x > x0 and x + w > x1:
                x, y = x0, y + row_h + gap
                row_h = 0.0
            fp.SetPosition(V(x - l, y - t))
            self.fps[ref][1]['_placed'] = True
            x += w + gap
            row_h = max(row_h, h)
        if y + row_h > y1 + 0.01:
            print('WARNING: region (%.0f,%.0f)-(%.0f,%.0f) overflows by %.1f mm (%s...)'
                  % (x0, y0, x1, y1, y + row_h - y1, refs[0]))

    # --------------------------------------------------------------- copper
    def track(self, net, pts, width, layer=pcbnew.F_Cu, lock=True):
        for (xa, ya), (xb, yb) in zip(pts, pts[1:]):
            t = pcbnew.PCB_TRACK(self.board)
            t.SetStart(V(xa, ya))
            t.SetEnd(V(xb, yb))
            t.SetWidth(mm(width))
            t.SetLayer(layer)
            t.SetNet(self.net(net))
            t.SetLocked(lock)
            self.board.Add(t)

    def via(self, net, x, y, dia=1.0, drill=0.5, lock=True):
        v = pcbnew.PCB_VIA(self.board)
        v.SetPosition(V(x, y))
        v.SetWidth(mm(dia))
        v.SetDrill(mm(drill))
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNet(self.net(net))
        v.SetLocked(lock)
        self.board.Add(v)

    def both(self, net, pts, width, vias_every=None):
        self.track(net, pts, width, pcbnew.F_Cu)
        self.track(net, pts, width, pcbnew.B_Cu)
        if vias_every:
            for (xa, ya), (xb, yb) in zip(pts, pts[1:]):
                L = ((xb - xa) ** 2 + (yb - ya) ** 2) ** 0.5
                n = max(1, int(L / vias_every))
                for i in range(n + 1):
                    f = i / n
                    x, y = xa + (xb - xa) * f, ya + (yb - ya) * f
                    if all((x - hx) ** 2 + (y - hy) ** 2 > 6.0 ** 2 for hx, hy in self.via_exclude):
                        self.via(net, x, y)

    def rect_zone(self, x0, y0, x1, y1, layer, net=None, keepout=False, prio=0):
        z = pcbnew.ZONE(self.board)
        z.SetLayer(layer)
        if net:
            z.SetNet(self.net(net))
        o = z.Outline()
        o.NewOutline()
        for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
            o.Append(mm(x), mm(y))
        if keepout:
            z.SetIsRuleArea(True)
            z.SetDoNotAllowCopperPour(True)
            z.SetDoNotAllowTracks(True)
            z.SetDoNotAllowVias(True)
            z.SetDoNotAllowPads(False)
            z.SetDoNotAllowFootprints(False)
        else:
            z.SetAssignedPriority(prio)
            z.SetMinThickness(mm(0.25))
            z.SetLocalClearance(mm(0.5))
            z.SetThermalReliefGap(mm(0.5))
            z.SetThermalReliefSpokeWidth(mm(0.6))
            z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
        self.board.Add(z)
        return z

    def outline(self):
        pts = [(0, 0), (BW, 0), (BW, BH), (0, BH), (0, 0)]
        for (xa, ya), (xb, yb) in zip(pts, pts[1:]):
            s = pcbnew.PCB_SHAPE(self.board)
            s.SetShape(pcbnew.SHAPE_T_SEGMENT)
            s.SetStart(V(xa, ya))
            s.SetEnd(V(xb, yb))
            s.SetLayer(pcbnew.Edge_Cuts)
            s.SetWidth(mm(0.1))
            self.board.Add(s)

    def text(self, s, x, y, size=1.5, layer=pcbnew.F_SilkS):
        t = pcbnew.PCB_TEXT(self.board)
        t.SetText(s)
        t.SetPosition(V(x, y))
        t.SetLayer(layer)
        t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
        t.SetTextThickness(mm(size * 0.15))
        self.board.Add(t)

    # ------------------------------------------------------------- layout
    def layout(self):
        refs_in = lambda sheet, block: [r for r, (fp, p) in self.fps.items()
                                        if p['sheet'] == sheet and p['block'] == block]
        val = lambda r: self.fps[r][1]['value']
        lib = lambda r: self.fps[r][1]['lib_id']

        # ---- power channels
        for ch, cx in COL_X.items():
            refs = refs_in('power_path', ch)
            lug = [r for r in refs if lib(r) == 'Mechanical:MountingHole_Pad'][0]
            shunt = [r for r in refs if lib(r) == 'Device:R_Shunt'][0]
            qa, qb = [r for r in refs if lib(r) == 'Device:Q_PMOS_GDS']
            self.place(lug, cx, 8)
            self.place(shunt, cx, 19, 270)
            self.place(qa, cx, 33, 90)
            self.place(qb, cx + 5.08, 56, 270)
            rest = [r for r in refs if r not in (lug, shunt, qa, qb)]
            # biggest first packs better
            rest.sort(key=lambda r: -self._area(r))
            x1 = cx + 30 if ch == 'PV' else cx + 20
            self.pack(rest, cx + 6.5, 11, x1, 45, gap=1.2)
            self.text(ch, cx - 3, 2.2, 2.0)
            # high-current copper
            n_in, n_sh, n_com = ch + '_IN', ch + '_SH', ch + '_COM'
            p1 = self.pad_pos(shunt, 1)
            p4 = self.pad_pos(shunt, 4)
            self.both(n_in, [(cx, 8), (p1[0], 12.0)], 4.0)
            self.track(n_in, [(p1[0], 12.0), p1], 2.0)
            ta = self.pad_pos(qa, 2)
            self.track(n_sh, [p4, (p4[0], 26.5)], 2.0)
            self.track(n_sh, [(p4[0], 26.5), ta], 4.0)
            sa = self.pad_pos(qa, 3)
            sb = self.pad_pos(qb, 3)
            self.track(n_com, [sa, sb], 3.0)
            tb = self.pad_pos(qb, 2)
            self.both('BUS+', [tb, (tb[0], BUS_Y)], 8.0)
            for dy in (-2.5, 2.5):
                self.via('BUS+', tb[0] + 2.5, tb[1] + 3 + dy)
                self.via('BUS+', tb[0] - 2.5, tb[1] + 3 + dy)

        # ---- bus
        self.both('BUS+', [(6, BUS_Y), (172, BUS_Y)], 8.0, vias_every=6)
        refs = refs_in('power_path', 'BUS')
        lugs = [r for r in refs if lib(r) == 'Mechanical:MountingHole_Pad']
        bus_lugs = [r for r in lugs if val(r).startswith('BUS')]
        gnd_lugs = [r for r in lugs if val(r).startswith('GND')]
        self.place(bus_lugs[0], 152, BUS_Y)
        self.place(bus_lugs[1], 168, BUS_Y)
        for i, r in enumerate(gnd_lugs):
            self.place(r, 182, 14 + 16 * i)
        caps = [r for r in refs if lib(r) == 'Device:C_Polarized']
        self.place(caps[0], 140, 53)
        self.place(caps[1], 158, 53)
        for c in caps:
            p = self.pad_pos(c, 1)
            self.both('BUS+', [p, (p[0], BUS_Y)], 2.5)
        rest = [r for r in refs if r not in lugs + caps]
        rest.sort(key=lambda r: -self._area(r))
        self.pack(rest, 150, 4, 175, 44, gap=1.6)
        self.text('BUS+', 158, 72, 2.0)
        self.text('GND', 178, 2.2, 2.0)

        # ---- NTC near FETs
        th = [r for r, (fp, p) in self.fps.items() if p['lib_id'] == 'Device:Thermistor_NTC'][0]
        self.place(th, 56, 47, 90)

        # ---- logic supply / sensing
        self.pack(sorted(refs_in('supply_sense', 'BUCK'), key=lambda r: -self._area(r)), 8, 76, 52, 107, gap=2.0)
        exp_hdr = [r for r in refs_in('supply_sense', 'EXP') if lib(r).startswith('Connector')][0]
        self.place(exp_hdr, 3.5, 110)
        i2c_hdr = [r for r in refs_in('supply_sense', 'I2C') if lib(r).startswith('Connector')][0]
        rest = (refs_in('supply_sense', 'LDO') + refs_in('supply_sense', 'PADC') +
                refs_in('supply_sense', 'EXP') + refs_in('supply_sense', 'I2C'))
        rest = [r for r in rest if r not in (exp_hdr, i2c_hdr, th)]
        rest.sort(key=lambda r: -self._area(r))
        self.pack(rest, 8, 109, 54, 141, gap=2.2)

        # ---- MCU
        esp = [r for r in refs_in('mcu', 'ESP') if lib(r) == 'solbot:ESP32-S3-DevKitC-1'][0]
        self.place(esp, 84, 139, 180)
        self.pack(sorted([r for r in refs_in('mcu', 'ESP') if r != esp], key=lambda r: -self._area(r)),
                  57, 75.5, 89, 83.5)
        self.rect_zone(55, 141.5, 87, BH, pcbnew.F_Cu, keepout=True)  # antenna keepout
        self.text('ESP32-S3-DevKitC-1  (antenna this end)', 57, 152.5, 1.2)
        for i, r in enumerate(refs_in('mcu', 'MECH')):
            self.place(r, *[(4, 4), (186, 4), (4, 151), (186, 138)][i])

        # ---- connectors along edges
        estop = [r for r in refs_in('mcu', 'ESTOP') if lib(r).startswith('Connector')][0]
        imu = [r for r in refs_in('mcu', 'IMU') if lib(r).startswith('Connector')][0]
        self.edge_place(estop, 12, BH)
        self.edge_place(i2c_hdr, 27, BH)
        self.edge_place(imu, 170, BH)
        act_conn, act_small = [], []
        for n in range(1, 7):
            for r in refs_in('actuators', 'A%d' % n):
                (act_conn if lib(r).startswith('Connector') else act_small).append(r)
        ibt = [r for r in act_conn if val(r).startswith('IBT')]
        pos = [r for r in act_conn if val(r).startswith('POS')]
        for i, r in enumerate(ibt):
            self.place(r, 169.5, 78 + 7 * i, 90)
            self.text('A%d' % (i + 1), 164.5, 77.3 + 7 * i, 1.2)
        for i, r in enumerate(pos):
            self.edge_place(r, 94 + 12 * i, BH)

        # ---- actuator electronics
        small = (refs_in('actuators', 'BUF') + refs_in('actuators', 'POSREF') + refs_in('actuators', 'AADC')
                 + act_small + [r for r in refs_in('mcu', 'ESTOP') if r != estop]
                 + [r for r in refs_in('mcu', 'IMU') if r != imu])
        small.sort(key=lambda r: -self._area(r))
        self.pack(small, 90, 76, 165, 140, gap=2.6)

        missing = [r for r, (fp, p) in self.fps.items() if not p.get('_placed')]
        if missing:
            print('UNPLACED:', missing)

    def edge_place(self, ref, x, edge_y):
        """Place connector so its body sits against the bottom board edge."""
        fp = self.fps[ref][0]
        fp.SetOrientationDegrees(0)
        fp.SetPosition(V(x, edge_y - 10))
        l, t, r, b = self.bbox(ref)
        fp.SetPosition(V(x, edge_y - 10 - (b - (edge_y - 0.5))))
        self.fps[ref][1]['_placed'] = True

    def _area(self, ref):
        fp = self.fps[ref][0]
        fp.SetOrientationDegrees(0)
        l, t, r, b = self.bbox(ref)
        return (r - l) * (b - t)

    def design_rules(self):
        ds = self.board.GetDesignSettings()
        ds.SetCopperLayerCount(2)
        ds.m_TrackMinWidth = mm(0.2)
        ds.m_MinClearance = mm(0.2)
        ds.m_ViasMinSize = mm(0.6)
        ds.m_MinThroughDrill = mm(0.2)
        ds.m_CopperEdgeClearance = mm(0.3)   # JLCPCB min for routed outline; FreeRouting keeps ~0.3


def phase_place():
    g = Gen()
    g.design_rules()
    g.add_parts()
    g.outline()
    g.layout()
    g.text('solbot_main v4  ESP32-S3 controller  2026-10-15', 120, 148, 2.0, pcbnew.B_SilkS)
    pcbnew.SaveBoard(PCB, g.board)
    shutil.copy(PCB, PLACED)
    # reload so project net classes are applied, then export DSN
    b = pcbnew.LoadBoard(PCB)
    ok = pcbnew.ExportSpecctraDSN(b, DSN)
    fix_dsn_classes()
    print('saved', PCB, 'DSN export', ok)


HV_SUFFIX = ('_IN', '_SH', '_COM', '_G', '_VM', '_SNP', '_SNN')
HV_NETS = {'BUS+', 'VAUX', 'BUCK_SW', 'BUCK_BST', 'BUS_VM'}
POWER_NETS = {'GND', '+5V', '5V_ESP', '+3V3A', '+3V3_POT'}


def net_class(n):
    if n in HV_NETS or (n.startswith(('BAT', 'PV_')) and n.endswith(HV_SUFFIX)):
        return 'HV'
    if n in POWER_NETS:
        return 'Power'
    return 'kicad_default'


def fix_dsn_classes():
    """KiCad's DSN export only emits the default class; split nets into HV/Power/default."""
    import re
    txt = open(DSN).read()
    m = re.search(r'\(class kicad_default "" (.*?)\(circuit\s*\(use_via ([^)]+)\)\s*\)\s*\(rule.*?\)\s*\)\s*\)',
                  txt, re.S)
    nets = re.findall(r'"[^"]*"|\S+', m.group(1))
    via = m.group(2)
    groups = {}
    for n in nets:
        groups.setdefault(net_class(n.strip('"')), []).append(n)
    rules = {'kicad_default': (250, 200), 'Power': (500, 200), 'HV': (300, 500)}
    out = []
    for cls, (w, c) in rules.items():
        if cls in groups:
            out.append('(class %s "" %s\n      (circuit (use_via %s))\n      (rule (width %d) (clearance %d))\n    )'
                       % (cls, ' '.join(groups[cls]), via, w, c))
    txt = txt[:m.start()] + '\n    '.join(out) + txt[m.end():]
    # FreeRouting ignores KiCad's copper-to-edge clearance: route inside an inset outline
    e = int(EDGE_MIN * 1000)
    w, h = int(BW * 1000), int(BH * 1000)
    txt = re.sub(r'\(boundary\s*\(path pcb 0[^)]*\)\s*\)',
                 '(boundary (path pcb 0  %d %d  %d %d  %d %d  %d %d  %d %d))'
                 % (w - e, -(h - e), e, -(h - e), e, -e, w - e, -e, w - e, -(h - e)), txt, count=1)
    open(DSN, 'w').write(txt)


def import_ses(b, path):
    """KiCad 7's ImportSpecctraSES() only works inside the GUI; parse the session here."""
    sys.path.insert(0, HERE)
    from sexpr import parse, find, findall
    ses = parse(open(path).read())
    routes = find(ses, 'routes')
    res = find(routes, 'resolution')
    scale = 1.0 / (float(res[2]) * 1000.0)   # SES units -> mm  (um 10 -> 1e-4 mm)
    layers = {'F.Cu': pcbnew.F_Cu, 'B.Cu': pcbnew.B_Cu}
    existing = {(round(pcbnew.ToMM(t.GetPosition().x), 2), round(pcbnew.ToMM(t.GetPosition().y), 2))
                for t in b.GetTracks() if t.Type() == pcbnew.PCB_VIA_T}
    nw = nv = 0
    for net in findall(find(routes, 'network_out'), 'net'):
        ni = b.FindNet(str(net[1]))
        for w in findall(net, 'wire'):
            pth = find(w, 'path')
            layer, width = layers[str(pth[1])], float(pth[2]) * scale
            xy = [float(v) for v in pth[3:]]
            pts = [(xy[i] * scale, -xy[i + 1] * scale) for i in range(0, len(xy), 2)]
            for (xa, ya), (xb, yb) in zip(pts, pts[1:]):
                t = pcbnew.PCB_TRACK(b)
                t.SetStart(V(xa, ya))
                t.SetEnd(V(xb, yb))
                t.SetWidth(mm(width))
                t.SetLayer(layer)
                t.SetNet(ni)
                b.Add(t)
                nw += 1
        for v in findall(net, 'via'):
            x, y = float(v[2]) * scale, -float(v[3]) * scale
            if (round(x, 2), round(y, 2)) in existing:
                continue
            via = pcbnew.PCB_VIA(b)
            via.SetPosition(V(x, y))
            via.SetWidth(mm(0.6))
            via.SetDrill(mm(0.3))
            via.SetViaType(pcbnew.VIATYPE_THROUGH)
            via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            via.SetNet(ni)
            b.Add(via)
            nv += 1
    print('SES: %d track segments, %d vias imported' % (nw, nv))


EDGE_MIN = 0.0   # mm, optional routing-boundary inset (0 = board outline; FreeRouting keeps ~0.3 mm itself)
# later FreeRouting passes on the routed board (phase `export_routed`); the newest existing file wins
SES_EXTRA = [os.path.join(OUT, 'solbot_main_pass2.ses')]


def phase_finish():
    # always start from the unrouted board written by `place` (idempotent)
    shutil.copy(PLACED, PCB)
    b = pcbnew.LoadBoard(PCB)
    # each FreeRouting pass writes the complete routing -> import only the latest one
    if len(sys.argv) > 2:  # explicit session file
        sessions = [os.path.abspath(sys.argv[2])]
    else:
        sessions = [f for f in [SES] + SES_EXTRA if os.path.exists(f)]
    if sessions:
        import_ses(b, sessions[-1])
    g = Gen.__new__(Gen)
    g.board = b
    g.nets = {n: b.FindNet(n) for n in ('GND',)}
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        g.rect_zone(0.3, 0.3, BW - 0.3, BH - 0.3, layer, 'GND', prio=0)
    # GND stitching grid (skipped where it would collide; DRC tells)
    filler = pcbnew.ZONE_FILLER(b)
    filler.Fill(b.Zones())
    pcbnew.SaveBoard(PCB, b)
    rpt = os.path.join(OUT, 'drc_report.txt')
    pcbnew.WriteDRCReport(b, rpt, pcbnew.EDA_UNITS_MILLIMETRES, True)
    print('DRC report ->', rpt)


def phase_export_routed():
    """Export the routed board as DSN so FreeRouting can close remaining gaps."""
    global DSN
    b = pcbnew.LoadBoard(PCB)
    DSN = os.path.join(OUT, 'solbot_main_pass2.dsn')
    pcbnew.ExportSpecctraDSN(b, DSN)
    fix_dsn_classes()
    print('exported', DSN)


if __name__ == '__main__':
    {'place': phase_place, 'finish': phase_finish, 'export_routed': phase_export_routed}[sys.argv[1]]()
