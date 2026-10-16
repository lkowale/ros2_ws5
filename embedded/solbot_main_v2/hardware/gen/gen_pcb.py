"""Generate solbot_main_v2.kicad_pcb from netlist.json (written by gen_sch.py).

Phases:
  place   -> board outline, footprints, locked high-current copper, keepouts,
             exports solbot_main_v2.dsn for FreeRouting
  finish  -> imports solbot_main_v2.ses, adds GND pours + stitching, fills zones,
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
PCB = os.path.join(OUT, 'solbot_main_v2.kicad_pcb')
DSN = os.path.join(OUT, 'solbot_main_v2.dsn')
SES = os.path.join(OUT, 'solbot_main_v2.ses')
PLACED = os.path.join(HERE, 'placed.kicad_pcb')   # unrouted snapshot
FPDIRS = ['/usr/share/kicad/footprints', os.path.join(OUT, 'lib')]

BW, BH = 120.0, 85.0
ROLE = {1: 'TURN', 2: 'LIFT', 3: 'SLIDE', 4: 'SPARE'}


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
        self.via_exclude = []

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
            # symbol fields -> footprint properties (what "Update PCB from Schematic" does);
            # the JLC Fabrication Toolkit plugin reads 'LCSC' from here
            for k, v in p['fields'].items():
                fp.SetProperty(k, v)
            if p['fields'].get('Assembly'):  # hand-soldered: keep out of the JLC BOM / CPL
                fp.SetAttributes(fp.GetAttributes() | pcbnew.FP_EXCLUDE_FROM_BOM | pcbnew.FP_EXCLUDE_FROM_POS_FILES)
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

    def text(self, s, x, y, size=1.5, layer=pcbnew.F_SilkS, just='center', rot=0):
        t = pcbnew.PCB_TEXT(self.board)
        t.SetText(s)
        t.SetPosition(V(x, y))
        t.SetLayer(layer)
        t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
        t.SetTextThickness(mm(max(0.15, size * 0.15)))
        t.SetHorizJustify({'left': pcbnew.GR_TEXT_H_ALIGN_LEFT, 'right': pcbnew.GR_TEXT_H_ALIGN_RIGHT,
                           'center': pcbnew.GR_TEXT_H_ALIGN_CENTER}[just])
        if rot:
            t.SetTextAngleDegrees(rot)
        self.board.Add(t)

    def pin_labels(self, ref, labels, title, side, size=1.0, rot=0):
        """Silkscreen pin names next to each pad, outside the courtyard, and a title line.
        side = 'below' (connector on the top edge) or 'above' (connector on the bottom edge)."""
        l, t, r, b = self.bbox(ref)
        sgn = 1 if side == 'below' else -1
        edge = b if side == 'below' else t
        ypin = edge + sgn * (0.3 + (1.4 if rot else size * 0.6))
        for num, txt in labels.items():
            x, _ = self.pad_pos(ref, num)
            self.text(txt, x, ypin, size, rot=rot)
        cx = (l + r) / 2
        self.text(title, cx, ypin + sgn * ((1.6 if rot else size * 0.6) + 1.2), 1.2)

    # ------------------------------------------------------------- layout
    def layout(self):
        refs_in = lambda sheet, block: [r for r, (fp, p) in self.fps.items()
                                        if p['sheet'] == sheet and p['block'] == block]
        lib = lambda r: self.fps[r][1]['lib_id']
        val = lambda r: self.fps[r][1]['value']
        is_conn = lambda r: lib(r).startswith('Connector') or lib(r) == 'solbot:ESP32-S3-DevKitC-1'
        by_area = lambda refs: sorted(refs, key=lambda r: -self._area(r))
        conn_of = lambda sheet, block: [r for r in refs_in(sheet, block) if is_conn(r)][0]

        # ---- top edge connectors (wire entry towards the edge)
        top = [('power', 'IN12', 17), ('power', 'VMON', 29), ('power', 'SSR1', 39), ('power', 'SSR2', 49),
               ('mcu', 'ESTOP', 59), ('mcu', 'IMU', 77)]
        for sheet, block, x in top:
            self.edge_place(conn_of(sheet, block), x, 'top')
        c = lambda sh, bl: conn_of(sh, bl)
        self.pin_labels(c('power', 'IN12'), {1: '+12V', 2: 'GND'}, '12V IN', 'below')
        self.pin_labels(c('power', 'VMON'), {1: '+', 2: '-'}, 'BUS SENSE', 'below', 1.2)
        self.pin_labels(c('power', 'SSR1'), {1: '+', 2: '-'}, 'SSR1 PV', 'below', 1.2)
        self.pin_labels(c('power', 'SSR2'), {1: '+', 2: '-'}, 'SSR2 PV', 'below', 1.2)
        self.pin_labels(c('mcu', 'ESTOP'), {1: 'NC', 2: 'NC'}, 'E-STOP', 'below')
        self.pin_labels(c('mcu', 'IMU'), {1: '5V', 2: 'GND', 3: 'SCL', 4: 'SDA', 5: 'INT', 6: 'RST'},
                        'IMU BNO085', 'below', 0.9, rot=90)

        # ---- IBT-2 IDC sockets stacked on the right edge, pot terminals on the bottom edge
        for n in range(1, 5):
            refs = refs_in('actuators', 'A%d' % n)
            idc = [r for r in refs if val(r).startswith('IBT')][0]
            pos = [r for r in refs if val(r).startswith('POS')][0]
            self.place_bbox(idc, 90, right=BW - 0.6, top=12.5 + 11 * (n - 1))
            il, it, ir, ib = self.bbox(idc)
            self.text(ROLE[n], il - 0.8, (it + ib) / 2 - 0.9, 1.5, just='right')
            self.text('A%d' % n, il - 0.8, (it + ib) / 2 + 1.1, 1.0, just='right')
            self.edge_place(pos, 68 + 11.5 * (n - 1), 'bottom')
            self.pin_labels(pos, {1: '+', 2: 'W', 3: '-'}, ROLE[n] + ' POS', 'above', 1.2)
        for k, line in enumerate(['IBT-2 IDC 2x5:  1 RPWM  2 LPWM', '3,4 EN  5,6 IS  7 +5V', '8 GND  9,10 n.c.',
                                  'A1 TURN A2 LIFT A3 SLIDE A4 SPARE']):
            self.text(line, 81, 2.4 + 1.8 * k, 0.9, just='left')
        # connector designators would sit on top of the pin labels: hide them
        for ref, (fp, p) in self.fps.items():
            if p['lib_id'].startswith('Connector'):
                fp.Reference().SetVisible(False)

        # ---- ESP32-S3 DevKit along the bottom-left, antenna at the left edge
        esp = [r for r in refs_in('mcu', 'ESP') if lib(r) == 'solbot:ESP32-S3-DevKitC-1'][0]
        self.place_bbox(esp, 90, left=0.6, bottom=BH - 0.6)
        l, t, r, b = self.bbox(esp)
        pin1x = self.pad_pos(esp, 1)[0]
        self.rect_zone(0, t, pin1x - 1.5, BH, pcbnew.F_Cu, keepout=True)
        self.rect_zone(0, t, pin1x - 1.5, BH, pcbnew.B_Cu, keepout=True)
        self.text('antenna', l + 0.8, t + 2, 1.0, just='left')
        self.text('program via UART USB port', (l + r) / 2, t - 1.2, 1.0)

        for i, ref in enumerate(refs_in('mcu', 'MECH')):
            self.place(ref, *[(3.5, 3.5), (BW - 3.5, 3.5), (3.5, 50.0), (BW - 3.5, BH - 3.5)][i])

        # ---- small parts
        power = [r for blk in ('IN12', 'LDO5', 'LDO3', 'VMON', 'SSR1', 'SSR2')
                 for r in refs_in('power', blk) if not is_conn(r)]
        self.pack(by_area(power), 7, 16.5, 46, 49.5, gap=2.4)
        rest = [r for blk in ('BUF', 'A1', 'A2', 'A3', 'A4') for r in refs_in('actuators', blk)
                if not is_conn(r)]
        rest += [r for blk in ('ESP', 'ESTOP', 'IMU') for r in refs_in('mcu', blk) if not is_conn(r)]
        self.pack(by_area(rest), 48, 15.5, 84, 54.5, gap=1.4)

        missing = [r for r, (fp, p) in self.fps.items() if not p.get('_placed')]
        if missing:
            # overflow area above the pot terminals
            self.pack(by_area(missing), 66, 56, 94, 73, gap=1.4)

    def place_bbox(self, ref, rot, left=None, right=None, top=None, bottom=None):
        fp = self.fps[ref][0]
        fp.SetOrientationDegrees(rot)
        fp.SetPosition(V(50, 50))
        l, t, r, b = self.bbox(ref)
        dx = (left - l) if left is not None else (right - r) if right is not None else 0
        dy = (top - t) if top is not None else (bottom - b) if bottom is not None else 0
        fp.SetPosition(V(50 + dx, 50 + dy))
        self.fps[ref][1]['_placed'] = True

    def edge_place(self, ref, x, edge):
        """Connector body against the top/bottom board edge, wire entry facing out."""
        rot = 180 if edge == 'top' else 0
        fp = self.fps[ref][0]
        fp.SetOrientationDegrees(rot)
        fp.SetPosition(V(x, BH / 2))
        l, t, r, b = self.bbox(ref)
        dy = (0.5 - t) if edge == 'top' else (BH - 0.5 - b)
        fp.SetPosition(V(x, BH / 2 + dy))
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
    g.text('solbot_main v2 rev B  ESP32-S3 controller  2026-10-16', 60, 40, 2.0, pcbnew.B_SilkS)
    pcbnew.SaveBoard(PCB, g.board)
    shutil.copy(PCB, PLACED)
    # reload so project net classes are applied, then export DSN
    b = pcbnew.LoadBoard(PCB)
    ok = pcbnew.ExportSpecctraDSN(b, DSN)
    fix_dsn_classes()
    print('saved', PCB, 'DSN export', ok)


HV_SUFFIX = ('_IN', '_SH', '_COM', '_G', '_VM', '_SNP', '_SNN')
HV_NETS = {'BUS_IN', 'BUS_VM'}
POWER_NETS = {'+5V', '5V_ESP', '+12V', '12V_RAW', '12V_F'}  # GND is poured; +3V3A must reach 0.5 mm-pitch pads


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
SES_EXTRA = [os.path.join(OUT, 'solbot_main_v2_pass2.ses')]


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
    # heat-spreading copper under the linear regulators' tabs (tab = output)
    for fp in b.GetFootprints():
        if fp.GetValue().startswith('AMS1117'):
            tab = max((p for p in fp.Pads() if p.GetNumber() == '2'), key=lambda p: p.GetSize().x * p.GetSize().y)
            x, y = pcbnew.ToMM(tab.GetPosition().x), pcbnew.ToMM(tab.GetPosition().y)
            g.nets[tab.GetNetname()] = tab.GetNet()
            z = g.rect_zone(x - 5.5, y - 4.5, x + 5.5, y + 4.5, pcbnew.F_Cu, tab.GetNetname(), prio=1)
            z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
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
    DSN = os.path.join(OUT, 'solbot_main_v2_pass2.dsn')
    pcbnew.ExportSpecctraDSN(b, DSN)
    fix_dsn_classes()
    print('exported', DSN)


if __name__ == '__main__':
    {'place': phase_place, 'finish': phase_finish, 'export_routed': phase_export_routed}[sys.argv[1]]()
