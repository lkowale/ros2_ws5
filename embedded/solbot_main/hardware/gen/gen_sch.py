"""Generate the KiCad 7 hierarchical schematic from design.py.

Connectivity is label based: every pin gets a short wire stub and a net label
(global label when the net spans several sheets). Parts are grouped into
framed functional blocks with an explanatory note.
"""
import json
import math
import os
import textwrap
import uuid

from design import build, SHEETS
from kicadlib import sym
from sexpr import dump, Sym

PROJECT = 'solbot_main'
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.abspath(os.path.join(HERE, '..'))
NS = uuid.UUID('6f1c2b1e-5a1b-4c0d-8e7f-50b0750b0700')
DATE = '2026-10-15'
REV = 'v4.0'
G = 2.54
STUB = 2.54
PAPERS = [('A3', 420, 297), ('A2', 594, 420), ('A1', 841, 594), ('A0', 1189, 841)]


def uid(*a):
    return str(uuid.uuid5(NS, '/'.join(map(str, a))))


ROOT = uid('root')
S = Sym


def snapg(v, g=G):
    return round(v / g) * g


def rot_pt(x, y, rot):
    r = math.radians(rot)
    c, s = round(math.cos(r)), round(math.sin(r))
    return x * c - y * s, x * s + y * c


def font(size=1.27):
    return [S('font'), [S('size'), size, size]]


def label_len(net, glob):
    return len(net) * 1.1 + (6 if glob else 2)


class Placed:
    def __init__(self, part):
        self.p = part
        self.si = sym(part.lib_id)
        vis = [pp for pp in self.si.pins if not pp['hidden']]
        two_vertical = len(vis) == 2 and all(abs(pp['x']) < 0.01 for pp in vis)
        self.rot = 90 if two_vertical else 0
        self.pins = self._resolve()
        self._bbox()

    def _resolve(self):
        p, si = self.p, self.si
        net_of = {}
        for key, net in p.conns.items():
            hits = si.pin(key)
            if not hits:
                raise KeyError('%s (%s): no pin %r' % (p.ref, p.lib_id, key))
            for h in hits:
                net_of[h['num']] = net
        for key in p.nc:
            hits = si.pin(key)
            if not hits:
                raise KeyError('%s: no pin %r for NC' % (p.ref, key))
            for h in hits:
                net_of[h['num']] = None
        out = []
        for pp in si.pins:
            if pp['num'] not in net_of:
                if pp['hidden']:
                    # stacked hidden pin: inherits the net of the visible pin at same spot
                    twin = [q for q in si.pins if not q['hidden'] and q['x'] == pp['x'] and q['y'] == pp['y']]
                    if twin and twin[0]['num'] in net_of:
                        net_of[pp['num']] = net_of[twin[0]['num']]
                    elif pp['type'] == 'no_connect':
                        net_of[pp['num']] = None
                    else:
                        raise ValueError('%s: hidden pin %s unresolved' % (p.ref, pp['num']))
                else:
                    raise ValueError('%s (%s): pin %s/%s not connected and not NC'
                                     % (p.ref, p.lib_id, pp['num'], pp['name']))
            tx, ty = rot_pt(pp['x'], pp['y'], self.rot)
            out.append(dict(pp, net=net_of[pp['num']], sx=tx, sy=-ty,
                            out=(pp['ang'] + self.rot + 180) % 360))
        return out

    def _bbox(self, glob=None):
        xs, ys = [0.0], [0.0]
        for pp in self.pins:
            xs.append(pp['sx'])
            ys.append(pp['sy'])
            if pp['hidden'] or pp['net'] is None:
                continue
            L = STUB + label_len(pp['net'], True)
            dx, dy = math.cos(math.radians(pp['out'])), -math.sin(math.radians(pp['out']))
            xs.append(pp['sx'] + dx * L)
            ys.append(pp['sy'] + dy * L)
        bx0, bx1 = min(xs) - 2 * G, max(xs) + 2 * G
        by0, by1 = min(ys) - 3 * G, max(ys) + 2 * G   # extra top room for ref/value
        self.bb = (bx0, by0, bx1, by1)
        self.w, self.h = bx1 - bx0, by1 - by0


def layout_block(placed, max_w=260.0):
    """Row-pack parts; returns block size and sets .x/.y relative to block origin."""
    x = y = 0.0
    row_h = 0.0
    width = 0.0
    for pl in placed:
        if x > 0 and x + pl.w > max_w:
            x = 0.0
            y += row_h
            row_h = 0.0
        pl.x = snapg(x - pl.bb[0])
        pl.y = snapg(y - pl.bb[1])
        x += pl.w
        row_h = max(row_h, pl.h)
        width = max(width, x)
    return width, y + row_h


def pack_blocks(blocks, avail_w):
    """Shelf-pack blocks (each with .bw/.bh). Returns total height."""
    x = y = 0.0
    shelf_h = 0.0
    for b in blocks:
        if x > 0 and x + b.bw > avail_w:
            x = 0.0
            y += shelf_h + 8
            shelf_h = 0.0
        b.bx, b.by = x, y
        x += b.bw + 8
        shelf_h = max(shelf_h, b.bh)
    return y + shelf_h


class Block:
    pass


def title_block(title, page_comment=''):
    return [S('title_block'), [S('title'), title], [S('date'), DATE], [S('rev'), REV],
            [S('company'), 'solbot'], [S('comment'), 1, 'Main controller - ESP32-S3'],
            [S('comment'), 2, page_comment]]


def text_item(txt, x, y, size=1.5, bold=False):
    eff = [S('effects'), font(size) + ([S('bold')] if bold else []), [S('justify'), S('left'), S('bottom')]]
    return [S('text'), txt, [S('at'), x, y, 0], eff, [S('uuid'), uid('text', txt, x, y)]]


def rect_poly(x0, y0, x1, y1, key):
    return [S('polyline'), [S('pts'), [S('xy'), x0, y0], [S('xy'), x1, y0], [S('xy'), x1, y1],
                            [S('xy'), x0, y1], [S('xy'), x0, y0]],
            [S('stroke'), [S('width'), 0], [S('type'), S('dash')]], [S('uuid'), uid('frame', key)]]


def label_item(net, x, y, out_ang, glob, key):
    ang = int(out_ang) % 360
    just = S('left') if ang in (0, 90) else S('right')
    if glob:
        return [S('global_label'), net, [S('shape'), S('bidirectional')], [S('at'), x, y, ang],
                [S('fields_autoplaced')],
                [S('effects'), font(), [S('justify'), just]],
                [S('uuid'), uid('glabel', key)],
                [S('property'), 'Intersheetrefs', '${INTERSHEET_REFS}', [S('at'), x, y, 0],
                 [S('effects'), font(), S('hide')]]]
    return [S('label'), net, [S('at'), x, y, ang], [S('fields_autoplaced')],
            [S('effects'), font(), [S('justify'), just, S('bottom')]], [S('uuid'), uid('label', key)]]


def symbol_instance(pl, sheet_uuid):
    p = pl.p
    X, Y = pl.x, pl.y
    bx0, by0, bx1, by1 = pl.bb
    su = uid('sym', p.ref)
    props = [
        ('Reference', p.ref, X + bx0 + 2 * G, Y + by0 + 2.0, False),
        ('Value', p.value, X + bx0 + 2 * G, Y + by0 + 4.5, False),
        ('Footprint', p.footprint, X, Y, True),
        ('Datasheet', pl.si.props.get('Datasheet', '~'), X, Y, True),
    ]
    for k, v in p.fields.items():
        props.append((k, v, X, Y, True))
    e = [S('symbol'), [S('lib_id'), p.lib_id], [S('at'), X, Y, pl.rot], [S('unit'), 1],
         [S('in_bom'), S('yes')], [S('on_board'), S('yes')], [S('dnp'), S('yes') if p.dnp else S('no')],
         [S('uuid'), su]]
    for name, val, x, y, hide in props:
        eff = [S('effects'), font(), [S('justify'), S('left')]]
        if hide:
            eff.append(S('hide'))
        e.append([S('property'), name, val, [S('at'), round(x, 2), round(y, 2), pl.rot], eff])
    for pp in pl.pins:
        e.append([S('pin'), pp['num'], [S('uuid'), uid('pin', p.ref, pp['num'])]])
    e.append([S('instances'), [S('project'), PROJECT,
                               [S('path'), '/%s/%s' % (ROOT, sheet_uuid), [S('reference'), p.ref], [S('unit'), 1]]]])
    return e, su


def pin_items(pl, global_nets):
    items = []
    for pp in pl.pins:
        if pp['hidden']:
            continue
        px, py = round(pl.x + pp['sx'], 3), round(pl.y + pp['sy'], 3)
        key = '%s.%s' % (pl.p.ref, pp['num'])
        if pp['net'] is None:
            items.append([S('no_connect'), [S('at'), px, py], [S('uuid'), uid('nc', key)]])
            continue
        dx, dy = round(math.cos(math.radians(pp['out']))), -round(math.sin(math.radians(pp['out'])))
        ex, ey = round(px + dx * STUB, 3), round(py + dy * STUB, 3)
        items.append([S('wire'), [S('pts'), [S('xy'), px, py], [S('xy'), ex, ey]],
                      [S('stroke'), [S('width'), 0], [S('type'), S('default')]], [S('uuid'), uid('wire', key)]])
        items.append(label_item(pp['net'], ex, ey, pp['out'], pp['net'] in global_nets, key))
    return items


def wrap(note, width=95):
    return '\n'.join(textwrap.wrap(note, width))


def gen_sheet(name, desc, placed_by_block, notes, global_nets, page):
    sheet_uuid = uid('sheet', name)
    blocks = []
    for bname, pls in placed_by_block:
        b = Block()
        b.name, b.pls = bname, pls
        w, h = layout_block(pls)
        b.note = wrap(notes.get((name, bname), ''))
        nlines = b.note.count('\n') + 1 if b.note else 0
        b.head = 7 + nlines * 2.6
        b.bw, b.bh = max(w, 120), h + b.head + 4
        blocks.append(b)
    for pname, pw, ph in PAPERS:
        total_h = pack_blocks(blocks, pw - 30)
        if total_h <= ph - 60:
            break
    body = []
    symbols = []
    ox, oy = 15.24, 15.24
    lib_ids = set()
    for b in blocks:
        bx, by = snapg(ox + b.bx), snapg(oy + b.by)
        body.append(rect_poly(bx - 2, by - 2, bx + b.bw - 2, by + b.bh - 2, name + b.name))
        body.append(text_item(b.name, bx, by + 4, 2.5, True))
        if b.note:
            body.append(text_item(b.note, bx, by + b.head - 2, 1.4))
        for pl in b.pls:
            pl.x = snapg(pl.x + bx)
            pl.y = snapg(pl.y + by + b.head)
            e, su = symbol_instance(pl, sheet_uuid)
            pl.sym_uuid = su
            pl.sheet_uuid = sheet_uuid
            symbols.append(e)
            body.extend(pin_items(pl, global_nets))
            lib_ids.add(pl.p.lib_id)
    libsyms = [sym(l).sexpr for l in sorted(lib_ids)]
    doc = [S('kicad_sch'), [S('version'), 20230121], [S('generator'), S('eeschema')],
           [S('uuid'), uid('file', name)], [S('paper'), pname],
           title_block(desc, 'Sheet: ' + name), [S('lib_symbols')] + libsyms] + body + symbols
    open(os.path.join(OUT, name + '.kicad_sch'), 'w').write(dump(doc) + '\n')
    return sheet_uuid, pname


ROOT_NOTES = """solbot_main v4 - ESP32-S3 main controller

ESP32-S3-DevKitC-1 GPIO map
  ADC1 (IBT-2 IS current):  GPIO1=A1  GPIO2=A2  GPIO4=A3  GPIO5=A4  GPIO6=A5  GPIO7=A6
  PWM R/L:  A1=9/10  A2=11/12  A3=13/14  A4=15/16  A5=17/18  A6=21/47
  I2C bus A (IMU + actuator ADS1115 0x48/0x49): SDA=41 SCL=42
  I2C bus B (power ADS1115 0x48/0x49/0x4A, TCA9535 0x20, ext): SDA=19 SCL=20  (USB-OTG port unusable)
  ACT_EN=39 (through E-stop loop)  ESTOP_SENSE=8  PV_EN=40  IMU_INT=3  LED_ERR=46
  TCA9535: P00..P03=BAT1..4_EN  P04=IMU_RST  P05=PWR LED  P06..P17=EXP_IO0..9

Power: up to 4 packs (populated: BAT1, BAT2) 10S..20S + PV, all channels rated 100 V
  (FETs -100 V: for 20S use PV Voc <= 90 V; TVS SMCJ58A is for 10S - use SMCJ85A for 20S).
  Pack fuses are external (fit one per pack, close to the pack).
"""


def gen_root(sheet_info):
    items = [text_item(ROOT_NOTES, 20, 25, 2.0)]
    x, y = 20.32, 120.0
    for i, (name, desc, su, pname) in enumerate(sheet_info):
        sx, sy = x + (i % 2) * 110, y + (i // 2) * 40
        items.append([S('sheet'), [S('at'), sx, sy], [S('size'), 90, 25], [S('fields_autoplaced')],
                      [S('stroke'), [S('width'), 0.1524], [S('type'), S('solid')]],
                      [S('fill'), [S('color'), 0, 0, 0, 0.0]], [S('uuid'), su],
                      [S('property'), 'Sheetname', name, [S('at'), sx, sy - 0.7, 0],
                       [S('effects'), font(), [S('justify'), S('left'), S('bottom')]]],
                      [S('property'), 'Sheetfile', name + '.kicad_sch', [S('at'), sx, sy + 25.6, 0],
                       [S('effects'), font(), [S('justify'), S('left'), S('top')]]],
                      [S('instances'), [S('project'), PROJECT, [S('path'), '/' + ROOT, [S('page'), str(i + 2)]]]]])
        items.append(text_item(wrap(desc, 40), sx + 3, sy + 12, 2.0))
    doc = [S('kicad_sch'), [S('version'), 20230121], [S('generator'), S('eeschema')],
           [S('uuid'), ROOT], [S('paper'), 'A3'],
           title_block('solbot_main v4 - ESP32-S3 main controller', 'Root sheet'),
           [S('lib_symbols')]] + items + [[S('sheet_instances'), [S('path'), '/', [S('page'), '1']]]]
    open(os.path.join(OUT, PROJECT + '.kicad_sch'), 'w').write(dump(doc) + '\n')


def main():
    d = build()
    net_sheets = {}
    for p in d.parts:
        for net in p.conns.values():
            net_sheets.setdefault(net, set()).add(p.sheet)
    global_nets = {n for n, s in net_sheets.items() if len(s) > 1}
    placed = {}
    by_sheet = {}
    for p in d.parts:
        pl = Placed(p)
        placed[p.ref] = pl
        lst = by_sheet.setdefault(p.sheet, [])
        if not lst or lst[-1][0] != p.block:
            lst.append((p.block, []))
        lst[-1][1].append(pl)
    info = []
    for i, (name, desc) in enumerate(SHEETS):
        su, pname = gen_sheet(name, desc, by_sheet[name], d.block_notes, global_nets, i + 2)
        info.append((name, desc, su, pname))
        print('sheet %-14s %-3s %3d parts' % (name, pname, sum(len(x[1]) for x in by_sheet[name])))
    gen_root(info)
    # netlist for the PCB generator
    nl = []
    for p in d.parts:
        pl = placed[p.ref]
        pads = {pp['num']: pp['net'] for pp in pl.pins if pp['net'] is not None}
        nl.append(dict(ref=p.ref, value=p.value, footprint=p.footprint, lib_id=p.lib_id,
                       sheet=p.sheet, block=p.block, dnp=p.dnp, fields=p.fields,
                       path='/%s/%s' % (pl.sheet_uuid, pl.sym_uuid), pads=pads))
    json.dump(nl, open(os.path.join(HERE, 'netlist.json'), 'w'), indent=1)
    nets = sorted({n for p in nl for n in p['pads'].values()})
    print(len(d.parts), 'parts,', len(nets), 'nets,', len(global_nets), 'global')
    return d


if __name__ == '__main__':
    main()
